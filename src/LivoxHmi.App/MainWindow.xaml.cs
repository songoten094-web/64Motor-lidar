using HelixToolkit;
using HelixToolkit.SharpDX;
using HelixToolkit.Wpf.SharpDX;
using HelixToolkit.SharpDX.Core;
using HelixToolkit.Maths;
using HelixColor4 = HelixToolkit.Maths.Color4;
using LivoxHmi.Core;
using LivoxHmi.Device;
using WaveMotionControl.Services;
using WaveMotionControl.State;
using WaveMotionControl.UI;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using System.Windows;
using System.Windows.Media;
using MediaPoint3D = System.Windows.Media.Media3D.Point3D;
using MediaVector3D = System.Windows.Media.Media3D.Vector3D;
using Vector3D = System.Windows.Media.Media3D.Vector3D;
using System.Windows.Threading;
namespace LivoxHmi.App;
using WpfMessageBox = System.Windows.MessageBox;
using WpfBrushes = System.Windows.Media.Brushes;
public partial class MainWindow : Window
{
    public DefaultEffectsManager EffectsManager { get; } = new();

    // WaveMotion stays in the same process/HMI. The existing RS485 service is reused intact;
    // the integration layer only routes LIDAR rising-edge events into it.
    private readonly ApplicationState _waveState = new();
    private Em2RsModbusService _waveService = null!;
    private ShellForm? _waveShell;
    private ZoneFastEffectRouter? _zoneFastRouter;

    private readonly LivoxDeviceService _livox = new();
    private readonly SimulatedPointCloudSource _simulator = new();
    private readonly ProjectStore _projectStore = new();
    private readonly ZoneEngine _zoneEngine = new();
    private readonly CoordinateEngine _coordinateEngine = new();
    private StaticTouchDetector _touchDetector = null!;
    private readonly StaticMapBuilder _staticMapBuilder = new();
    private readonly StaticMapStore _staticMapStore = new();
    private StaticMapSnapshot? _staticMap;
    // Static map is an editing reference only. Normal runtime always stays LIVE.
    // Surface/Zone geometry remains in MID360_SENSOR and is rendered over either view.
    private bool _surfaceDesignMode;

    private ProjectDefinition _project;
    private IPointCloudSource? _activeSource;
    private CancellationTokenSource? _cts;
    private Task? _acquisitionTask;
    private Task? _detectionTask;
    private PointCloudFrame? _latestFrame; // viewer mailbox; RenderTimer consumes it
    private PointCloudFrame? _latestDetectionFrame; // detection mailbox; independent from viewer consumption
    private IReadOnlyList<TouchEvidence> _latestTouchEvidence = Array.Empty<TouchEvidence>();
    private long _frameNumber;
    private long _lastFrameReceivedTicks;
    private readonly Stopwatch _renderClock = Stopwatch.StartNew();
    private readonly DispatcherTimer _renderTimer;
    private readonly DispatcherTimer _healthTimer;
    private readonly DispatcherTimer _imuTimer;
    private bool _isShuttingDown;
    private bool _loaded;
    private bool _panActive;
    private System.Windows.Point _panLastPoint;
    private long _lastInspectorTicks;
    private long _lastTagUiTicks;
    private const int TagUiIntervalMs = 500; // telemetry UI only: 2 Hz; point processing remains realtime
    private Point3D? _lastPickedWorldPoint; // display/view coordinate only
    private Point3D? _lastPickedSensorPoint;
    private LivoxImuSnapshot? _lastImuSnapshot;
    private Vector3? _calibratedZAxisSensor;
    private bool _setX0DirectionMode;
    private Vector3 _x0PreviewDirectionProject = Vector3.UnitX;

    // Viewer-only temporal persistence. MID-360 uses a non-repeating scan pattern,
    // so replacing the whole scene every 50 ms can look like flicker even though
    // acquisition is healthy. Processing still consumes the newest frame only;
    // only the visualizer keeps a short bounded history.
    private readonly Queue<PointCloudFrame> _renderHistory = new();
    private const int RenderPersistenceFrames = 3;      // bounded viewer history
    private const int MaxRenderPoints = 32_000;         // hard GPU/UI budget

    // Surface Editor: picked points are ALWAYS stored in raw MID-360 SENSOR coordinates.
    private readonly List<Vector3> _surfacePickPoints = new(3);
    private bool _surfacePickMode;

    // Direct Zone drawing: interaction happens on the selected Surface, but
    // persisted geometry is always Surface-local UV backed by MID360_SENSOR.
    private enum ZoneDrawMode { None, Rectangle, Polygon }
    private ZoneDrawMode _zoneDrawMode;
    private readonly List<UvPoint> _zoneDrawUv = new();
    private UvPoint? _zoneDrawHoverUv;

    private string _colorMode = "Uniform";
    private const float DefaultGridSizeMeters = 10f;
    private const float GridStepMeters = 0.5f;

    // MID-360 point-cloud coordinate frame. Units are meters in our internal frame.
    // The origin O=(0,0,0) is the sensor point-cloud origin.
    private const float WorldAxisLengthMeters = 2.5f;
    private const float WorldAxisArrowLengthMeters = 0.16f;
    private const float WorldAxisArrowRadiusMeters = 0.035f;

    public MainWindow()
    {
        InitializeComponent();
        DataContext = this;
        InitializeWaveMotionModule();

        _project = CreateDefaultProject();
        InitializeTouchRuntime();
        _coordinateEngine.Configure(_project.Calibration);
        RestoreCoordinateCalibrationState();
        SurfaceList.ItemsSource = _project.Surfaces;
        RefreshZones();

        // Draw the persistent world coordinate frame at the MID-360 origin.
        UpdateWorldCoordinateFrame();
        UpdateWorldGrid();
        ConfigureStaticGeometryMaterials();
        ApplyViewerMode();

        // Register the 3D picking handler in code-behind. This avoids WPF XAML
        // event-signature resolution issues across HelixToolkit 3.x builds.
        Viewport.MouseMove += Viewport_MouseMove;
        Viewport.MouseDown += Viewport_MouseDown;
        Viewport.MouseUp += Viewport_MouseUp;
        KeyDown += MainWindow_KeyDown;

        _renderTimer = new DispatcherTimer(DispatcherPriority.Render)
        {
            Interval = TimeSpan.FromMilliseconds(50) // UI render ceiling: 20 FPS.
        };
        _renderTimer.Tick += RenderTimer_Tick;
        _renderTimer.Start();

        _healthTimer = new DispatcherTimer(DispatcherPriority.Background)
        {
            Interval = TimeSpan.FromSeconds(2)
        };
        _healthTimer.Tick += HealthTimer_Tick;
        _healthTimer.Start();

        _imuTimer = new DispatcherTimer(DispatcherPriority.Background)
        {
            Interval = TimeSpan.FromMilliseconds(500)
        };
        _imuTimer.Tick += ImuTimer_Tick;
        _imuTimer.Start();

        Loaded += MainWindow_Loaded;
        Closed += MainWindow_Closed;
    }


    private void InitializeWaveMotionModule()
    {
        _waveService = new Em2RsModbusService(_waveState);
        _zoneFastRouter = ZoneFastEffectRouter.Load(
            _waveService,
            _waveState,
            Path.GetFullPath("integration/zone_motor_map.json"));

        // Host the original WaveMotion WinForms HMI inside the WPF tab.
        // This preserves its existing Auto/Manual/Settings UI and RS485 service flow.
        _waveShell = new ShellForm(_waveState, _waveService)
        {
            TopLevel = false,
            FormBorderStyle = System.Windows.Forms.FormBorderStyle.None,
            Dock = System.Windows.Forms.DockStyle.Fill,
            WindowState = System.Windows.Forms.FormWindowState.Normal
        };
        WaveMotionHost.Child = _waveShell;
        _waveShell.Show();
    }

    private void InitializeTouchRuntime()
    {
        _touchDetector = new StaticTouchDetector { Enabled = true };
        _project.Detection.DetectionFps = 18;
        _project.Detection.ConfirmFrames = 7;
        _project.Detection.ReleaseFrames = 7;
    }

    private async void MainWindow_Loaded(object sender, RoutedEventArgs e)
    {
        Loaded -= MainWindow_Loaded;
        _loaded = true;
        OcctStatusText.Text = SurfaceGeometry.IsOcctAvailable(out var occtStatus) ? $"OCCT: READY | {occtStatus}" : $"OCCT: NOT READY | {occtStatus}";

        try
        {
            var projectPath = Path.GetFullPath("projects/Machine_A.project.json");
            if (File.Exists(projectPath))
            {
                _project = await _projectStore.LoadAsync(projectPath).ConfigureAwait(true);
                InitializeTouchRuntime();
                _coordinateEngine.Configure(_project.Calibration);
                RestoreCoordinateCalibrationState();
                SurfaceList.ItemsSource = _project.Surfaces;
                RefreshZones();
                if (_project.Surfaces.Count > 0)
                {
                    SurfaceList.SelectedIndex = 0;
                    RenderSelectedSurface();
                }
                ProjectStatus.Text = $"{_project.Name} (loaded)";
                await TryLoadStaticMapAsync(showWarning: false).ConfigureAwait(true);
            }

            SystemStatus.Text = "SYSTEM: IDLE";
            DiagnosticStatus.Text = "No source selected - Simulator is ready";
        }
        catch (Exception ex)
        {
            SystemStatus.Text = "SYSTEM: PROJECT LOAD FAULT";
            DiagnosticStatus.Text = "Using default project";
            WpfMessageBox.Show($"The HMI opened, but the saved project could not be loaded.\n\n{ex}",
                "Livox HMI - Project load warning", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
    }

    private static ProjectDefinition CreateDefaultProject() => new()
    {
        Id = "MACHINE_A",
        Name = "Machine A",
        Mid360ConfigPath = "config/mid360_config.json",
        Surfaces =
        {
            new SurfaceDefinition
            {
                Id = "S01", Name = "Surface 01",
                Origin = new Point3D(0, 0, 0, 0, 0),
                Normal = new Point3D(0, 0, 1, 0, 0),
                UAxis = new Point3D(1, 0, 0, 0, 0),
                VAxis = new Point3D(0, 1, 0, 0, 0)
            }
        },
        Zones =
        {
            new ZoneDefinition
            {
                Id = "Z01", Name = "Zone 01", SurfaceId = "S01",
                Polygon = new List<UvPoint> { new(0,0), new(0.5,0), new(0.5,0.5), new(0,0.5) }
            }
        }
    };

    private async void SimulatorButton_Click(object sender, RoutedEventArgs e)
    {
        if (_activeSource == _simulator)
        {
            await StopSourceAsync();
            return;
        }

        if (_activeSource is not null)
            await StopSourceAsync();

        try
        {
            ApplySimulatorSettings();
            _simulator.Connect(string.Empty);
            StartAcquisition(_simulator);
            SimulatorButton.Content = "Stop Simulator";
            SourceStatus.Text = "Source: Simulator";
            SourceStatus.Foreground = WpfBrushes.LightGreen;
            SystemStatus.Text = "SYSTEM: SIMULATOR RUNNING";
            DiagnosticStatus.Text = "Bounded latest-frame pipeline";
        }
        catch (Exception ex)
        {
            SystemStatus.Text = "SYSTEM: SIMULATOR FAULT";
            DiagnosticStatus.Text = ex.Message;
            WpfMessageBox.Show(ex.ToString(), "Simulator error", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private async void LivoxButton_Click(object sender, RoutedEventArgs e)
    {
        if (_activeSource == _livox)
        {
            await StopSourceAsync();
            return;
        }

        if (_activeSource is not null)
            await StopSourceAsync();

        try
        {
            var config = Path.GetFullPath(_project.Mid360ConfigPath);
            _livox.Connect(config);
            Volatile.Write(ref _lastFrameReceivedTicks, Stopwatch.GetTimestamp());
            StartAcquisition(_livox);
            LivoxButton.Content = "Stop MID-360";
            SourceStatus.Text = "Source: Livox MID-360";
            SourceStatus.Foreground = WpfBrushes.LightGreen;
            SystemStatus.Text = "SYSTEM: LIVOX RUNNING";
            DiagnosticStatus.Text = $"Waiting for point cloud | Native API {_livox.NativeApiVersion}";
        }
        catch (Exception ex)
        {
            SystemStatus.Text = "SYSTEM: LIVox FAULT";
            DiagnosticStatus.Text = ex.Message;
            WpfMessageBox.Show(ex.ToString(), "Livox connection error",
                MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void StartAcquisition(IPointCloudSource source)
    {
        _activeSource = source;
        _renderHistory.Clear();
        PointCloudModel.IsHitTestVisible = true;
        _cts = new CancellationTokenSource();
        _acquisitionTask = Task.Run(() => AcquisitionLoopAsync(source, _cts.Token));
        _detectionTask = Task.Run(() => DetectionLoopAsync(source, _cts.Token));
    }

    private async Task AcquisitionLoopAsync(IPointCloudSource source, CancellationToken ct)
    {
        while (!ct.IsCancellationRequested && ReferenceEquals(_activeSource, source))
        {
            try
            {
                if (source.TryGetFrame(out var sensorFrame) && sensorFrame is not null)
                {
                    // SENSOR FRAME is the single source of truth for geometry/detection.
                    // IMU / X0 alignment is display-only and is applied later by RenderTimer.
                    Volatile.Write(ref _lastFrameReceivedTicks, Stopwatch.GetTimestamp());
                    Interlocked.Exchange(ref _latestFrame, sensorFrame);
                    Interlocked.Exchange(ref _latestDetectionFrame, sensorFrame);

                    // Static-map geometry is accumulated ONLY in raw MID-360 SENSOR XYZ.
                    // The builder is bounded and never receives the IMU/view transform.
                    if (_staticMapBuilder.IsBuilding)
                        _staticMapBuilder.AddFrame(sensorFrame);

                }

                await Task.Delay(2, ct).ConfigureAwait(false);
            }
            catch (OperationCanceledException) { break; }
            catch (Exception ex)
            {
                if (_isShuttingDown) break;
                try
                {
                    await Dispatcher.InvokeAsync(() =>
                    {
                        SystemStatus.Text = "SYSTEM: PROCESSING FAULT";
                        DiagnosticStatus.Text = ex.Message;
                    });
                    await Task.Delay(100, ct).ConfigureAwait(false);
                }
                catch (OperationCanceledException) { break; }
            }
        }
    }

    private async Task DetectionLoopAsync(IPointCloudSource source, CancellationToken ct)
    {
        var effectiveDetectionFps = Math.Clamp(_project.Detection.DetectionFps, 1, 20);
        var periodMs = Math.Max(20, 1000 / effectiveDetectionFps);
        PointCloudFrame? lastProcessed = null;
        long lastUiTicks = 0;

        while (!ct.IsCancellationRequested && ReferenceEquals(_activeSource, source))
        {
            try
            {
                var frame = Volatile.Read(ref _latestDetectionFrame);
                if (frame is null || ReferenceEquals(frame, lastProcessed))
                {
                    await Task.Delay(2, ct).ConfigureAwait(false);
                    continue;
                }
                lastProcessed = frame;
                var cycleStart = Stopwatch.GetTimestamp();

                // Static-map differential is restricted to Zone spatial cells and confirmed by ZoneEngine.
                var candidates = _touchDetector.Detect(frame, _project, _staticMap, ct);
                Interlocked.Exchange(ref _latestTouchEvidence, candidates);
                var events = _zoneEngine.Update(candidates, _project, frame.Timestamp);

                var now = Stopwatch.GetTimestamp();
                if (lastUiTicks == 0 || (now - lastUiTicks) * 1000.0 / Stopwatch.Frequency >= 500)
                {
                    lastUiTicks = now;
                    _ = Dispatcher.InvokeAsync(() =>
                    {
                        if (!_isShuttingDown) DiagnosticStatus.Text = _touchDetector.Diagnostics;
                    }, DispatcherPriority.Background);
                }

                foreach (var ev in events)
                {
                    // Only the rising edge (FREE -> ACTIVE) starts the 30 s motor effect.
                    // FREE/release is intentionally ignored by WaveMotion.
                    if (ev.State == ZoneState.Active && _zoneFastRouter is not null)
                        _ = _zoneFastRouter.OnZoneActiveAsync(ev.ZoneId, ct);

                    _ = Dispatcher.InvokeAsync(() =>
                    {
                        if (!_isShuttingDown)
                            SystemStatus.Text = $"ZONE {ev.ZoneId}: {ev.State.ToString().ToUpperInvariant()}";
                    }, DispatcherPriority.Background);
                }

                // Keep the target cadence based on total cycle time instead of adding a full
                // period after detection. This preserves 5-frame touch latency as Zone count grows.
                var cycleElapsedMs = (Stopwatch.GetTimestamp() - cycleStart) * 1000.0 / Stopwatch.Frequency;
                var remainingMs = Math.Max(1, periodMs - (int)Math.Ceiling(cycleElapsedMs));
                await Task.Delay(remainingMs, ct).ConfigureAwait(false);
            }
            catch (OperationCanceledException) { break; }
            catch (Exception ex)
            {
                if (_isShuttingDown) break;
                try
                {
                    await Dispatcher.InvokeAsync(() =>
                    {
                        SystemStatus.Text = "SYSTEM: DETECTION FAULT";
                        DiagnosticStatus.Text = ex.Message;
                    }, DispatcherPriority.Background);
                    await Task.Delay(100, ct).ConfigureAwait(false);
                }
                catch (OperationCanceledException) { break; }
            }
        }
    }

    private void RenderTimer_Tick(object? sender, EventArgs e)
    {
        if (_isShuttingDown) return;
        try
        {
            var lastTicks = Volatile.Read(ref _lastFrameReceivedTicks);
            if (_activeSource is not null && lastTicks != 0)
            {
                var ageMs = (Stopwatch.GetTimestamp() - lastTicks) * 1000.0 / Stopwatch.Frequency;
                if (ageMs > 2000 && !DiagnosticStatus.Text.StartsWith("VIEWER FAULT", StringComparison.Ordinal))
                    DiagnosticStatus.Text = $"NO FRAME > 2 s | Source: {_activeSource.Name} | kiểm tra mạng/cảm biến";
            }

            var sensorFrame = Interlocked.Exchange(ref _latestFrame, null);
            if (sensorFrame is null) return;

            // DISPLAY ONLY: rotate a copy for the viewer. Raw Sensor XYZ remains
            // unchanged for Surface/Zone/touch logic and project persistence.
            var frame = _coordinateEngine.ToWorld(sensorFrame);

            // Keep a short, bounded display history. This is a viewer-only
            // persistence layer: algorithms still see only the newest SENSOR frame.
            _renderHistory.Enqueue(frame);
            while (_renderHistory.Count > RenderPersistenceFrames)
                _renderHistory.Dequeue();

            var sourcePointCount = 0;
            foreach (var f in _renderHistory)
                sourcePointCount += f.Points.Count;

            var stride = Math.Max(1, (int)Math.Ceiling(sourcePointCount / (double)MaxRenderPoints));
            var geometry = new PointGeometry3D
            {
                Positions = new Vector3Collection(),
                Colors = new()
            };

            float minDistance = float.MaxValue, maxDistance = 0f;
            if (_colorMode == "Distance")
            {
                foreach (var f in _renderHistory)
                {
                    for (int i = 0; i < f.Points.Count; i += stride)
                    {
                        var p = f.Points[i];
                        if (!ValidPoint(p)) continue;
                        var d = MathF.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
                        minDistance = MathF.Min(minDistance, d);
                        maxDistance = MathF.Max(maxDistance, d);
                    }
                }
                if (minDistance == float.MaxValue) minDistance = 0;
                if (maxDistance <= minDistance) maxDistance = minDistance + 1;
            }

            foreach (var f in _renderHistory)
            {
                for (int i = 0; i < f.Points.Count; i += stride)
                {
                    var p = f.Points[i];
                    if (!ValidPoint(p)) continue;
                    geometry.Positions.Add(new Vector3(p.X, p.Y, p.Z));
                    geometry.Colors.Add(GetPointColor(p, minDistance, maxDistance));
                    if (geometry.Positions.Count >= MaxRenderPoints) break;
                }
                if (geometry.Positions.Count >= MaxRenderPoints) break;
            }

            // Assign one complete geometry snapshot. Never clear the live model
            // between frames, so the scene stays visible while the next snapshot
            // is being prepared.
            PointCloudModel.Geometry = geometry;
            PointCloudModel.EnableColorBlending = true;
            PointCloudModel.BlendingFactor = 1.0;
            _frameNumber++;
            var renderMs = _renderClock.Elapsed.TotalMilliseconds;
            _renderClock.Restart();

            PointCountText.Text = $"Points: {sensorFrame.Points.Count:N0}";
            FrameText.Text = $"Frame: {_frameNumber:N0}";
            LatencyText.Text = $"Render: {renderMs:0.0} ms";
            var source = _activeSource;
            PacketText.Text = $"Packets: {source?.PacketsReceived ?? 0:N0}";
            DroppedText.Text = $"Dropped: {source?.PacketsDropped ?? 0:N0}";
            UpdateTagDiagnostics(sensorFrame);
            RenderTouchEvidence();
            UpdateStaticMapProgressUi();
            DiagnosticStatus.Text =
                $"Queue: {source?.QueueDepth ?? 0} | Viewer persistence: {_renderHistory.Count} frames | Rendered: {geometry.Positions.Count:N0} | Color: {_colorMode}";
        }
        catch (Exception ex)
        {
            // A render fault must never tear down the acquisition thread or the
            // WPF dispatcher. Keep the last stable scene visible and skip only
            // the faulty frame.
            DiagnosticStatus.Text = $"VIEWER FAULT (frame skipped): {ex.Message}";
        }
    }

    private static bool ValidPoint(Point3D p) =>
        !float.IsNaN(p.X) && !float.IsNaN(p.Y) && !float.IsNaN(p.Z) &&
        !float.IsInfinity(p.X) && !float.IsInfinity(p.Y) && !float.IsInfinity(p.Z);

    private HelixColor4 GetPointColor(Point3D p, float minDistance, float maxDistance)
    {
        if (_colorMode == "Reflectivity")
            return LivoxReflectivityColor(p.Reflectivity);

        if (_colorMode == "Distance")
        {
            var d = MathF.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
            var t = (d - minDistance) / MathF.Max(0.0001f, maxDistance - minDistance);
            return GradientColor(t);
        }

        if (_colorMode == "Tag")
        {
            var tag = LivoxTagDecoder.Decode(p.Tag);
            if (tag.IsNormal) return new HelixColor4(0.25f, 1.00f, 0.35f, 1f);
            if (tag.Atmospheric == LivoxTagConfidence.Low) return new HelixColor4(1.00f, 0.20f, 0.15f, 1f);
            if (tag.Atmospheric == LivoxTagConfidence.Moderate) return new HelixColor4(1.00f, 0.72f, 0.10f, 1f);
            if (tag.Dragging != LivoxTagConfidence.HighConfidenceNormal) return new HelixColor4(1.00f, 0.35f, 0.85f, 1f);
            if (tag.Other != LivoxTagConfidence.HighConfidenceNormal) return new HelixColor4(0.65f, 0.35f, 1.00f, 1f);
            return new HelixColor4(0.65f, 0.65f, 0.65f, 1f);
        }

        return new HelixColor4(0.86f, 0.88f, 0.92f, 1f);
    }

    // Livox MID-series reflectivity semantics: 0..150 is diffuse/Lambertian;
    // 151..255 is reserved for retroreflective targets. Do not normalize 151..255
    // as if it were 101..170 percent reflectivity.
    private static HelixColor4 LivoxReflectivityColor(byte value)
    {
        if (value <= 150)
        {
            var t = value / 150f;
            return GradientColor(t);
        }

        // Keep the retroreflective range visually distinct.
        var r = (value - 151) / 104f;
        return new HelixColor4(1f, 0.15f + 0.65f * (1f - r), 0.05f, 1f);
    }

    private static HelixColor4 GradientColor(float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        // blue -> cyan -> yellow -> red, with no artificial semantic meaning
        // beyond a visual ordering.
        float r = Math.Clamp(2f * t - 0.05f, 0f, 1f);
        float g = Math.Clamp(2f - MathF.Abs(4f * t - 2f), 0f, 1f);
        float b = Math.Clamp(1.1f - 2f * t, 0f, 1f);
        return new HelixColor4(r, g, b, 1f);
    }

    private void ColorModeCombo_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
    {
        if (ColorModeCombo.SelectedItem is not System.Windows.Controls.ComboBoxItem item)
            return;

        _colorMode = item.Tag?.ToString() ?? "Uniform";

        // Resolve the legend at runtime so this handler does not depend on
        // XAML-generated field wiring. This avoids CS0103/CS1061 issues when
        // Visual Studio temporarily has stale generated WPF fields.
        var legend = FindName("ColorLegendText") as System.Windows.Controls.TextBlock;
        if (legend == null)
            return;

        if (_colorMode == "Reflectivity")
            legend.Text = "Livox: 0–150 = diffuse/Lambertian; 151–255 = retroreflective";
        else if (_colorMode == "Distance")
            legend.Text = "Khoảng cách từ gốc World O(0,0,0), đơn vị mét";
        else if (_colorMode == "Tag")
            legend.Text = "Tag Livox: xanh=normal | vàng/đỏ=rain/fog/dust/tiny-particle affected | hồng=dragging | tím=other";
        else
            legend.Text = "Tất cả điểm dùng một màu";
    }

    private void Viewport_MouseDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
    {
        if (_zoneDrawMode != ZoneDrawMode.None && e.ChangedButton == System.Windows.Input.MouseButton.Left)
        {
            if (SurfaceList.SelectedItem is not SurfaceDefinition surface)
            {
                ZoneStatusText.Text = "DRAW ZONE: Surface selection was lost.";
                CancelZoneDraw();
                e.Handled = true;
                return;
            }

            var pos = e.GetPosition(Viewport);
            if (!TryGetSurfaceUvFromViewer(surface, pos, out var uv, out var sensorPoint))
            {
                ZoneStatusText.Text = "DRAW ZONE: con trỏ phải nằm trên Surface đang chọn.";
                e.Handled = true;
                return;
            }
            uv = ApplyZoneSnap(surface, uv, out var snapLabel);
            sensorPoint = SurfaceGeometry.FromUv(surface, uv.U, uv.V);
            ZoneSnapStatusText.Text = snapLabel;

            _zoneDrawUv.Add(uv);
            ZoneStatusText.Text = $"DRAW {_zoneDrawMode.ToString().ToUpperInvariant()}: P{_zoneDrawUv.Count} UV=({uv.U:0.###},{uv.V:0.###}) | SENSOR=({sensorPoint.X:0.###},{sensorPoint.Y:0.###},{sensorPoint.Z:0.###}) m";
            UpdateZoneDrawPreview();

            if (_zoneDrawMode == ZoneDrawMode.Rectangle && _zoneDrawUv.Count >= 2)
                CommitDirectDrawZone();

            e.Handled = true;
            return;
        }

        if (_surfacePickMode && e.ChangedButton == System.Windows.Input.MouseButton.Left)
        {
            var pos = e.GetPosition(Viewport);
            var hit = Viewport.FindNearestPoint(pos);
            if (hit.HasValue)
            {
                var display = new Point3D((float)hit.Value.X, (float)hit.Value.Y, (float)hit.Value.Z, 0, 0);
                var sensor = _coordinateEngine.ToSensor(display);
                _surfacePickPoints.Add(new Vector3(sensor.X, sensor.Y, sensor.Z));
                UpdateSurfacePickGeometry();
                SurfacePickStatus.Text = $"PICK SENSOR: {_surfacePickPoints.Count}/3 | last=({sensor.X:0.###}, {sensor.Y:0.###}, {sensor.Z:0.###}) m";
                if (_surfacePickPoints.Count >= 3)
                    CommitPickedPlane();
                e.Handled = true;
                return;
            }
            SurfacePickStatus.Text = "PICK: không bắt được point cloud. Zoom gần hơn rồi click lại.";
            e.Handled = true;
            return;
        }

        if (_setX0DirectionMode && e.ChangedButton == System.Windows.Input.MouseButton.Left)
        {
            var pos = e.GetPosition(Viewport);
            if (TryGetX0DirectionFromViewer(pos, out var directionProject))
            {
                CommitX0Direction(directionProject);
                e.Handled = true;
                return;
            }
        }

        if (e.ChangedButton == System.Windows.Input.MouseButton.Middle)
        {
            _panActive = true;
            _panLastPoint = e.GetPosition(Viewport);
            Viewport.CaptureMouse();
            e.Handled = true;
        }
    }

    private void Viewport_MouseUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
    {
        if (e.ChangedButton == System.Windows.Input.MouseButton.Middle)
        {
            _panActive = false;
            if (Viewport.IsMouseCaptured)
                Viewport.ReleaseMouseCapture();
            e.Handled = true;
        }
    }

    private void Viewport_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
    {
        var pos = e.GetPosition(Viewport);

        if (_panActive && e.MiddleButton == System.Windows.Input.MouseButtonState.Pressed)
        {
            var dx = pos.X - _panLastPoint.X;
            var dy = pos.Y - _panLastPoint.Y;
            _panLastPoint = pos;

            // Do not depend on HelixToolkit CameraController here. The exact
            // CameraController API differs between HelixToolkit 3.x builds.
            // Pan the camera directly in its local image plane instead. This
            // keeps the feature stable and independent of optional controller APIs.
            PanCamera(dx, dy);

            e.Handled = true;
            return;
        }

        if (_zoneDrawMode != ZoneDrawMode.None)
        {
            if (SurfaceList.SelectedItem is SurfaceDefinition hoverSurface &&
                TryGetSurfaceUvFromViewer(hoverSurface, pos, out var uv, out _))
            {
                uv = ApplyZoneSnap(hoverSurface, uv, out var snapLabel);
                ZoneSnapStatusText.Text = snapLabel;
                _zoneDrawHoverUv = uv;
                UpdateZoneDrawPreview();
            }
            else
            {
                _zoneDrawHoverUv = null;
                UpdateZoneDrawPreview();
            }
            return;
        }

        if (_setX0DirectionMode)
        {
            if (TryGetX0DirectionFromViewer(pos, out var directionProject))
            {
                _x0PreviewDirectionProject = directionProject;
                UpdateWorldCoordinateFramePreview(directionProject);
                var previewAngle = CalculateX0YawForProjectDirection(directionProject);
                XDirectionValueText.Text = $"Calculated yaw preview: {previewAngle:+0.00;-0.00;0.00}°";
                XDirectionHintText.Text = "LMB = confirm X0 direction | MMB = pan | Wheel = zoom | Esc = cancel";
            }
            return;
        }
        // Coordinate hit-testing is intentionally throttled. Point-cloud hit
        // testing gives real XYZ (including Z), but doing it on every raw mouse
        // event can make a dense live viewer feel heavy.
        var nowTicks = Stopwatch.GetTimestamp();
        if (nowTicks - _lastInspectorTicks < Stopwatch.Frequency / 20)
            return;
        _lastInspectorTicks = nowTicks;

        // Manual throttled hit test. Automatic cursor hit-testing is disabled
        // in XAML so camera rotation/pan does not pay the cost on every mouse event.
        // When a point is hit, this returns its real World XYZ from the cloud.
        var hit = Viewport.FindNearestPoint(pos);
        if (hit.HasValue)
        {
            var worldPoint = new Point3D((float)hit.Value.X, (float)hit.Value.Y, (float)hit.Value.Z, 0, 0);
            var sensorPoint = _coordinateEngine.ToSensor(worldPoint);
            _lastPickedWorldPoint = worldPoint;
            _lastPickedSensorPoint = sensorPoint;
            SetCoordinateInspector(sensorPoint, worldPoint, "POINT HIT");
            return;
        }

        // When there is no geometry hit, project the mouse ray onto the active
        // World construction plane. This is intentionally a real 3D plane,
        // not a screen/pixel coordinate. If a surface is selected, use its
        // actual World origin + normal so Z is no longer artificially forced
        // to 0 for a surface at another height/orientation.
        MediaPoint3D planeOrigin = new(0, 0, 0);
        MediaVector3D planeNormal = new(0, 0, 1);
        string planeLabel = "WORLD XY";

        if (SurfaceList.SelectedItem is SurfaceDefinition surface)
        {
            // Surface is persisted in SENSOR coordinates; convert only its display
            // plane to the current viewer alignment for mouse-ray intersection.
            var so = _coordinateEngine.ToWorld(surface.Origin);
            var sn = SensorDirectionToDisplay(new Vector3(surface.Normal.X, surface.Normal.Y, surface.Normal.Z));
            planeOrigin = new MediaPoint3D(so.X, so.Y, so.Z);
            planeNormal = new MediaVector3D(sn.X, sn.Y, sn.Z);
            if (planeNormal.LengthSquared < 1e-9)
                planeNormal = new MediaVector3D(0, 0, 1);
            planeNormal.Normalize();
            planeLabel = surface.Id + " (SENSOR)";
        }

        var world = Viewport.UnProjectOnPlane(pos, planeOrigin, planeNormal);
        if (world.HasValue)
        {
            var wp = new Point3D((float)world.Value.X, (float)world.Value.Y, (float)world.Value.Z, 0, 0);
            var sp = _coordinateEngine.ToSensor(wp);
            SetCoordinateInspector(sp, wp, planeLabel);
        }
    }

    private void SetCoordinateInspector(Point3D sensor, Point3D world, string source)
    {
        var sensorRange = Math.Sqrt(sensor.X * sensor.X + sensor.Y * sensor.Y + sensor.Z * sensor.Z);
        var projectRange = Math.Sqrt(world.X * world.X + world.Y * world.Y + world.Z * world.Z);
        var xy = Math.Sqrt(world.X * world.X + world.Y * world.Y);
        var rangeErrorMm = Math.Abs(sensorRange - projectRange) * 1000.0;
        CoordinateInspectorText.Text =
            $"SENSOR   Xs {sensor.X:+0.000;-0.000;0.000}  Ys {sensor.Y:+0.000;-0.000;0.000}  Zs {sensor.Z:+0.000;-0.000;0.000} m | R {sensorRange:0.000} m\n" +
            $"DISPLAY  X0 {world.X:+0.000;-0.000;0.000}  Y0 {world.Y:+0.000;-0.000;0.000}  Z0 {world.Z:+0.000;-0.000;0.000} m | XY {xy:0.000} | R {projectRange:0.000} | ΔR {rangeErrorMm:0.0} mm | {source}";
    }

    private void PanCamera(double dxPixels, double dyPixels)
    {
        MediaPoint3D position;
        MediaVector3D look;
        MediaVector3D up;
        double scale;

        if (Viewport.Camera is PerspectiveCamera perspective)
        {
            position = perspective.Position;
            look = perspective.LookDirection;
            up = perspective.UpDirection;
            scale = Math.Max(0.0005, look.Length * 0.0015);
        }
        else if (Viewport.Camera is OrthographicCamera orthographic)
        {
            position = orthographic.Position;
            look = orthographic.LookDirection;
            up = orthographic.UpDirection;
            scale = Math.Max(0.0002, orthographic.Width / Math.Max(300.0, Viewport.ActualWidth));
        }
        else
        {
            return;
        }

        if (look.Length < 1e-9) return;
        var forward = look;
        forward.Normalize();
        if (up.Length < 1e-9) up = new MediaVector3D(0, 0, 1);
        up.Normalize();

        var right = Vector3D.CrossProduct(forward, up);
        if (right.Length < 1e-9) return;
        right.Normalize();

        var move = right * (-dxPixels * scale) + up * (dyPixels * scale);
        var moved = new MediaPoint3D(position.X + move.X, position.Y + move.Y, position.Z + move.Z);

        if (Viewport.Camera is PerspectiveCamera p) p.Position = moved;
        else if (Viewport.Camera is OrthographicCamera o) o.Position = moved;
    }

    private void MainWindow_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
    {
        if (e.Key == System.Windows.Input.Key.Escape && _zoneDrawMode != ZoneDrawMode.None)
        {
            CancelZoneDraw();
            e.Handled = true;
            return;
        }

        if (e.Key == System.Windows.Input.Key.Escape && _setX0DirectionMode)
        {
            CancelX0DirectionMode();
            e.Handled = true;
            return;
        }

        if (e.Key == System.Windows.Input.Key.F)
        {
            Viewport.ZoomExtents(1.2);
            e.Handled = true;
        }
        else if (e.Key == System.Windows.Input.Key.R)
        {
            SetPerspectiveCamera(new MediaPoint3D(0, -8, 5),
                new MediaVector3D(0, 8, -5), new MediaVector3D(0, 0, 1));
            e.Handled = true;
        }
    }

    private void SetPerspectiveCamera(MediaPoint3D position, MediaVector3D look, MediaVector3D up)
    {
        Viewport.Camera = new PerspectiveCamera
        {
            Position = position,
            LookDirection = look,
            UpDirection = up,
            FieldOfView = 45,
            NearPlaneDistance = 0.01,
            FarPlaneDistance = 10000
        };
        Viewport.InvalidateRender();
    }

    private void SetOrthographicCamera(MediaPoint3D position, MediaVector3D look,
        MediaVector3D up, double width = 12.0)
    {
        Viewport.Camera = new OrthographicCamera
        {
            Position = position,
            LookDirection = look,
            UpDirection = up,
            Width = width,
            NearPlaneDistance = 0.01,
            FarPlaneDistance = 10000
        };
        Viewport.InvalidateRender();
    }

    private void TopView_Click(object sender, RoutedEventArgs e)
    {
        SetOrthographicCamera(new MediaPoint3D(0, 0, 10),
            new MediaVector3D(0, 0, -10), new MediaVector3D(0, 1, 0));
        Viewport.ZoomExtents(1.15);
    }

    private void FrontView_Click(object sender, RoutedEventArgs e)
    {
        // Front looks along +Y; Z remains vertical on screen.
        SetOrthographicCamera(new MediaPoint3D(0, -10, 0),
            new MediaVector3D(0, 10, 0), new MediaVector3D(0, 0, 1));
        Viewport.ZoomExtents(1.15);
    }

    private void RightView_Click(object sender, RoutedEventArgs e)
    {
        SetOrthographicCamera(new MediaPoint3D(10, 0, 0),
            new MediaVector3D(-10, 0, 0), new MediaVector3D(0, 0, 1));
        Viewport.ZoomExtents(1.15);
    }

    private void LeftView_Click(object sender, RoutedEventArgs e)
    {
        SetOrthographicCamera(new MediaPoint3D(-10, 0, 0),
            new MediaVector3D(10, 0, 0), new MediaVector3D(0, 0, 1));
        Viewport.ZoomExtents(1.15);
    }

    private void HomeView_Click(object sender, RoutedEventArgs e)
    {
        SetPerspectiveCamera(new MediaPoint3D(0, -8, 5),
            new MediaVector3D(0, 8, -5), new MediaVector3D(0, 0, 1));
        Viewport.ZoomExtents(1.2);
    }

    private void ScenarioCombo_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
    {
        if (!_loaded || ScenarioCombo.SelectedItem is not System.Windows.Controls.ComboBoxItem item) return;
        if (Enum.TryParse<SimulationScenario>(item.Tag?.ToString(), out var scenario))
            _simulator.Scenario = scenario;
    }

    private void PointCountCombo_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
    {
        if (!_loaded || PointCountCombo.SelectedItem is not System.Windows.Controls.ComboBoxItem item) return;
        if (int.TryParse(item.Tag?.ToString(), out var count))
            _simulator.PointCount = count;
    }

    private void FpsCombo_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
    {
        if (!_loaded || FpsCombo.SelectedItem is not System.Windows.Controls.ComboBoxItem item) return;
        if (int.TryParse(item.Tag?.ToString(), out var fps))
            _simulator.Fps = fps;
    }

    private void ApplySimulatorSettings()
    {
        if (ScenarioCombo.SelectedItem is System.Windows.Controls.ComboBoxItem s &&
            Enum.TryParse<SimulationScenario>(s.Tag?.ToString(), out var scenario))
            _simulator.Scenario = scenario;

        if (PointCountCombo.SelectedItem is System.Windows.Controls.ComboBoxItem p &&
            int.TryParse(p.Tag?.ToString(), out var points))
            _simulator.PointCount = points;

        if (FpsCombo.SelectedItem is System.Windows.Controls.ComboBoxItem f &&
            int.TryParse(f.Tag?.ToString(), out var fps))
            _simulator.Fps = fps;
    }

    private void UpdateWorldCoordinateFrame()
    {
        var o = Vector3.Zero;
        var x = new Vector3(WorldAxisLengthMeters, 0, 0);
        var y = new Vector3(0, WorldAxisLengthMeters, 0);
        var z = new Vector3(0, 0, WorldAxisLengthMeters);

        WorldXAxisModel.Geometry = new LineGeometry3D
        {
            Positions = new Vector3Collection { o, x },
            Indices = new IntCollection { 0, 1 }
        };
        WorldYAxisModel.Geometry = new LineGeometry3D
        {
            Positions = new Vector3Collection { o, y },
            Indices = new IntCollection { 0, 1 }
        };
        WorldZAxisModel.Geometry = new LineGeometry3D
        {
            Positions = new Vector3Collection { o, z },
            Indices = new IntCollection { 0, 1 }
        };

        WorldXAxisArrowModel.Geometry = CreateArrowHeadGeometry(x, Vector3.UnitX, WorldAxisArrowLengthMeters, WorldAxisArrowRadiusMeters);
        WorldYAxisArrowModel.Geometry = CreateArrowHeadGeometry(y, Vector3.UnitY, WorldAxisArrowLengthMeters, WorldAxisArrowRadiusMeters);
        WorldZAxisArrowModel.Geometry = CreateArrowHeadGeometry(z, Vector3.UnitZ, WorldAxisArrowLengthMeters, WorldAxisArrowRadiusMeters);

        SensorOriginModel.Geometry = new PointGeometry3D
        {
            Positions = new Vector3Collection { o }
        };
        SensorOriginSphereModel.Geometry = CreateSphereGeometry(o, 0.065f, 20, 12);
    }

    private void UpdateWorldCoordinateFramePreview(Vector3 xDirectionProject)
    {
        var zDirectionProject = Vector3.UnitZ;
        var x = xDirectionProject - Vector3.Dot(xDirectionProject, zDirectionProject) * zDirectionProject;
        if (x.LengthSquared() < 1e-6f) x = Vector3.UnitX;
        x = Vector3.Normalize(x);
        var y = Vector3.Normalize(Vector3.Cross(zDirectionProject, x));
        x = Vector3.Normalize(Vector3.Cross(y, zDirectionProject));

        var o = Vector3.Zero;
        var xTip = x * WorldAxisLengthMeters;
        var yTip = y * WorldAxisLengthMeters;
        var zTip = zDirectionProject * WorldAxisLengthMeters;

        WorldXAxisModel.Geometry = new LineGeometry3D
        {
            Positions = new Vector3Collection { o, xTip },
            Indices = new IntCollection { 0, 1 }
        };
        WorldYAxisModel.Geometry = new LineGeometry3D
        {
            Positions = new Vector3Collection { o, yTip },
            Indices = new IntCollection { 0, 1 }
        };
        WorldZAxisModel.Geometry = new LineGeometry3D
        {
            Positions = new Vector3Collection { o, zTip },
            Indices = new IntCollection { 0, 1 }
        };

        WorldXAxisArrowModel.Geometry = CreateArrowHeadGeometry(xTip, x, WorldAxisArrowLengthMeters, WorldAxisArrowRadiusMeters);
        WorldYAxisArrowModel.Geometry = CreateArrowHeadGeometry(yTip, y, WorldAxisArrowLengthMeters, WorldAxisArrowRadiusMeters);
        WorldZAxisArrowModel.Geometry = CreateArrowHeadGeometry(zTip, zDirectionProject, WorldAxisArrowLengthMeters, WorldAxisArrowRadiusMeters);
        Viewport.InvalidateRender();
    }

    private static MeshGeometry3D CreateArrowHeadGeometry(Vector3 tip, Vector3 direction, float length, float radius)
    {
        direction = Vector3.Normalize(direction);
        var baseCenter = tip - direction * length;
        var helper = MathF.Abs(Vector3.Dot(direction, Vector3.UnitZ)) < 0.9f ? Vector3.UnitZ : Vector3.UnitX;
        var u = Vector3.Normalize(Vector3.Cross(direction, helper));
        var v = Vector3.Normalize(Vector3.Cross(direction, u));
        const int segments = 12;
        var positions = new Vector3Collection { tip };
        var normals = new Vector3Collection { direction };
        var indices = new IntCollection();

        for (int i = 0; i < segments; i++)
        {
            float a = 2f * MathF.PI * i / segments;
            var p = baseCenter + (u * MathF.Cos(a) + v * MathF.Sin(a)) * radius;
            positions.Add(p);
            normals.Add(Vector3.Normalize((p - baseCenter) / MathF.Max(radius, 1e-6f) * 0.35f + direction));
        }

        for (int i = 0; i < segments; i++)
        {
            int next = (i + 1) % segments;
            indices.Add(0); indices.Add(1 + i); indices.Add(1 + next);
        }

        return new MeshGeometry3D
        {
            Positions = positions,
            Normals = normals,
            Indices = indices
        };
    }

    private static MeshGeometry3D CreateSphereGeometry(Vector3 center, float radius, int segments, int rings)
    {
        var positions = new Vector3Collection();
        var normals = new Vector3Collection();
        var indices = new IntCollection();

        for (int ring = 0; ring <= rings; ring++)
        {
            float phi = MathF.PI * ring / rings;
            float sinPhi = MathF.Sin(phi);
            float cosPhi = MathF.Cos(phi);

            for (int seg = 0; seg < segments; seg++)
            {
                float theta = 2f * MathF.PI * seg / segments;
                var n = new Vector3(
                    sinPhi * MathF.Cos(theta),
                    sinPhi * MathF.Sin(theta),
                    cosPhi);
                positions.Add(center + n * radius);
                normals.Add(n);
            }
        }

        for (int ring = 0; ring < rings; ring++)
        {
            for (int seg = 0; seg < segments; seg++)
            {
                int next = (seg + 1) % segments;
                int a = ring * segments + seg;
                int b = ring * segments + next;
                int c = (ring + 1) * segments + next;
                int d = (ring + 1) * segments + seg;
                indices.Add(a); indices.Add(b); indices.Add(c);
                indices.Add(a); indices.Add(c); indices.Add(d);
            }
        }

        return new MeshGeometry3D
        {
            Positions = positions,
            Normals = normals,
            Indices = indices
        };
    }

    private void ConfigureStaticGeometryMaterials()
    {
        // Bright emissive origin marker: it must remain visible even when the
        // point cloud or surface lighting changes.
        SensorOriginSphereModel.Material = new PhongMaterial
        {
            DiffuseColor = new HelixColor4(1f, 0.85f, 0f, 1f),
            EmissiveColor = new HelixColor4(1f, 0.65f, 0f, 1f),
            SpecularColor = new HelixColor4(1f, 1f, 1f, 1f)
        };

        WorldXAxisArrowModel.Material = new PhongMaterial
        { DiffuseColor = new HelixColor4(1f, 0.10f, 0.10f, 1f), EmissiveColor = new HelixColor4(0.45f, 0.02f, 0.02f, 1f) };
        WorldYAxisArrowModel.Material = new PhongMaterial
        { DiffuseColor = new HelixColor4(0.10f, 1f, 0.20f, 1f), EmissiveColor = new HelixColor4(0.02f, 0.45f, 0.04f, 1f) };
        WorldZAxisArrowModel.Material = new PhongMaterial
        { DiffuseColor = new HelixColor4(0.20f, 0.45f, 1f, 1f), EmissiveColor = new HelixColor4(0.04f, 0.12f, 0.45f, 1f) };

        // Surface material is intentionally translucent. The outline remains
        // bright so the physical World-coordinate boundary is always readable.
        SurfaceFaceModel.Material = new PhongMaterial
        {
            // Surface is a design reference, not an occluding solid. Keep it
            // light enough that point cloud + Zone remain readable from BOTH sides.
            DiffuseColor = new HelixColor4(0.05f, 0.85f, 1f, 0.08f),
            EmissiveColor = new HelixColor4(0.02f, 0.18f, 0.26f, 0.08f),
            SpecularColor = new HelixColor4(0.10f, 0.10f, 0.10f, 0.08f)
        };
        SurfaceFaceModel.IsTransparent = true;

        ZoneFaceModel.Material = new PhongMaterial
        {
            DiffuseColor = new HelixColor4(1f, 0.62f, 0.02f, 0.34f),
            EmissiveColor = new HelixColor4(0.55f, 0.20f, 0.01f, 0.34f),
            SpecularColor = new HelixColor4(0.18f, 0.18f, 0.18f, 0.34f)
        };
        ZoneFaceModel.IsTransparent = true;
        AllZoneFaceModel.Material = new PhongMaterial
        {
            DiffuseColor = new HelixColor4(1f, 0.55f, 0.01f, 0.11f),
            EmissiveColor = new HelixColor4(0.35f, 0.12f, 0.01f, 0.11f),
            SpecularColor = new HelixColor4(0.08f, 0.08f, 0.08f, 0.11f)
        };
        AllZoneFaceModel.IsTransparent = true;

        ZoneVolumeModel.Material = new PhongMaterial
        {
            DiffuseColor = new HelixColor4(0.02f, 0.78f, 1f, 0.10f),
            EmissiveColor = new HelixColor4(0.01f, 0.24f, 0.32f, 0.10f),
            SpecularColor = new HelixColor4(0.06f, 0.06f, 0.06f, 0.10f)
        };
        ZoneVolumeModel.IsTransparent = true;
    }

    private void UpdateWorldGrid()
    {
        var positions = new Vector3Collection();
        var half = DefaultGridSizeMeters / 2f;
        for (float a = -half; a <= half + 0.001f; a += GridStepMeters)
        {
            positions.Add(new Vector3(a, -half, 0));
            positions.Add(new Vector3(a, half, 0));
            positions.Add(new Vector3(-half, a, 0));
            positions.Add(new Vector3(half, a, 0));
        }
        var indices = new IntCollection();
        for (int i = 0; i + 1 < positions.Count; i += 2)
        {
            indices.Add(i);
            indices.Add(i + 1);
        }
        WorldGridModel.Geometry = new LineGeometry3D
        {
            Positions = positions,
            Indices = indices
        };
    }

    private void SurfaceList_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
    {
        RenderSelectedSurface();
        RefreshZones();
    }

    private void RenderSelectedSurface()
    {
        if (SurfaceList.SelectedItem is not SurfaceDefinition s)
        {
            SurfaceOutlineModel.Geometry = null;
            SurfaceFaceModel.Geometry = null;
            SurfaceUvGridModel.Geometry = null;
            return;
        }
        if (s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.OcctBSpline && !SurfaceGeometry.IsOcctAvailable(out var occtMissing))
        {
            SurfaceOutlineModel.Geometry = null; SurfaceFaceModel.Geometry = null; SurfaceUvGridModel.Geometry = null;
            OcctStatusText.Text = $"OCCT: NOT READY | {occtMissing}";
            SurfacePickStatus.Text = $"{s.Id}: OCCT Surface đã lưu nhưng native bridge chưa sẵn sàng.";
        }
        else if (s.Type == SurfaceType.Curved)
            DrawCurvedSurface(s);
        else
        {
            var o = SensorPointToDisplay(s.Origin);
            var u = SensorDirectionToDisplay(new Vector3(s.UAxis.X, s.UAxis.Y, s.UAxis.Z));
            var v = SensorDirectionToDisplay(new Vector3(s.VAxis.X, s.VAxis.Y, s.VAxis.Z));
            DrawSurfaceOutline(o, u, v, 0, (float)Math.Max(0.01, s.WidthMeters), 0, (float)Math.Max(0.01, s.HeightMeters));
            SurfaceUvGridModel.Geometry = null;
        }
        SurfaceXText.Text = s.Origin.X.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        SurfaceYText.Text = s.Origin.Y.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        SurfaceZText.Text = s.Origin.Z.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        SurfaceWidthText.Text = s.WidthMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        SurfaceHeightText.Text = s.HeightMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        SurfaceRadiusText.Text = s.CurvatureRadiusMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        OcctBulgeText.Text = s.OcctBulgeMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        OcctTwistText.Text = s.OcctTwistMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        if (s.CurvedKind == CurvedSurfaceKind.OcctBSpline && s.OcctFitControlPoints.Count > 0)
            OcctFitStatusText.Text = $"FIT: {s.OcctFitSourcePointCount:N0} map pts | {s.OcctFitUCount}×{s.OcctFitVCount} | RMSE {s.OcctFitRmseMm:0.0} mm";
        else OcctFitStatusText.Text = "FIT: manual seed (Bulge/Twist)";
        SurfaceFrameText.Text = $"FRAME: {s.Frame} 🔒 | TYPE: {s.Type}";
        SurfaceAxesText.Text =
            $"U [{s.UAxis.X:+0.000;-0.000;+0.000},{s.UAxis.Y:+0.000;-0.000;+0.000},{s.UAxis.Z:+0.000;-0.000;+0.000}]  " +
            $"V [{s.VAxis.X:+0.000;-0.000;+0.000},{s.VAxis.Y:+0.000;-0.000;+0.000},{s.VAxis.Z:+0.000;-0.000;+0.000}]\n" +
            $"N [{s.Normal.X:+0.000;-0.000;+0.000},{s.Normal.Y:+0.000;-0.000;+0.000},{s.Normal.Z:+0.000;-0.000;+0.000}]" +
            (s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.Cylinder
                ? $"\nCURVED Cylinder R={s.CurvatureRadiusMeters:0.###} m | arc U={s.WidthMeters:0.###} m | sign={(s.CurvatureSign < 0 ? "-" : "+")}"
                : s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.OcctBSpline
                    ? $"\nOCCT B-Spline | bulge={s.OcctBulgeMeters:+0.###;-0.###;0} m | twist={s.OcctTwistMeters:+0.###;-0.###;0} m" : "");
        SelectedSurfaceText.Text = $"SELECTED: {s.Id} | {s.Name} | SENSOR";
        CurvedInfoText.Text = s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.Cylinder ? $"Arc angle: {s.WidthMeters / Math.Max(1e-6, s.CurvatureRadiusMeters) * 180.0 / Math.PI:0.0}° | bend {(s.CurvatureSign < 0 ? "−N" : "+N")}" : s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.OcctBSpline ? $"OCCT B-Spline | bulge {s.OcctBulgeMeters:+0.###;-0.###;0} m | twist {s.OcctTwistMeters:+0.###;-0.###;0} m" : "Curved: --";
        RenderAllZones();
        RenderSelectedZone();
    }

    private async void FlipCurvedSurface_Click(object sender, RoutedEventArgs e)
    {
        if (SurfaceList.SelectedItem is not SurfaceDefinition s || s.Type != SurfaceType.Curved || s.CurvedKind != CurvedSurfaceKind.Cylinder) { SurfacePickStatus.Text = "FLIP CURVE chỉ dùng cho Cylinder."; return; }
        s.CurvatureSign = s.CurvatureSign < 0 ? 1.0 : -1.0;
        RenderSelectedSurface();
        SurfacePickStatus.Text = $"{s.Id}: curvature flipped {(s.CurvatureSign < 0 ? "−N" : "+N")}.";
        await _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void CurvedRadiusMinus_Click(object sender, RoutedEventArgs e) => AdjustCurvedRadius(-0.05);
    private void CurvedRadiusPlus_Click(object sender, RoutedEventArgs e) => AdjustCurvedRadius(+0.05);
    private async void AdjustCurvedRadius(double delta)
    {
        if (SurfaceList.SelectedItem is not SurfaceDefinition s || s.Type != SurfaceType.Curved || s.CurvedKind != CurvedSurfaceKind.Cylinder) { SurfacePickStatus.Text = "RADIUS chỉ dùng cho Cylinder."; return; }
        var minR = Math.Max(0.02, s.WidthMeters / (2.0*Math.PI));
        s.CurvatureRadiusMeters = Math.Max(minR, s.CurvatureRadiusMeters + delta);
        SurfaceRadiusText.Text = s.CurvatureRadiusMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        OcctBulgeText.Text = s.OcctBulgeMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        OcctTwistText.Text = s.OcctTwistMeters.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        RenderSelectedSurface();
        await _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void ClearSurfaceButton_Click(object sender, RoutedEventArgs e)
    {
        SurfaceOutlineModel.Geometry = null;
        SurfaceFaceModel.Geometry = null;
        SurfacePickPointsModel.Geometry = null;
        SurfacePickStatus.Text = "Surface: đã ẩn khỏi Viewer (geometry vẫn được lưu).";
    }

    private void ShowSurfaceButton_Click(object sender, RoutedEventArgs e)
    {
        if (SurfaceList.SelectedItem is not SurfaceDefinition s)
        {
            SurfacePickStatus.Text = "HIỆN: chọn một Surface trước.";
            return;
        }
        RenderSelectedSurface();
        SurfacePickStatus.Text = $"{s.Id}: đang hiển thị trong Viewer.";
    }

    private async void DeleteSurfaceButton_Click(object sender, RoutedEventArgs e)
    {
        if (SurfaceList.SelectedItem is not SurfaceDefinition surface)
        {
            SurfacePickStatus.Text = "XÓA: chọn một Surface trước.";
            return;
        }

        var linkedZones = _project.Zones.Where(z => z.SurfaceId == surface.Id).ToList();
        var message = linkedZones.Count == 0
            ? $"Xóa {surface.Id} - {surface.Name}?"
            : $"Xóa {surface.Id} - {surface.Name}?\n\n{linkedZones.Count} Zone gắn với Surface này cũng sẽ bị xóa.";
        if (WpfMessageBox.Show(message, "Delete Surface", MessageBoxButton.YesNo, MessageBoxImage.Warning) != MessageBoxResult.Yes)
            return;

        foreach (var z in linkedZones) _project.Zones.Remove(z);
        _project.Surfaces.Remove(surface);
        SurfaceList.ItemsSource = null;
        SurfaceList.ItemsSource = _project.Surfaces;
        SurfaceOutlineModel.Geometry = null;
        SurfaceFaceModel.Geometry = null;
        ClearZoneRender();
        SelectedSurfaceText.Text = "SELECTED: --";
        RefreshZones();
        SurfacePickStatus.Text = $"{surface.Id}: đã xóa cùng {linkedZones.Count} Zone liên kết.";
        await _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void NewSensorPlaneButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            SurfacePickStatus.Text = "NEW PLANE: hãy BUILD/LOAD STATIC MAP rồi vào SURFACE DESIGN.";
            return;
        }
        // NEW means new geometry record. It does not depend on a point-cloud pick,
        // camera ray, IMU orientation or display coordinates.
        SurfaceList.SelectedItem = null;
        CreatePlaneButton_Click(sender, e);
        if (SurfaceList.SelectedItem is SurfaceDefinition created)
        {
            SurfacePickStatus.Text = $"{created.Id}: NEW plane created directly in MID360_SENSOR. Use MOVE / ROTATE / SIZE to place it.";
        }
    }

    private void NewSensorCylinderButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            SurfacePickStatus.Text = "NEW CURVED: hãy BUILD/LOAD STATIC MAP rồi vào SURFACE DESIGN.";
            return;
        }
        if (!TryReadDouble(SurfaceXText.Text, out var x) || !TryReadDouble(SurfaceYText.Text, out var y) ||
            !TryReadDouble(SurfaceZText.Text, out var z) || !TryReadDouble(SurfaceWidthText.Text, out var width) ||
            !TryReadDouble(SurfaceHeightText.Text, out var height) || !TryReadDouble(SurfaceRadiusText.Text, out var radius) ||
            width <= 0 || height <= 0 || radius <= 0)
        {
            WpfMessageBox.Show("Curved Surface: X/Y/Z, arc width, height và radius phải là số > 0.", "Surface Editor", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        if (width > 2.0 * Math.PI * radius + 1e-9)
        {
            WpfMessageBox.Show("Curved Surface: arc Width không được lớn hơn chu vi 2πR.", "Surface Editor", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        // Cylinder patch basis at U=0: N is outward radial, U is tangent, V is cylinder axis.
        var orientation = (SurfaceOrientationCombo.SelectedItem as System.Windows.Controls.ComboBoxItem)?.Tag?.ToString() ?? "XY";
        Vector3 n, u, v;
        switch (orientation)
        {
            case "XZ": n = -Vector3.UnitY; u = Vector3.UnitX; v = Vector3.UnitZ; break;
            case "YZ": n = Vector3.UnitX; u = Vector3.UnitY; v = Vector3.UnitZ; break;
            default: n = Vector3.UnitZ; u = Vector3.UnitX; v = Vector3.UnitY; break;
        }
        var next = 1;
        while (_project.Surfaces.Any(q => q.Id.Equals($"S{next:00}", StringComparison.OrdinalIgnoreCase))) next++;
        var surface = new SurfaceDefinition
        {
            Id = $"S{next:00}", Name = $"Curved {next:00}", Frame = "MID360_SENSOR", Type = SurfaceType.Curved,
            CurvedKind = CurvedSurfaceKind.Cylinder, CurvatureRadiusMeters = radius,
            Origin = new Point3D((float)x,(float)y,(float)z,0,0), WidthMeters = width, HeightMeters = height,
            UAxis = new Point3D(u.X,u.Y,u.Z,0,0), VAxis = new Point3D(v.X,v.Y,v.Z,0,0), Normal = new Point3D(n.X,n.Y,n.Z,0,0)
        };
        _project.Surfaces.Add(surface);
        SurfaceList.ItemsSource = null; SurfaceList.ItemsSource = _project.Surfaces; SurfaceList.SelectedItem = surface;
        RenderSelectedSurface(); RefreshZones();
        SurfacePickStatus.Text = $"{surface.Id}: CYLINDER in MID360_SENSOR | R={radius:0.###} m | arc U={width:0.###} m | V={height:0.###} m.";
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void NewOcctBsplineButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null) { SurfacePickStatus.Text = "NEW OCCT: hãy BUILD/LOAD STATIC MAP rồi vào SURFACE DESIGN."; return; }
        if (!SurfaceGeometry.IsOcctAvailable(out var status))
        {
            OcctStatusText.Text = $"OCCT: NOT READY | {status}";
            WpfMessageBox.Show("OCCT native bridge chưa sẵn sàng. Build scripts\\build_occt.ps1 sau khi cài Open CASCADE Technology (OCCT). Plane/Cylinder vẫn hoạt động bình thường.", "OCCT", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        if (!TryReadDouble(SurfaceXText.Text,out var x) || !TryReadDouble(SurfaceYText.Text,out var y) || !TryReadDouble(SurfaceZText.Text,out var z) ||
            !TryReadDouble(SurfaceWidthText.Text,out var width) || !TryReadDouble(SurfaceHeightText.Text,out var height) || width<=0 || height<=0 ||
            !TryReadDouble(OcctBulgeText.Text,out var bulge) || !TryReadDouble(OcctTwistText.Text,out var twist))
        { WpfMessageBox.Show("OCCT Surface: nhập X/Y/Z, Width/Height > 0, Bulge/Twist hợp lệ.","OCCT Surface",MessageBoxButton.OK,MessageBoxImage.Warning); return; }
        var orientation=(SurfaceOrientationCombo.SelectedItem as System.Windows.Controls.ComboBoxItem)?.Tag?.ToString() ?? "XY";
        Vector3 n,u,v;
        switch(orientation){ case "XZ": n=-Vector3.UnitY;u=Vector3.UnitX;v=Vector3.UnitZ;break; case "YZ": n=Vector3.UnitX;u=Vector3.UnitY;v=Vector3.UnitZ;break; default:n=Vector3.UnitZ;u=Vector3.UnitX;v=Vector3.UnitY;break;}
        var next=1; while(_project.Surfaces.Any(q=>q.Id.Equals($"S{next:00}",StringComparison.OrdinalIgnoreCase))) next++;
        var surface=new SurfaceDefinition{Id=$"S{next:00}",Name=$"OCCT B-Spline {next:00}",Frame="MID360_SENSOR",Type=SurfaceType.Curved,CurvedKind=CurvedSurfaceKind.OcctBSpline,Origin=new Point3D((float)x,(float)y,(float)z,0,0),WidthMeters=width,HeightMeters=height,UAxis=new Point3D(u.X,u.Y,u.Z,0,0),VAxis=new Point3D(v.X,v.Y,v.Z,0,0),Normal=new Point3D(n.X,n.Y,n.Z,0,0),OcctBulgeMeters=bulge,OcctTwistMeters=twist};
        try { _ = SurfaceGeometry.FromUv(surface,width*0.5,height*0.5); }
        catch(Exception ex){ WpfMessageBox.Show($"OCCT tạo surface thất bại:\n{ex.Message}","OCCT Surface",MessageBoxButton.OK,MessageBoxImage.Error); return; }
        _project.Surfaces.Add(surface); SurfaceList.ItemsSource=null; SurfaceList.ItemsSource=_project.Surfaces; SurfaceList.SelectedItem=surface; RenderSelectedSurface(); RefreshZones();
        OcctStatusText.Text=$"OCCT: READY | {status}"; SurfacePickStatus.Text=$"{surface.Id}: OCCT B-Spline trong MID360_SENSOR | {width:0.###}×{height:0.###} m | bulge={bulge:+0.###;-0.###;0} m.";
        _ = _projectStore.SaveAsync(_project,Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void SurfaceTransformButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            SurfacePickStatus.Text = "Surface transform chỉ hoạt động trong SURFACE DESIGN trên Static Map.";
            return;
        }
        if (SurfaceList.SelectedItem is not SurfaceDefinition surface)
        {
            SurfacePickStatus.Text = "Surface transform: chọn một Surface trước.";
            return;
        }
        if (!string.Equals(surface.Frame, "MID360_SENSOR", StringComparison.Ordinal))
        {
            WpfMessageBox.Show($"Surface {surface.Id} không thuộc MID360_SENSOR. Không cho phép transform để tránh trộn coordinate frame.",
                "Surface frame guard", MessageBoxButton.OK, MessageBoxImage.Error);
            return;
        }
        if (sender is not System.Windows.Controls.Button button || button.Tag is not string op)
            return;

        if (op.StartsWith("M", StringComparison.Ordinal))
        {
            var step = GetComboStep(SurfaceMoveStepCombo, 0.01);
            var sign = op.EndsWith("+", StringComparison.Ordinal) ? 1.0 : -1.0;
            var dx = 0.0; var dy = 0.0; var dz = 0.0;
            switch (op[1])
            {
                case 'X': dx = sign * step; break;
                case 'Y': dy = sign * step; break;
                case 'Z': dz = sign * step; break;
                default: return;
            }
            surface.Origin = new Point3D(
                surface.Origin.X + (float)dx,
                surface.Origin.Y + (float)dy,
                surface.Origin.Z + (float)dz, 0, 0);
            if (surface.OcctFitControlPoints.Count > 0)
            {
                for (var i=0;i<surface.OcctFitControlPoints.Count;i++)
                {
                    var fp=surface.OcctFitControlPoints[i];
                    surface.OcctFitControlPoints[i]=new Point3D(fp.X+(float)dx,fp.Y+(float)dy,fp.Z+(float)dz,0,0);
                }
            }
            SurfacePickStatus.Text = $"{surface.Id}: moved in SENSOR {op[1]} by {sign * step:+0.###;-0.###} m.";
        }
        else if (op.StartsWith("R", StringComparison.Ordinal))
        {
            var stepDeg = GetComboStep(SurfaceRotateStepCombo, 1.0);
            var sign = op.EndsWith("+", StringComparison.Ordinal) ? 1.0 : -1.0;
            var axis = op[1] switch
            {
                'X' => Vector3.UnitX,
                'Y' => Vector3.UnitY,
                'Z' => Vector3.UnitZ,
                _ => Vector3.Zero
            };
            if (axis == Vector3.Zero) return;
            RotateSurfaceInSensorFrame(surface, axis, sign * stepDeg);
            SurfacePickStatus.Text = $"{surface.Id}: rotated around SENSOR {op[1]} by {sign * stepDeg:+0.###;-0.###}°.";
        }
        else return;

        RenderSelectedSurface();
        RefreshZones();
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
        DiagnosticStatus.Text = $"{surface.Id} transformed in MID360_SENSOR only; display/camera transform unchanged.";
    }

    private static double GetComboStep(System.Windows.Controls.ComboBox combo, double fallback)
    {
        if (combo.SelectedItem is System.Windows.Controls.ComboBoxItem item &&
            double.TryParse(item.Tag?.ToString(), System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var step) && step > 0)
            return step;
        return fallback;
    }

    private static void RotateSurfaceInSensorFrame(SurfaceDefinition surface, Vector3 sensorAxis, double angleDeg)
    {
        var radians = (float)(Math.PI / 180.0 * angleDeg);
        var axis = Vector3.Normalize(sensorAxis);
        if (surface.OcctFitControlPoints.Count > 0)
        {
            var o=new Vector3(surface.Origin.X,surface.Origin.Y,surface.Origin.Z);
            for(var i=0;i<surface.OcctFitControlPoints.Count;i++)
            {
                var fp=surface.OcctFitControlPoints[i]; var rel=new Vector3(fp.X,fp.Y,fp.Z)-o;
                var rr=RotateVectorRodrigues(rel,axis,radians)+o;
                surface.OcctFitControlPoints[i]=new Point3D(rr.X,rr.Y,rr.Z,0,0);
            }
        }
        var u = RotateVectorRodrigues(new Vector3(surface.UAxis.X, surface.UAxis.Y, surface.UAxis.Z), axis, radians);
        var v = RotateVectorRodrigues(new Vector3(surface.VAxis.X, surface.VAxis.Y, surface.VAxis.Z), axis, radians);

        // Re-orthogonalize after each edit so accumulated floating-point error never
        // turns the stored Surface into a skew coordinate basis.
        u = Vector3.Normalize(u);
        v -= Vector3.Dot(v, u) * u;
        if (v.LengthSquared() < 1e-10f)
            throw new InvalidOperationException("Surface rotation produced a degenerate V axis.");
        v = Vector3.Normalize(v);
        var n = Vector3.Normalize(Vector3.Cross(u, v));
        v = Vector3.Normalize(Vector3.Cross(n, u));

        surface.UAxis = new Point3D(u.X, u.Y, u.Z, 0, 0);
        surface.VAxis = new Point3D(v.X, v.Y, v.Z, 0, 0);
        surface.Normal = new Point3D(n.X, n.Y, n.Z, 0, 0);
        surface.Frame = "MID360_SENSOR";
    }

    private static Vector3 RotateVectorRodrigues(Vector3 value, Vector3 axis, float angle)
    {
        var c = MathF.Cos(angle);
        var s = MathF.Sin(angle);
        return value * c + Vector3.Cross(axis, value) * s + axis * Vector3.Dot(axis, value) * (1f - c);
    }

    private void CreatePlaneButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            SurfacePickStatus.Text = "APPLY: hãy BUILD/LOAD STATIC MAP rồi vào SURFACE DESIGN.";
            return;
        }
        if (!TryReadDouble(SurfaceXText.Text, out var x) ||
            !TryReadDouble(SurfaceYText.Text, out var y) ||
            !TryReadDouble(SurfaceZText.Text, out var z) ||
            !TryReadDouble(SurfaceWidthText.Text, out var width) ||
            !TryReadDouble(SurfaceHeightText.Text, out var height) ||
            !TryReadDouble(SurfaceRadiusText.Text, out var radius) ||
            width <= 0 || height <= 0 || radius <= 0)
        {
            WpfMessageBox.Show("Hãy nhập X/Y/Z và kích thước mặt bằng số thực hợp lệ. Kích thước phải > 0.",
                "Surface Editor", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        if (!TryReadDouble(OcctBulgeText.Text, out var occtBulge)) occtBulge = 0.20;
        if (!TryReadDouble(OcctTwistText.Text, out var occtTwist)) occtTwist = 0.0;

        var surface = SurfaceList.SelectedItem as SurfaceDefinition;
        if (surface is not null && surface.Type == SurfaceType.Curved && surface.CurvedKind == CurvedSurfaceKind.Cylinder && width > 2.0 * Math.PI * radius + 1e-9)
        {
            WpfMessageBox.Show("Curved Surface: arc Width không được lớn hơn chu vi 2πR.", "Surface Editor", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        var orientation = (SurfaceOrientationCombo.SelectedItem as System.Windows.Controls.ComboBoxItem)?.Tag?.ToString() ?? "XY";
        var isNew = surface is null;
        Vector3 normal = Vector3.UnitZ, u = Vector3.UnitX, v = Vector3.UnitY;

        if (isNew)
        {
            // Orientation presets are used ONLY at creation. After a Surface has been
            // rotated in Sensor Frame, APPLY VALUES must never silently reset U/V/N.
            switch (orientation)
            {
                case "XZ": normal = new Vector3(0, -1, 0); u = new Vector3(1, 0, 0); v = new Vector3(0, 0, 1); break;
                case "YZ": normal = new Vector3(1, 0, 0); u = new Vector3(0, 1, 0); v = new Vector3(0, 0, 1); break;
                default: normal = new Vector3(0, 0, 1); u = new Vector3(1, 0, 0); v = new Vector3(0, 1, 0); break;
            }
            var next = 1;
            while (_project.Surfaces.Any(s => s.Id.Equals($"S{next:00}", StringComparison.OrdinalIgnoreCase))) next++;
            surface = new SurfaceDefinition { Id = $"S{next:00}", Name = $"Surface {next:00}" };
            _project.Surfaces.Add(surface);
            surface.Normal = new Point3D(normal.X, normal.Y, normal.Z, 0, 0);
            surface.UAxis = new Point3D(u.X, u.Y, u.Z, 0, 0);
            surface.VAxis = new Point3D(v.X, v.Y, v.Z, 0, 0);
        }

        surface!.Frame = "MID360_SENSOR";
        if (isNew) surface.Type = SurfaceType.Plane;
        surface.Origin = new Point3D((float)x, (float)y, (float)z, 0, 0);
        surface.WidthMeters = width;
        surface.HeightMeters = height;
        surface.CurvatureRadiusMeters = radius;
        if (surface.CurvedKind == CurvedSurfaceKind.OcctBSpline)
        {
            surface.OcctBulgeMeters = occtBulge; surface.OcctTwistMeters = occtTwist;
            if (surface.OcctFitControlPoints.Count > 0)
            {
                surface.OcctFitControlPoints.Clear(); surface.OcctFitUCount=0; surface.OcctFitVCount=0;
                surface.OcctFitSourcePointCount=0; surface.OcctFitRmseMm=0;
                OcctFitStatusText.Text = "FIT: cleared by manual APPLY; bấm FIT lại sau khi chỉnh coarse frame.";
            }
        }
        surface.ValidationRmseMm = 0;

        SurfaceList.ItemsSource = null;
        SurfaceList.ItemsSource = _project.Surfaces;
        SurfaceList.SelectedItem = surface;
        RenderSelectedSurface();
        RefreshZones();
        SurfacePickStatus.Text = isNew
            ? $"{surface.Id}: created in SENSOR {orientation}, O=({x:0.###}, {y:0.###}, {z:0.###}) m, {width:0.###} × {height:0.###} m."
            : $"{surface.Id}: SENSOR values applied; U/V/N orientation preserved.";
        DiagnosticStatus.Text = $"Surface {surface.Id} ({surface.Type}) lưu theo MID-360 SENSOR XYZ; Viewer alignment không làm đổi geometry.";
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private static bool TryReadDouble(string text, out double value) =>
        double.TryParse(text, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out value) ||
        double.TryParse(text, out value);

    private void DrawSurfaceOutline(Vector3 origin, Vector3 u, Vector3 v,
                                    float minU, float maxU, float minV, float maxV)
    {
        var p0 = origin + u * minU + v * minV;
        var p1 = origin + u * maxU + v * minV;
        var p2 = origin + u * maxU + v * maxV;
        var p3 = origin + u * minU + v * maxV;
        SurfaceOutlineModel.Geometry = new LineGeometry3D
        {
            Positions = new Vector3Collection { p0, p1, p2, p3 },
            Indices = new IntCollection { 0, 1, 1, 2, 2, 3, 3, 0 }
        };

        // Filled plane: same real World XYZ vertices as the outline.
        // Two triangles are used so the surface is visible from both sides.
        // Explicit normals are supplied so the Phong material is lit correctly
        // regardless of the active camera direction. Both triangles use the
        // same World-space normal, and culling is disabled in XAML.
        var n = Vector3.Normalize(Vector3.Cross(u, v));
        SurfaceFaceModel.Geometry = new MeshGeometry3D
        {
            Positions = new Vector3Collection { p0, p1, p2, p3 },
            Normals = new Vector3Collection { n, n, n, n },
            Indices = new IntCollection { 0, 1, 2, 0, 2, 3 }
        };
    }

    private void DrawCurvedSurface(SurfaceDefinition surface)
    {
        // Curved patch in MID360_SENSOR. SurfaceGeometry dispatches to analytic Cylinder or real OCCT B-Spline.
        // Tessellation below is display-only; persisted geometry remains Sensor-frame parameters.
        var uSegments = Math.Clamp((int)Math.Ceiling(surface.WidthMeters / 0.03), 12, 128);
        var vSegments = Math.Clamp((int)Math.Ceiling(surface.HeightMeters / 0.08), 2, 48);
        var positions = new Vector3Collection();
        var normals = new Vector3Collection();
        for (int j = 0; j <= vSegments; j++)
        {
            var v = surface.HeightMeters * j / vSegments;
            for (int i = 0; i <= uSegments; i++)
            {
                var u = surface.WidthMeters * i / uSegments;
                var p = SurfaceGeometry.FromUv(surface, u, v);
                var n = SurfaceGeometry.NormalAtUv(surface, u, v);
                positions.Add(SensorPointToDisplay(p));
                normals.Add(SensorDirectionToDisplay(Vector3.Normalize(new Vector3(n.X, n.Y, n.Z))));
            }
        }
        var indices = new IntCollection();
        var stride = uSegments + 1;
        for (int j = 0; j < vSegments; j++)
        for (int i = 0; i < uSegments; i++)
        {
            var a = j * stride + i; var b = a + 1; var c = a + stride; var d = c + 1;
            indices.Add(a); indices.Add(b); indices.Add(d);
            indices.Add(a); indices.Add(d); indices.Add(c);
        }
        SurfaceFaceModel.Geometry = new MeshGeometry3D { Positions = positions, Normals = normals, Indices = indices };

        // Four sampled boundaries; no thick chord is drawn through the curvature.
        var outline = new Vector3Collection();
        var lineIdx = new IntCollection();
        void AddEdge(Func<double, UvPoint> uvAt, int segments)
        {
            var baseIndex = outline.Count;
            for (int k = 0; k <= segments; k++)
            {
                var uv = uvAt((double)k / segments);
                outline.Add(SensorPointToDisplay(SurfaceGeometry.FromUv(surface, uv.U, uv.V)));
                if (k > 0) { lineIdx.Add(baseIndex + k - 1); lineIdx.Add(baseIndex + k); }
            }
        }
        AddEdge(t => new UvPoint(surface.WidthMeters * t, 0), uSegments);
        AddEdge(t => new UvPoint(surface.WidthMeters * t, surface.HeightMeters), uSegments);
        AddEdge(t => new UvPoint(0, surface.HeightMeters * t), vSegments);
        AddEdge(t => new UvPoint(surface.WidthMeters, surface.HeightMeters * t), vSegments);
        SurfaceOutlineModel.Geometry = new LineGeometry3D { Positions = outline, Indices = lineIdx };

        // Display-only UV guide grid. It makes curved Zone placement readable without changing SENSOR geometry.
        var gpos = new Vector3Collection(); var gidx = new IntCollection();
        void AddGridCurve(Func<double, UvPoint> uvAt, int segments)
        {
            var b0 = gpos.Count;
            for (int k=0;k<=segments;k++) { var uv=uvAt((double)k/segments); gpos.Add(SensorPointToDisplay(SurfaceGeometry.FromUv(surface,uv.U,uv.V))); if(k>0){gidx.Add(b0+k-1);gidx.Add(b0+k);} }
        }
        for(int q=1;q<6;q++){ var u=surface.WidthMeters*q/6.0; AddGridCurve(t=>new UvPoint(u,surface.HeightMeters*t),vSegments); }
        for(int q=1;q<4;q++){ var v=surface.HeightMeters*q/4.0; AddGridCurve(t=>new UvPoint(surface.WidthMeters*t,v),uSegments); }
        SurfaceUvGridModel.Geometry = new LineGeometry3D{Positions=gpos,Indices=gidx};
    }

    private Vector3Collection BuildZonePolylineDisplay(SurfaceDefinition surface, IReadOnlyList<UvPoint> uvPoints, bool closed, float lift, double targetSegmentMeters = 0.06, int maxSegments = 32)
    {
        var display = new Vector3Collection();
        if (uvPoints.Count == 0) return display;
        var edgeCount = closed ? uvPoints.Count : Math.Max(0, uvPoints.Count - 1);
        for (int e = 0; e < edgeCount; e++)
        {
            var a = uvPoints[e]; var b = uvPoints[(e + 1) % uvPoints.Count];
            var lengthUv = Math.Sqrt((b.U-a.U)*(b.U-a.U) + (b.V-a.V)*(b.V-a.V));
            // Overview rendering is deliberately coarse. Precise Zone detection still uses
            // the mathematical SurfaceGeometry, not this display polyline.
            var seg = surface.Type == SurfaceType.Curved ? Math.Clamp((int)Math.Ceiling(lengthUv / Math.Max(0.02, targetSegmentMeters)), 1, Math.Max(4, maxSegments)) : 1;
            for (int k = 0; k < seg; k++)
            {
                var t = (double)k / seg;
                var uv = new UvPoint(a.U + (b.U-a.U)*t, a.V + (b.V-a.V)*t);
                var p0 = SurfaceGeometry.FromUv(surface, uv.U, uv.V);
                var np = SurfaceGeometry.NormalAtUv(surface, uv.U, uv.V);
                var sn = Vector3.Normalize(new Vector3(np.X,np.Y,np.Z));
                var sp = new Vector3(p0.X,p0.Y,p0.Z) + sn*lift;
                display.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0)));
            }
        }
        if (!closed && uvPoints.Count > 0)
        {
            var uv = uvPoints[^1]; var p0 = SurfaceGeometry.FromUv(surface,uv.U,uv.V);
            var np=SurfaceGeometry.NormalAtUv(surface,uv.U,uv.V); var sn=Vector3.Normalize(new Vector3(np.X,np.Y,np.Z));
            var sp=new Vector3(p0.X,p0.Y,p0.Z)+sn*lift;
            display.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0)));
        }
        return display;
    }

    private static List<int> TriangulateZoneUv(IReadOnlyList<UvPoint> poly)
    {
        var result = new List<int>();
        if (poly.Count < 3) return result;
        double area = 0;
        for (int i=0;i<poly.Count;i++){var a=poly[i];var b=poly[(i+1)%poly.Count];area += a.U*b.V-b.U*a.V;}
        var ccw = area >= 0;
        var verts = Enumerable.Range(0, poly.Count).ToList();
        bool IsConvex(UvPoint a,UvPoint b,UvPoint c)
        {
            var cross=(b.U-a.U)*(c.V-b.V)-(b.V-a.V)*(c.U-b.U);
            return ccw ? cross > 1e-12 : cross < -1e-12;
        }
        static bool InTri(UvPoint p,UvPoint a,UvPoint b,UvPoint c)
        {
            static double C(UvPoint p1,UvPoint p2,UvPoint p3)=>(p2.U-p1.U)*(p3.V-p1.V)-(p2.V-p1.V)*(p3.U-p1.U);
            var d1=C(p,a,b);var d2=C(p,b,c);var d3=C(p,c,a);
            var neg=d1 < -1e-10 || d2 < -1e-10 || d3 < -1e-10; var pos=d1 > 1e-10 || d2 > 1e-10 || d3 > 1e-10;
            return !(neg && pos);
        }
        var guard=0;
        while(verts.Count>3 && guard++<poly.Count*poly.Count)
        {
            var clipped=false;
            for(int k=0;k<verts.Count;k++)
            {
                var ia=verts[(k-1+verts.Count)%verts.Count];var ib=verts[k];var ic=verts[(k+1)%verts.Count];
                var a=poly[ia];var b=poly[ib];var c=poly[ic]; if(!IsConvex(a,b,c)) continue;
                var contains=false;
                foreach(var j in verts){if(j==ia||j==ib||j==ic)continue;if(InTri(poly[j],a,b,c)){contains=true;break;}}
                if(contains)continue;
                if(ccw){result.Add(ia);result.Add(ib);result.Add(ic);}else{result.Add(ia);result.Add(ic);result.Add(ib);}
                verts.RemoveAt(k);clipped=true;break;
            }
            if(!clipped) break;
        }
        if(verts.Count==3){if(ccw){result.Add(verts[0]);result.Add(verts[1]);result.Add(verts[2]);}else{result.Add(verts[0]);result.Add(verts[2]);result.Add(verts[1]);}}
        // Invalid/self-intersecting polygon: no fill rather than misleading crossed fan triangles.
        return result.Count == (poly.Count-2)*3 ? result : new List<int>();
    }

    private MeshGeometry3D? BuildCurvedZoneFace(SurfaceDefinition surface, ZoneDefinition zone, float lift)
    {
        // Rectangle zones receive a dense curved fill. Polygon zones keep a light
        // triangulated fill; the precise detection boundary remains UV, not this mesh.
        if (zone.Polygon.Count < 3) return null;
        if (zone.Type == ZoneType.Rectangle && TryGetRectangleZone(zone, out var cu, out var cv, out var w, out var h))
        {
            var u0=cu-w/2; var u1=cu+w/2; var v0=cv-h/2; var v1=cv+h/2;
            var us=Math.Clamp((int)Math.Ceiling(w/0.025),2,80); var vs=Math.Clamp((int)Math.Ceiling(h/0.05),1,40);
            var pos=new Vector3Collection(); var norms=new Vector3Collection(); var idx=new IntCollection();
            for(int j=0;j<=vs;j++) for(int i=0;i<=us;i++)
            {
                var u=u0+(u1-u0)*i/us; var v=v0+(v1-v0)*j/vs;
                var p0=SurfaceGeometry.FromUv(surface,u,v); var np=SurfaceGeometry.NormalAtUv(surface,u,v);
                var sn=Vector3.Normalize(new Vector3(np.X,np.Y,np.Z)); var sp=new Vector3(p0.X,p0.Y,p0.Z)+sn*lift;
                pos.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0))); norms.Add(SensorDirectionToDisplay(sn));
            }
            var stride=us+1;
            for(int j=0;j<vs;j++) for(int i=0;i<us;i++) { var a=j*stride+i; var b=a+1; var c=a+stride; var d=c+1; idx.Add(a);idx.Add(b);idx.Add(d);idx.Add(a);idx.Add(d);idx.Add(c); }
            return new MeshGeometry3D{Positions=pos,Normals=norms,Indices=idx};
        }
        var ppos=new Vector3Collection(); var pnorm=new Vector3Collection();
        foreach(var uv in zone.Polygon) { var p0=SurfaceGeometry.FromUv(surface,uv.U,uv.V); var np=SurfaceGeometry.NormalAtUv(surface,uv.U,uv.V); var sn=Vector3.Normalize(new Vector3(np.X,np.Y,np.Z)); var sp=new Vector3(p0.X,p0.Y,p0.Z)+sn*lift; ppos.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0))); pnorm.Add(SensorDirectionToDisplay(sn)); }
        var tri = TriangulateZoneUv(zone.Polygon); var pidx=new IntCollection(); foreach(var ti in tri)pidx.Add(ti); return tri.Count==0 ? null : new MeshGeometry3D{Positions=ppos,Normals=pnorm,Indices=pidx};
    }

    private Vector3 SensorPointToDisplay(Point3D sensor)
    {
        var d = _coordinateEngine.ToWorld(sensor);
        return new Vector3(d.X, d.Y, d.Z);
    }

    private Vector3 SensorDirectionToDisplay(Vector3 sensorDirection)
    {
        var d = _coordinateEngine.ToWorld(new Point3D(sensorDirection.X, sensorDirection.Y, sensorDirection.Z, 0, 0));
        var v = new Vector3(d.X, d.Y, d.Z);
        return v.LengthSquared() < 1e-10f ? sensorDirection : Vector3.Normalize(v);
    }

    private void PickSurfaceButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            SurfacePickStatus.Text = "PICK: hãy BUILD/LOAD MAP rồi vào SURFACE DESIGN trước.";
            return;
        }
        _surfacePickMode = true;
        _surfacePickPoints.Clear();
        PickSurfaceButton.IsEnabled = false;
        CancelSurfacePickButton.IsEnabled = true;
        Viewport.IsRotationEnabled = false;
        StaticMapModel.IsHitTestVisible = true;
        UpdateSurfacePickGeometry();
        SurfacePickStatus.Text = "PICK PLANE on STATIC MAP: P1=Origin, P2=+U/Width, P3=+V/Height. Stored as MID360_SENSOR XYZ.";
    }

    private void CancelSurfacePickButton_Click(object sender, RoutedEventArgs e) => CancelSurfacePickMode();

    private void CancelSurfacePickMode()
    {
        _surfacePickMode = false;
        _surfacePickPoints.Clear();
        PickSurfaceButton.IsEnabled = true;
        CancelSurfacePickButton.IsEnabled = false;
        Viewport.IsRotationEnabled = true;
        UpdateSurfacePickGeometry();
        SurfacePickStatus.Text = "Surface: sẵn sàng";
    }

    private void CommitPickedPlane()
    {
        if (_surfacePickPoints.Count < 3) return;
        var p0 = _surfacePickPoints[0];
        var p1 = _surfacePickPoints[1];
        var p2 = _surfacePickPoints[2];
        var e1 = p1 - p0;
        var e2 = p2 - p0;
        if (e1.LengthSquared() < 1e-6f || e2.LengthSquared() < 1e-6f)
        {
            SurfacePickStatus.Text = "PICK lỗi: các điểm quá gần nhau.";
            return;
        }
        var u = Vector3.Normalize(e1);
        var normalRaw = Vector3.Cross(e1, e2);
        if (normalRaw.LengthSquared() < 1e-6f)
        {
            SurfacePickStatus.Text = "PICK lỗi: P1/P2/P3 gần thẳng hàng. Chọn lại.";
            return;
        }
        var normal = Vector3.Normalize(normalRaw);
        var v = Vector3.Normalize(Vector3.Cross(normal, u));
        var width = e1.Length();
        var height = MathF.Abs(Vector3.Dot(e2, v));
        if (height < 0.02f)
        {
            SurfacePickStatus.Text = "PICK lỗi: chiều cao surface quá nhỏ. Chọn P3 lệch khỏi P1-P2.";
            return;
        }

        var surface = SurfaceList.SelectedItem as SurfaceDefinition;
        if (surface is null)
        {
            var next = 1;
            while (_project.Surfaces.Any(x => x.Id.Equals($"S{next:00}", StringComparison.OrdinalIgnoreCase))) next++;
            surface = new SurfaceDefinition { Id = $"S{next:00}", Name = $"Surface {next:00}" };
            _project.Surfaces.Add(surface);
        }

        surface.Frame = "MID360_SENSOR";
        surface.Type = SurfaceType.Plane;
        surface.Origin = new Point3D(p0.X, p0.Y, p0.Z, 0, 0);
        surface.UAxis = new Point3D(u.X, u.Y, u.Z, 0, 0);
        surface.VAxis = new Point3D(v.X, v.Y, v.Z, 0, 0);
        surface.Normal = new Point3D(normal.X, normal.Y, normal.Z, 0, 0);
        surface.WidthMeters = width;
        surface.HeightMeters = height;
        surface.ValidationRmseMm = Math.Abs(Vector3.Dot(e2, normal)) * 1000.0;

        _surfacePickMode = false;
        PickSurfaceButton.IsEnabled = true;
        CancelSurfacePickButton.IsEnabled = false;
        Viewport.IsRotationEnabled = true;
        SurfaceList.ItemsSource = null;
        SurfaceList.ItemsSource = _project.Surfaces;
        SurfaceList.SelectedItem = surface;
        UpdateSurfacePickGeometry();
        RenderSelectedSurface();
        RefreshZones();
        SurfacePickStatus.Text = $"{surface.Id} SAVED SENSOR | W={width:0.###} m H={height:0.###} m | plane residual={surface.ValidationRmseMm:0.0} mm";
        DiagnosticStatus.Text = $"Surface {surface.Id}: geometry saved in MID-360 SENSOR frame. IMU/View alignment is display-only.";
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void UpdateSurfacePickGeometry()
    {
        if (_surfacePickPoints.Count == 0)
        {
            SurfacePickPointsModel.Geometry = null;
            return;
        }
        var positions = new Vector3Collection();
        foreach (var sp in _surfacePickPoints)
            positions.Add(SensorPointToDisplay(new Point3D(sp.X, sp.Y, sp.Z, 0, 0)));
        SurfacePickPointsModel.Geometry = new PointGeometry3D { Positions = positions };
    }

    private void UpdateTagDiagnostics(PointCloudFrame frame)
    {
        // Tag telemetry is diagnostic-only. Do not update it at the 20 FPS renderer rate:
        // changing number widths caused WPF to repeatedly measure/arrange the sidebar.
        // Keep acquisition/rendering realtime, but refresh this fixed-size panel at 2 Hz.
        var now = Stopwatch.GetTimestamp();
        var previous = Volatile.Read(ref _lastTagUiTicks);
        if (previous != 0)
        {
            var elapsedMs = (now - previous) * 1000.0 / Stopwatch.Frequency;
            if (elapsedMs < TagUiIntervalMs) return;
        }
        Volatile.Write(ref _lastTagUiTicks, now);

        int normal = 0, atmosphereModerate = 0, atmosphereLow = 0, dragging = 0, other = 0, reserved = 0;
        foreach (var p in frame.Points)
        {
            var t = LivoxTagDecoder.Decode(p.Tag);
            if (t.IsNormal) normal++;
            if (t.Atmospheric == LivoxTagConfidence.Moderate) atmosphereModerate++;
            else if (t.Atmospheric == LivoxTagConfidence.Low) atmosphereLow++;
            else if (t.Atmospheric == LivoxTagConfidence.Reserved) reserved++;
            if (t.Dragging != LivoxTagConfidence.HighConfidenceNormal) dragging++;
            if (t.Other != LivoxTagConfidence.HighConfidenceNormal) other++;
        }

        // Fixed-width numeric fields + fixed-height/no-wrap XAML keep layout geometry constant.
        TagStatsText.Text =
            $"Normal  {normal,8:D}   Atmos M {atmosphereModerate,8:D}   Atmos L {atmosphereLow,8:D}\n" +
            $"Drag    {dragging,8:D}   Other   {other,8:D}   Reserv  {reserved,8:D}";
        SmokeCandidateText.Text = $"Atmos candidates  {atmosphereModerate + atmosphereLow,8:D}";
    }

    private void HealthTimer_Tick(object? sender, EventArgs e)
    {
        if (_isShuttingDown || !_livox.IsConnected)
        {
            HmsStatusText.Text = "HMS: not connected";
            HmsDetailsText.Text = "-";
            return;
        }
        try
        {
            _livox.RequestHms();
            var diagnostics = _livox.GetHmsCodes().Select(Mid360HmsDecoder.Decode).ToList();
            if (diagnostics.Count == 0)
            {
                HmsStatusText.Text = _livox.HmsQueryStatus < 0 ? $"HMS query status: {_livox.HmsQueryStatus}" : "HMS: OK / no active diagnostic code";
                HmsStatusText.Foreground = WpfBrushes.LightGreen;
                HmsDetailsText.Text = "No active HMS warning reported.";
                return;
            }

            var worst = diagnostics.Max(d => d.Severity);
            HmsStatusText.Text = $"HMS: {worst.ToString().ToUpperInvariant()} | {diagnostics.Count} active";
            HmsStatusText.Foreground = worst switch
            {
                HmsSeverity.Fatal => WpfBrushes.Red,
                HmsSeverity.Error => WpfBrushes.OrangeRed,
                HmsSeverity.Warning => WpfBrushes.Orange,
                _ => WpfBrushes.LightSkyBlue
            };
            HmsDetailsText.Text = string.Join("\n", diagnostics.Select(d =>
                $"0x{d.AbnormalId:X4} [{d.Severity}] {d.Description}\n→ {d.SuggestedAction}"));
            if (worst == HmsSeverity.Fatal)
                SystemStatus.Text = "SYSTEM: MID-360 FATAL HMS";
        }
        catch (Exception ex)
        {
            HmsStatusText.Text = "HMS: read fault";
            HmsStatusText.Foreground = WpfBrushes.OrangeRed;
            HmsDetailsText.Text = ex.Message;
        }
    }

    private void ImuTimer_Tick(object? sender, EventArgs e)
    {
        if (_isShuttingDown) return;

        if (!_livox.IsConnected || !_livox.TryGetImuSnapshot(out var imu))
        {
            _lastImuSnapshot = null;
            ImuStatusText.Text = _livox.IsConnected ? "IMU: waiting for MID-360 data" : "IMU: not connected";
            ImuStatusText.Foreground = WpfBrushes.Gray;
            ImuAnglesText.Text = "Roll --°   Pitch --°   Tilt --°";
            ImuRawText.Text = "ACC -- -- -- g\nGYRO -- -- -- °/s";
            ImuQualityText.Text = "Window: 0 | waiting for samples";
            UpdateImuReferenceText();
            CalibrateZButton.IsEnabled = false;
            return;
        }

        _lastImuSnapshot = imu;
        var accel = new Vector3(imu.AvgAccXG, imu.AvgAccYG, imu.AvgAccZG);
        if (accel.LengthSquared() < 1e-8f) return;

        // A stationary accelerometer reports specific force opposite gravity.
        // Default Project Z0+ is DOWN (same direction as gravity), matching the
        // installation convention selected for this project.
        var down = -Vector3.Normalize(accel);
        var zForDisplay = _project.Calibration.ZPositiveDown ? down : -down;
        var angles = CalculateTiltAngles(zForDisplay);

        ImuStatusText.Text = imu.IsStable ? "IMU: STABLE" : $"IMU: WAITING ({imu.StabilityReason})";
        ImuStatusText.Foreground = imu.IsStable ? WpfBrushes.LightGreen : WpfBrushes.Orange;
        ImuAnglesText.Text = $"Roll {angles.RollDeg,+7:0.00}°   Pitch {angles.PitchDeg,+7:0.00}°   Tilt {angles.TiltDeg,6:0.00}°";
        const double radToDeg = 180.0 / Math.PI;
        ImuRawText.Text =
            $"ACC  {imu.AvgAccXG,+7:0.000} {imu.AvgAccYG,+7:0.000} {imu.AvgAccZG,+7:0.000} g\n" +
            $"GYRO {imu.AvgGyroXRadS * radToDeg,+7:0.000} {imu.AvgGyroYRadS * radToDeg,+7:0.000} {imu.AvgGyroZRadS * radToDeg,+7:0.000} °/s";
        ImuQualityText.Text =
            $"N {imu.WindowSamples,3}   |a| {imu.AccelMagnitudeG,5:0.000} g   σa {imu.AccelStdG,6:0.0000} g   σgyro {imu.GyroRmsDegS,6:0.000} °/s";
        UpdateImuReferenceText();
        CalibrateZButton.IsEnabled = imu.IsStable && !_project.Calibration.CoordinateFrameLocked;
    }

    private static (double RollDeg, double PitchDeg, double TiltDeg) CalculateTiltAngles(Vector3 zAxisInSensor)
    {
        var z = Vector3.Normalize(zAxisInSensor);
        var roll = Math.Atan2(z.Y, z.Z) * 180.0 / Math.PI;
        var pitch = Math.Atan2(-z.X, Math.Sqrt(z.Y * z.Y + z.Z * z.Z)) * 180.0 / Math.PI;
        var dot = Math.Clamp(Vector3.Dot(Vector3.UnitZ, z), -1f, 1f);
        var tilt = Math.Acos(dot) * 180.0 / Math.PI;
        return (roll, pitch, tilt);
    }

    private void CalibrateZ_Click(object sender, RoutedEventArgs e)
    {
        if (_project.Calibration.CoordinateFrameLocked)
        {
            CoordinateFrameStatusText.Text = "FRAME LOCKED: unlock/reset before recalibration.";
            return;
        }
        if (_lastImuSnapshot is not LivoxImuSnapshot imu || !imu.IsStable)
        {
            CoordinateFrameStatusText.Text = "IMU is not stable yet. Keep MID-360 fixed and wait for STABLE.";
            return;
        }

        var accel = new Vector3(imu.AvgAccXG, imu.AvgAccYG, imu.AvgAccZG);
        if (accel.LengthSquared() < 1e-8f)
        {
            CoordinateFrameStatusText.Text = "IMU acceleration vector is invalid.";
            return;
        }

        var down = -Vector3.Normalize(accel);
        _calibratedZAxisSensor = _project.Calibration.ZPositiveDown ? down : -down;
        var angles = CalculateTiltAngles(_calibratedZAxisSensor.Value);
        _project.Calibration.ImuCalibrated = true;
        _project.Calibration.ImuRollDeg = angles.RollDeg;
        _project.Calibration.ImuPitchDeg = angles.PitchDeg;
        _project.Calibration.ImuTiltDeg = angles.TiltDeg;
        ApplyProjectFrameFromCalibration();
        UpdateImuReferenceText();
        SetXDirectionButton.IsEnabled = true;
        CoordinateFrameStatusText.Text =
            $"Z0 CALIBRATED from IMU | Roll {angles.RollDeg:+0.00;-0.00;0.00}° | Pitch {angles.PitchDeg:+0.00;-0.00;0.00}° | Z0 {(_project.Calibration.ZPositiveDown ? "DOWN" : "UP")} | now SET X0 IN VIEWER";
    }

    private void FlipZ_Click(object sender, RoutedEventArgs e)
    {
        if (_project.Calibration.CoordinateFrameLocked)
        {
            CoordinateFrameStatusText.Text = "FRAME LOCKED: unlock/reset before changing Z0.";
            return;
        }
        _project.Calibration.ZPositiveDown = !_project.Calibration.ZPositiveDown;
        if (_calibratedZAxisSensor.HasValue)
        {
            _calibratedZAxisSensor = -_calibratedZAxisSensor.Value;
            var angles = CalculateTiltAngles(_calibratedZAxisSensor.Value);
            _project.Calibration.ImuRollDeg = angles.RollDeg;
            _project.Calibration.ImuPitchDeg = angles.PitchDeg;
            _project.Calibration.ImuTiltDeg = angles.TiltDeg;
            ApplyProjectFrameFromCalibration();
        }
        CoordinateFrameStatusText.Text = $"Z0 direction: {(_project.Calibration.ZPositiveDown ? "DOWN / gravity" : "UP / opposite gravity")}";
    }

    private void SetXDirection_Click(object sender, RoutedEventArgs e)
    {
        if (_project.Calibration.CoordinateFrameLocked)
        {
            CoordinateFrameStatusText.Text = "FRAME LOCKED: unlock/reset before changing X0.";
            return;
        }
        if (!_project.Calibration.ImuCalibrated || !_calibratedZAxisSensor.HasValue)
        {
            CoordinateFrameStatusText.Text = "Calibrate Z0 from IMU before setting X0.";
            return;
        }

        _setX0DirectionMode = true;
        _x0PreviewDirectionProject = Vector3.UnitX;
        SetXDirectionButton.IsEnabled = false;
        CancelXDirectionButton.IsEnabled = true;
        Viewport.IsRotationEnabled = false;
        TopView_Click(this, new RoutedEventArgs());
        UpdateWorldCoordinateFramePreview(Vector3.UnitX);
        XDirectionHintText.Text = "Move cursor around MID-360 origin. Red arrow previews X0. Left-click to confirm; Esc cancels.";
        CoordinateFrameStatusText.Text = "SET X0 MODE: choose the +X0 direction in TOP view. The program will calculate yaw relative to leveled LiDAR +X.";
    }

    private void CancelXDirection_Click(object sender, RoutedEventArgs e) => CancelX0DirectionMode();

    private void CancelX0DirectionMode()
    {
        if (!_setX0DirectionMode) return;
        _setX0DirectionMode = false;
        SetXDirectionButton.IsEnabled = true;
        CancelXDirectionButton.IsEnabled = false;
        Viewport.IsRotationEnabled = true;
        UpdateWorldCoordinateFrame();
        XDirectionValueText.Text = $"Calculated yaw: {_project.Calibration.UserXRotationDeg:+0.00;-0.00;0.00}°";
        XDirectionHintText.Text = "SET X0 → move the red arrow in TOP view → left-click to confirm.";
        UpdateCoordinateFrameStatus();
    }

    private bool TryGetX0DirectionFromViewer(System.Windows.Point mousePosition, out Vector3 directionProject)
    {
        directionProject = Vector3.UnitX;
        // Once Z0 has been calibrated, Project Z is canonical +Z.  The user
        // therefore chooses X0 on the Project XY plane through the MID-360 origin.
        var hit = Viewport.UnProjectOnPlane(
            mousePosition,
            new MediaPoint3D(0, 0, 0),
            new MediaVector3D(0, 0, 1));
        if (!hit.HasValue) return false;

        var d = new Vector3((float)hit.Value.X, (float)hit.Value.Y, 0f);
        if (!float.IsFinite(d.X) || !float.IsFinite(d.Y) || d.LengthSquared() < 0.01f)
            return false;
        directionProject = Vector3.Normalize(d);
        return true;
    }

    private double CalculateX0YawForProjectDirection(Vector3 directionProject)
    {
        if (!_calibratedZAxisSensor.HasValue) return 0.0;
        var desiredSensorPoint = _coordinateEngine.ToSensor(
            new Point3D(directionProject.X, directionProject.Y, directionProject.Z, 0, 0));
        var desiredXSensor = Vector3.Normalize(new Vector3(
            desiredSensorPoint.X, desiredSensorPoint.Y, desiredSensorPoint.Z));
        return NormalizeAngleDeg(CoordinateEngine.CalculateX0YawFromSensorX(
            _calibratedZAxisSensor.Value, desiredXSensor));
    }

    private void CommitX0Direction(Vector3 directionProject)
    {
        if (!_calibratedZAxisSensor.HasValue) return;

        // Convert the selected viewer direction back into Sensor coordinates,
        // then calculate the signed yaw relative to the physical MID-360 +X
        // after leveling. The user never has to enter this angle manually.
        _project.Calibration.UserXRotationDeg = CalculateX0YawForProjectDirection(directionProject);
        ApplyProjectFrameFromCalibration();

        _setX0DirectionMode = false;
        SetXDirectionButton.IsEnabled = true;
        CancelXDirectionButton.IsEnabled = false;
        Viewport.IsRotationEnabled = true;
        UpdateWorldCoordinateFrame();
        XDirectionValueText.Text = $"Calculated yaw: {_project.Calibration.UserXRotationDeg:+0.00;-0.00;0.00}°";
        XDirectionHintText.Text = "X0 locked to the selected direction. Press SET X0 IN VIEWER to choose again.";
        CoordinateFrameStatusText.Text =
            $"X0 SET | calculated yaw {_project.Calibration.UserXRotationDeg:+0.00;-0.00;0.00}° relative to leveled MID-360 +X | Y0 auto = Z0 × X0";
    }

    private void ApplyProjectFrameFromCalibration()
    {
        if (!_calibratedZAxisSensor.HasValue) return;
        var result = _coordinateEngine.ConfigureGravityFrame(
            _calibratedZAxisSensor.Value, _project.Calibration.UserXRotationDeg);
        _project.Calibration.LidarToWorld = CoordinateEngine.ToRowMajorArray(result.SensorToProject);
        _renderHistory.Clear();
        Interlocked.Exchange(ref _latestFrame, null);
        Interlocked.Exchange(ref _latestDetectionFrame, null);
        // Surfaces/picks remain in SENSOR coordinates; redraw only their display copies.
        RenderSelectedSurface();
        UpdateSurfacePickGeometry();
        if (_staticMap is not null) RenderStaticMap();
        XDirectionValueText.Text = $"Calculated yaw: {_project.Calibration.UserXRotationDeg:+0.00;-0.00;0.00}°";
        UpdateCoordinateFrameStatus();
    }

    private void LockFrame_Click(object sender, RoutedEventArgs e)
    {
        if (!_project.Calibration.ImuCalibrated || !_calibratedZAxisSensor.HasValue)
        {
            CoordinateFrameStatusText.Text = "Calibrate Z from IMU before locking the Project frame.";
            return;
        }
        _project.Calibration.CoordinateFrameLocked = !_project.Calibration.CoordinateFrameLocked;
        if (_project.Calibration.CoordinateFrameLocked && _setX0DirectionMode)
            CancelX0DirectionMode();
        LockFrameButton.Content = _project.Calibration.CoordinateFrameLocked ? "UNLOCK FRAME" : "LOCK FRAME";
        UpdateImuReferenceText();
        CalibrateZButton.IsEnabled = !_project.Calibration.CoordinateFrameLocked && (_lastImuSnapshot?.IsStable ?? false);
        SetXDirectionButton.IsEnabled = !_project.Calibration.CoordinateFrameLocked && _project.Calibration.ImuCalibrated;
        UpdateCoordinateFrameStatus();
    }

    private void ResetCoordinateFrame_Click(object sender, RoutedEventArgs e)
    {
        _setX0DirectionMode = false;
        Viewport.IsRotationEnabled = true;
        SetXDirectionButton.IsEnabled = true;
        CancelXDirectionButton.IsEnabled = false;
        UpdateWorldCoordinateFrame();
        _project.Calibration = new CalibrationDefinition();
        _coordinateEngine.Configure(_project.Calibration);
        _calibratedZAxisSensor = null;
        _renderHistory.Clear();
        Interlocked.Exchange(ref _latestFrame, null);
        Interlocked.Exchange(ref _latestDetectionFrame, null);
        XDirectionValueText.Text = "Calculated yaw: +0.00°";
        XDirectionHintText.Text = "Calibrate Z0 first. Then SET X0 → move the red arrow in TOP view → left-click to confirm.";
        LockFrameButton.Content = "LOCK FRAME";
        CoordinateFrameStatusText.Text = "FRAME: Sensor = Project | IMU not calibrated";
        UpdateImuReferenceText();
    }

    private void UpdateImuReferenceText()
    {
        if (!_project.Calibration.ImuCalibrated)
        {
            ImuReferenceText.Text = "Z0 not calibrated";
            ImuReferenceText.Foreground = WpfBrushes.Gray;
            return;
        }

        ImuReferenceText.Foreground = WpfBrushes.LightSkyBlue;
        ImuReferenceText.Text =
            $"Roll ref  {_project.Calibration.ImuRollDeg,+7:0.00}°\n" +
            $"Pitch ref {_project.Calibration.ImuPitchDeg,+7:0.00}°   Tilt ref {_project.Calibration.ImuTiltDeg,6:0.00}°\n" +
            $"Z0 {(_project.Calibration.ZPositiveDown ? "DOWN" : "UP")} | {(_project.Calibration.CoordinateFrameLocked ? "FRAME LOCKED" : "FRAME EDIT")}";
    }

    private void RestoreCoordinateCalibrationState()
    {
        XDirectionValueText.Text = $"Calculated yaw: {_project.Calibration.UserXRotationDeg:+0.00;-0.00;0.00}°";
        XDirectionHintText.Text = _project.Calibration.ImuCalibrated
            ? "SET X0 → move the red arrow in TOP view → left-click to confirm."
            : "Calibrate Z0 first. Then SET X0 → move the red arrow in TOP view → left-click to confirm.";
        LockFrameButton.Content = _project.Calibration.CoordinateFrameLocked ? "UNLOCK FRAME" : "LOCK FRAME";
        SetXDirectionButton.IsEnabled = !_project.Calibration.CoordinateFrameLocked && _project.Calibration.ImuCalibrated;
        CancelXDirectionButton.IsEnabled = false;
        if (_project.Calibration.ImuCalibrated)
            _calibratedZAxisSensor = _coordinateEngine.ProjectZAxisInSensor;
        UpdateImuReferenceText();
        UpdateCoordinateFrameStatus();
    }

    private void UpdateCoordinateFrameStatus()
    {
        if (!_project.Calibration.ImuCalibrated)
        {
            CoordinateFrameStatusText.Text = "FRAME: Sensor = Project | IMU not calibrated";
            return;
        }
        var x = _coordinateEngine.ProjectXAxisInSensor;
        var y = _coordinateEngine.ProjectYAxisInSensor;
        var z = _coordinateEngine.ProjectZAxisInSensor;
        CoordinateFrameStatusText.Text =
            $"FRAME {(_project.Calibration.CoordinateFrameLocked ? "LOCKED" : "EDIT")} | O=MID-360 | Z0={(_project.Calibration.ZPositiveDown ? "DOWN" : "UP")} | calculated X0 yaw {_project.Calibration.UserXRotationDeg:+0.00;-0.00;0.00}°\n" +
            $"X0s({x.X:0.000},{x.Y:0.000},{x.Z:0.000}) Y0s({y.X:0.000},{y.Y:0.000},{y.Z:0.000}) Z0s({z.X:0.000},{z.Y:0.000},{z.Z:0.000})";
    }

    private static double NormalizeAngleDeg(double a)
    {
        a %= 360.0;
        if (a > 180.0) a -= 360.0;
        if (a <= -180.0) a += 360.0;
        return a;
    }

    private async Task StopSourceAsync()
    {
        var source = _activeSource;
        var task = _acquisitionTask;
        var detectionTask = _detectionTask;
        var cts = _cts;

        _activeSource = null;
        _acquisitionTask = null;
        _detectionTask = null;
        _cts = null;
        cts?.Cancel();

        if (task is not null || detectionTask is not null)
        {
            try
            {
                var waits = new List<Task>();
                if (task is not null) waits.Add(task);
                if (detectionTask is not null) waits.Add(detectionTask);
                await Task.WhenAny(Task.WhenAll(waits), Task.Delay(1200)).ConfigureAwait(true);
            }
            catch { }
        }

        if (source is not null)
        {
            try
            {
                await Task.Run(source.Disconnect).ConfigureAwait(true);
            }
            catch { }
        }
        cts?.Dispose();

        _zoneEngine.Reset();
        Volatile.Write(ref _lastFrameReceivedTicks, 0);
        Interlocked.Exchange(ref _latestFrame, null);
        Interlocked.Exchange(ref _latestDetectionFrame, null);
        _renderHistory.Clear();
        _surfacePickMode = false;
        _surfacePickPoints.Clear();
        PickSurfaceButton.IsEnabled = true;
        CancelSurfacePickButton.IsEnabled = false;
        PointCloudModel.IsHitTestVisible = false;
        UpdateSurfacePickGeometry();

        SimulatorButton.Content = "Start Simulator";
        LivoxButton.Content = "Connect MID-360";
        SourceStatus.Text = "Source: None";
        SourceStatus.Foreground = WpfBrushes.White;
        SystemStatus.Text = "SYSTEM: IDLE";
        DiagnosticStatus.Text = "No source selected";
    }

    private async void MainWindow_Closed(object? sender, EventArgs e)
    {
        if (_isShuttingDown) return;
        _isShuttingDown = true;
        _renderTimer.Stop();
        _healthTimer.Stop();
        _imuTimer.Stop();
        if (_staticMapBuilder.IsBuilding) _staticMapBuilder.Cancel();
        await StopSourceAsync();
        try
        {
            if (_waveShell is not null && !_waveShell.IsDisposed)
            {
                _waveShell.Hide();
                _waveShell.Dispose();
            }
            _waveService?.Dispose();
        }
        catch { }
        _livox.Dispose();
        _simulator.Dispose();
        EffectsManager.Dispose();
    }



    private async void FitOcctStaticMapButton_Click(object sender, RoutedEventArgs e)
    {
        if (SurfaceList.SelectedItem is not SurfaceDefinition surface)
        {
            OcctFitStatusText.Text = "FIT: chọn OCCT B-Spline Surface trước.";
            return;
        }
        if (_staticMap is null || _staticMap.Points.Count == 0)
        {
            OcctFitStatusText.Text = "FIT: cần BUILD/LOAD STATIC MAP trước.";
            return;
        }
        if (surface.Type != SurfaceType.Curved || surface.CurvedKind != CurvedSurfaceKind.OcctBSpline)
        {
            OcctFitStatusText.Text = "FIT: Surface đang chọn không phải OCCT B-Spline.";
            return;
        }
        if (!SurfaceGeometry.IsOcctAvailable(out var occt))
        {
            OcctFitStatusText.Text = $"FIT: OCCT NOT READY | {occt}";
            return;
        }
        var depth = 0.30;
        if (OcctFitDepthCombo.SelectedItem is System.Windows.Controls.ComboBoxItem item &&
            double.TryParse(item.Tag?.ToString(), System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var parsed)) depth = parsed;
        FitOcctStaticMapButton.IsEnabled = false;
        OcctFitStatusText.Text = "FIT: processing static SENSOR map...";
        try
        {
            var map = _staticMap;
            // Fit a detached copy off the UI/acquisition path; publish geometry atomically afterwards.
            var fitSurface = new SurfaceDefinition
            {
                Frame=surface.Frame, Id=surface.Id, Name=surface.Name, Type=surface.Type, CurvedKind=surface.CurvedKind,
                Origin=surface.Origin, Normal=surface.Normal, UAxis=surface.UAxis, VAxis=surface.VAxis,
                WidthMeters=surface.WidthMeters, HeightMeters=surface.HeightMeters,
                OcctBulgeMeters=surface.OcctBulgeMeters, OcctTwistMeters=surface.OcctTwistMeters
            };
            var result = await Task.Run(() => OcctStaticMapFitter.Fit(fitSurface, map, 6, 6, depth));
            surface.OcctFitUCount=fitSurface.OcctFitUCount; surface.OcctFitVCount=fitSurface.OcctFitVCount;
            surface.OcctFitControlPoints=fitSurface.OcctFitControlPoints; surface.OcctFitSourcePointCount=fitSurface.OcctFitSourcePointCount;
            surface.OcctFitRmseMm=fitSurface.OcctFitRmseMm; surface.ValidationRmseMm=fitSurface.ValidationRmseMm;
            OcctFitStatusText.Text = result.Message + $" | max {result.MaxAbsMm:0.0} mm";
            SurfacePickStatus.Text = $"{surface.Id}: OCCT fitted from static map | SENSOR frame";
            RenderSelectedSurface(); RenderAllZones();
            await _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
        }
        catch (Exception ex)
        {
            OcctFitStatusText.Text = "FIT FAILED: " + ex.Message;
        }
        finally { FitOcctStaticMapButton.IsEnabled = true; }
    }

    private void TouchDetectionButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_touchDetector.Enabled)
        {
            if (_staticMap is null || _staticMap.Points.Count == 0)
            {
                TouchDetectionStatusText.Text = "TOUCH: cần locked/static map trước.";
                return;
            }
            if (_project.Zones.Count == 0)
            {
                TouchDetectionStatusText.Text = "TOUCH: chưa có Zone.";
                return;
            }
            _zoneEngine.Reset();
            _touchDetector.Enabled = true;
            TouchDetectionButton.Content = "STOP TOUCH DETECTION";
            TouchDetectionStatusText.Text = "STATIC TOUCH: ON";
        }
        else
        {
            _touchDetector.Enabled = false;
            Interlocked.Exchange(ref _latestTouchEvidence, Array.Empty<TouchEvidence>());
            TouchEvidenceModel.Geometry = null;
            _zoneEngine.Reset();
            TouchDetectionButton.Content = "START TOUCH DETECTION";
            TouchDetectionStatusText.Text = "STATIC TOUCH: OFF";
        }
    }

    private void RenderTouchEvidence()
    {
        if (!_touchDetector.Enabled)
        {
            TouchEvidenceModel.Geometry = null;
            return;
        }
        var candidates = Volatile.Read(ref _latestTouchEvidence);
        if (candidates.Count == 0)
        {
            TouchEvidenceModel.Geometry = null;
            TouchDetectionStatusText.Text = _touchDetector.Diagnostics;
            return;
        }
        var positions = new Vector3Collection();
        foreach (var c in candidates.Take(32)) positions.Add(SensorPointToDisplay(c.Position));
        TouchEvidenceModel.Geometry = new PointGeometry3D { Positions = positions };
        var states = _zoneEngine.Snapshot();
        var active = states.Where(x => x.Value == ZoneState.Active).Select(x => x.Key).ToArray();
        TouchDetectionStatusText.Text = $"{_touchDetector.Diagnostics} | ACTIVE {(active.Length == 0 ? "--" : string.Join(",", active))}";
    }

    private void BuildMapButton_Click(object sender, RoutedEventArgs e)
    {
        if (_activeSource is null)
        {
            WpfMessageBox.Show("Connect MID-360 (or start Simulator) before building a static map.",
                "Static Map", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        // Building happens in the background while the operator keeps the LIVE view.
        // Starting a new build invalidates the previous editing reference.
        ExitSurfaceDesignMode();
        _staticMapBuilder.Start(_project.StaticMapSettings);
        _staticMap = null;
        StaticMapModel.Geometry = null;
        StaticMapModel.Visibility = Visibility.Collapsed;
        PointCloudModel.Visibility = Visibility.Visible;
        EnterDesignModeButton.IsEnabled = false;
        _project.StaticMap.Locked = false;
        _project.StaticMap.PointCount = 0;
        BuildMapButton.IsEnabled = false;
        FinalizeMapButton.IsEnabled = true;
        StaticMapStatusText.Text = "MAP: BUILDING ...";
        SystemStatus.Text = "SYSTEM: STATIC MAP BUILDING";
    }

    private async void FinalizeMapButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_staticMapBuilder.IsBuilding) return;
        var map = await _staticMapBuilder.FinalizeMapAsync();
        if (map.Points.Count == 0)
        {
            BuildMapButton.IsEnabled = true;
            FinalizeMapButton.IsEnabled = false;
            StaticMapStatusText.Text = "MAP: NO STABLE VOXELS - build longer or check scene";
            return;
        }

        _staticMap = map;
        _project.StaticMap.Locked = true;
        _project.StaticMap.PointCount = map.Points.Count;
        _project.StaticMap.SourceFrames = map.SourceFrames;
        _project.StaticMap.VoxelSizeMeters = map.VoxelSizeMeters;
        _project.StaticMap.CreatedAtUtc = map.CreatedAtUtc;
        RenderStaticMap();
        ApplyViewerMode(); // stays LIVE until the operator explicitly enters Surface Design
        BuildMapButton.IsEnabled = true;
        FinalizeMapButton.IsEnabled = false;
        EnterDesignModeButton.IsEnabled = true;
        StaticMapStatusText.Text = $"MAP: READY | {map.Points.Count:N0} pts | {map.SourceFrames:N0} frames";
        StaticMapQualityText.Text = $"Voxel {map.VoxelSizeMeters * 1000:0.#} mm | stable {map.StableVoxels:N0}/{map.CandidateVoxels:N0}";
        SystemStatus.Text = "SYSTEM: LIVE | STATIC MAP READY FOR SURFACE DESIGN";
    }

    private async void SaveMapButton_Click(object sender, RoutedEventArgs e)
    {
        if (_staticMap is null)
        {
            WpfMessageBox.Show("Build and LOCK a static map first.", "Static Map",
                MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        try
        {
            await _staticMapStore.SaveAsync(_staticMap, _project.StaticMap.FilePath);
            _project.StaticMap.Locked = true;
            await _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
            StaticMapStatusText.Text = $"MAP: SAVED | {_staticMap.Points.Count:N0} pts";
            DiagnosticStatus.Text = $"Static map saved: {Path.GetFullPath(_project.StaticMap.FilePath)}";
        }
        catch (Exception ex)
        {
            WpfMessageBox.Show(ex.ToString(), "Save static map failed", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private async void LoadMapButton_Click(object sender, RoutedEventArgs e)
    {
        await TryLoadStaticMapAsync(showWarning: true);
    }

    private async Task TryLoadStaticMapAsync(bool showWarning)
    {
        try
        {
            var path = _project.StaticMap.FilePath;
            if (string.IsNullOrWhiteSpace(path) || !File.Exists(Path.GetFullPath(path)))
            {
                if (showWarning)
                    WpfMessageBox.Show("No saved static map was found for this project.", "Static Map",
                        MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            _staticMap = await _staticMapStore.LoadAsync(path);
            _project.StaticMap.Locked = true;
            _project.StaticMap.PointCount = _staticMap.Points.Count;
            _project.StaticMap.SourceFrames = _staticMap.SourceFrames;
            _project.StaticMap.VoxelSizeMeters = _staticMap.VoxelSizeMeters;
            _project.StaticMap.CreatedAtUtc = _staticMap.CreatedAtUtc;
            RenderStaticMap();
            EnterDesignModeButton.IsEnabled = true;
            ApplyViewerMode(); // loading a map must not replace the normal LIVE runtime view
            StaticMapStatusText.Text = $"MAP: LOADED / READY | {_staticMap.Points.Count:N0} pts";
            StaticMapQualityText.Text = $"Voxel {_staticMap.VoxelSizeMeters * 1000:0.#} mm | frame MID360_SENSOR";
        }
        catch (Exception ex)
        {
            if (showWarning)
                WpfMessageBox.Show(ex.ToString(), "Load static map failed", MessageBoxButton.OK, MessageBoxImage.Error);
            else
                DiagnosticStatus.Text = $"Static map load skipped: {ex.Message}";
        }
    }

    private void ClearMapButton_Click(object sender, RoutedEventArgs e)
    {
        if (_staticMapBuilder.IsBuilding) _staticMapBuilder.Cancel();
        ExitSurfaceDesignMode();
        _staticMap = null;
        StaticMapModel.Geometry = null;
        StaticMapModel.Visibility = Visibility.Collapsed;
        PointCloudModel.Visibility = Visibility.Visible;
        _project.StaticMap.Locked = false;
        _project.StaticMap.PointCount = 0;
        BuildMapButton.IsEnabled = true;
        FinalizeMapButton.IsEnabled = false;
        EnterDesignModeButton.IsEnabled = false;
        StaticMapStatusText.Text = "MAP: EMPTY";
        StaticMapQualityText.Text = $"Voxel {_project.StaticMapSettings.VoxelSizeMeters * 1000:0.#} mm | Stable: --";
    }

    private void UpdateStaticMapProgressUi()
    {
        if (!_staticMapBuilder.IsBuilding) return;
        var p = _staticMapBuilder.GetProgress();
        var ratio = p.CandidateVoxels == 0 ? 0.0 : 100.0 * p.StableVoxels / p.CandidateVoxels;
        StaticMapStatusText.Text = $"MAP: BUILDING | frames {p.Frames:N0} | voxels {p.CandidateVoxels:N0}";
        StaticMapQualityText.Text = $"Stable {p.StableVoxels:N0} ({ratio:0.0}%) | accepted {p.AcceptedPoints:N0} | map-drop {p.DroppedFrames:N0}";
    }

    private void RenderStaticMap()
    {
        var map = _staticMap;
        if (map is null || map.Points.Count == 0)
        {
            StaticMapModel.Geometry = null;
            StaticMapModel.Visibility = Visibility.Collapsed;
            return;
        }

        // Stored map stays in SENSOR XYZ. Only this display copy is transformed.
        var maxDisplay = 300_000;
        var stride = Math.Max(1, (int)Math.Ceiling(map.Points.Count / (double)maxDisplay));
        var geometry = new PointGeometry3D
        {
            Positions = new Vector3Collection(),
            Colors = new()
        };
        for (var i = 0; i < map.Points.Count; i += stride)
        {
            var sensor = map.Points[i];
            var display = _coordinateEngine.ToWorld(sensor);
            geometry.Positions.Add(new Vector3(display.X, display.Y, display.Z));
            geometry.Colors.Add(new HelixColor4(0.82f, 0.88f, 0.92f, 1f));
        }
        StaticMapModel.Geometry = geometry;
        ApplyViewerMode();
        Viewport.InvalidateRender();
    }

    private void EnterDesignModeButton_Click(object sender, RoutedEventArgs e)
    {
        if (_staticMap is null || _staticMap.Points.Count == 0)
        {
            WpfMessageBox.Show("Build or LOAD a static map first.", "Surface Design",
                MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        _surfaceDesignMode = true;
        ApplyViewerMode();
        DiagnosticStatus.Text = "SURFACE DESIGN: static MID360_SENSOR map is frozen. Surface geometry will be stored in Sensor XYZ.";
        SystemStatus.Text = "SYSTEM: SURFACE DESIGN";
    }

    private void ExitDesignModeButton_Click(object sender, RoutedEventArgs e) => ExitSurfaceDesignMode();

    private void ExitSurfaceDesignMode()
    {
        _surfaceDesignMode = false;
        if (_zoneDrawMode != ZoneDrawMode.None) CancelZoneDraw();
        CancelSurfacePickMode();
        ApplyViewerMode();
        if (_activeSource is not null)
            SystemStatus.Text = "SYSTEM: LIVE";
    }

    private void ApplyViewerMode()
    {
        var canDesign = _staticMap is not null && _staticMap.Points.Count > 0;
        EnterDesignModeButton.IsEnabled = canDesign && !_surfaceDesignMode;
        ExitDesignModeButton.IsEnabled = _surfaceDesignMode;

        if (_surfaceDesignMode && canDesign)
        {
            StaticMapModel.Visibility = Visibility.Visible;
            StaticMapModel.IsHitTestVisible = true;
            PointCloudModel.Visibility = Visibility.Collapsed;
            PointCloudModel.IsHitTestVisible = false;
            ViewerModeText.Text = "VIEW: SURFACE DESIGN • STATIC SENSOR MAP";
            ViewerModeText.Foreground = WpfBrushes.LightSkyBlue;
        }
        else
        {
            StaticMapModel.Visibility = Visibility.Collapsed;
            StaticMapModel.IsHitTestVisible = false;
            PointCloudModel.Visibility = Visibility.Visible;
            PointCloudModel.IsHitTestVisible = true;
            ViewerModeText.Text = "VIEW: LIVE POINT CLOUD + SAVED SURFACES/ZONES";
            ViewerModeText.Foreground = WpfBrushes.LightGreen;
        }
        Viewport.InvalidateRender();
    }

    private async void SaveProject_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            var path = Path.GetFullPath("projects/Machine_A.project.json");
            await _projectStore.SaveAsync(_project, path);
            ProjectStatus.Text = "Machine_A (saved)";
        }
        catch (Exception ex)
        {
            WpfMessageBox.Show(ex.Message, "Project save error",
                MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void RefreshZones()
    {
        var selectedSurfaceId = (SurfaceList.SelectedItem as SurfaceDefinition)?.Id;
        var previousId = (ZoneList.SelectedItem as ZoneDefinition)?.Id;
        var zones = string.IsNullOrWhiteSpace(selectedSurfaceId)
            ? _project.Zones.ToList()
            : _project.Zones.Where(z => z.SurfaceId == selectedSurfaceId).ToList();
        ZoneList.ItemsSource = null;
        ZoneList.ItemsSource = zones;
        if (!string.IsNullOrWhiteSpace(previousId))
            ZoneList.SelectedItem = zones.FirstOrDefault(z => z.Id == previousId);
        if (ZoneList.SelectedItem is null && zones.Count > 0)
            ZoneList.SelectedIndex = 0;
        RenderAllZones();
        if (zones.Count == 0)
        {
            SelectedZoneText.Text = "SELECTED ZONE: --";
            ZoneCoordinateText.Text = "--";
            ZoneStatusText.Text = selectedSurfaceId is null ? "Zone: chọn Surface trước" : $"Zone: {selectedSurfaceId} chưa có Zone";
            ClearZoneRender();
        }
    }

    private void ZoneList_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
    {
        if (ZoneList.SelectedItem is not ZoneDefinition zone)
        {
            SelectedZoneText.Text = "SELECTED ZONE: --";
            ZoneCoordinateText.Text = "--";
            ClearZoneRender();
            RenderAllZones();
            return;
        }
        SelectedZoneText.Text = $"SELECTED ZONE: {zone.Id} | {zone.Name} | {zone.SurfaceId}";
        LoadZoneFields(zone);
        RenderSelectedZone();
        UpdateZoneCoordinateInfo(zone);
    }

    private void NewZoneButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            ZoneStatusText.Text = "NEW ZONE: vào SURFACE DESIGN trên Static Map trước.";
            return;
        }
        if (SurfaceList.SelectedItem is not SurfaceDefinition surface)
        {
            ZoneStatusText.Text = "NEW ZONE: chọn Surface trước.";
            return;
        }
        if (!string.Equals(surface.Frame, "MID360_SENSOR", StringComparison.Ordinal))
        {
            ZoneStatusText.Text = "NEW ZONE: Surface không thuộc MID360_SENSOR.";
            return;
        }

        var next = 1;
        while (_project.Zones.Any(z => z.Id.Equals($"Z{next:00}", StringComparison.OrdinalIgnoreCase))) next++;
        var w = Math.Max(0.05, Math.Min(0.5, surface.WidthMeters * 0.5));
        var h = Math.Max(0.05, Math.Min(0.5, surface.HeightMeters * 0.5));
        var cu = Math.Max(w / 2.0, surface.WidthMeters / 2.0);
        var cv = Math.Max(h / 2.0, surface.HeightMeters / 2.0);
        var zone = new ZoneDefinition
        {
            Id = $"Z{next:00}", Name = $"Zone {next:00}", SurfaceId = surface.Id,
            Type = ZoneType.Rectangle, Enabled = true
        };
        SetRectangleZone(zone, cu, cv, w, h, surface);
        _project.Zones.Add(zone);
        RefreshZones();
        ZoneList.SelectedItem = zone;
        RenderSelectedZone();
        ZoneStatusText.Text = $"{zone.Id}: tạo trên {surface.Id} bằng UV local. Không phụ thuộc camera/IMU.";
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private async void DeleteZoneButton_Click(object sender, RoutedEventArgs e)
    {
        if (ZoneList.SelectedItem is not ZoneDefinition zone)
        {
            ZoneStatusText.Text = "XÓA ZONE: chọn Zone trước.";
            return;
        }
        if (WpfMessageBox.Show($"Xóa {zone.Id} - {zone.Name}?", "Delete Zone", MessageBoxButton.YesNo, MessageBoxImage.Warning) != MessageBoxResult.Yes)
            return;
        _project.Zones.Remove(zone);
        ClearZoneRender();
        RefreshZones();
        ZoneStatusText.Text = $"{zone.Id}: đã xóa.";
        await _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void ShowZoneButton_Click(object sender, RoutedEventArgs e)
    {
        if (ZoneList.SelectedItem is not ZoneDefinition zone)
        {
            ZoneStatusText.Text = "HIỆN ZONE: chọn Zone trước.";
            return;
        }
        RenderSelectedZone();
        ZoneStatusText.Text = $"{zone.Id}: đang hiển thị trên {zone.SurfaceId}.";
    }

    private void ApplyZoneButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            ZoneStatusText.Text = "APPLY ZONE: chỉ chỉnh trong SURFACE DESIGN.";
            return;
        }
        if (ZoneList.SelectedItem is not ZoneDefinition zone)
        {
            ZoneStatusText.Text = "APPLY ZONE: chọn Zone trước.";
            return;
        }
        var surface = _project.Surfaces.FirstOrDefault(s => s.Id == zone.SurfaceId);
        if (surface is null) { ZoneStatusText.Text = "APPLY ZONE: Surface liên kết không tồn tại."; return; }
        if (!TryReadDouble(ZoneUText.Text, out var cu) || !TryReadDouble(ZoneVText.Text, out var cv) ||
            !TryReadDouble(ZoneWidthText.Text, out var w) || !TryReadDouble(ZoneHeightText.Text, out var h) || w <= 0 || h <= 0)
        {
            ZoneStatusText.Text = "APPLY ZONE: U/V/Width/Height không hợp lệ.";
            return;
        }
        SetRectangleZone(zone, cu, cv, w, h, surface);
        RenderSelectedZone();
        ZoneStatusText.Text = $"{zone.Id}: UV center=({cu:0.###},{cv:0.###}), size={w:0.###}×{h:0.###} m.";
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void ApplyZoneExtrudeButton_Click(object sender, RoutedEventArgs e)
    {
        if (ZoneList.SelectedItem is not ZoneDefinition zone)
        {
            ZoneStatusText.Text = "EXTRUDE: chọn Zone trước.";
            return;
        }
        if (!TryReadDouble(ZoneExtrudeFrontText.Text, out var frontMm) ||
            !TryReadDouble(ZoneExtrudeBackText.Text, out var backMm) ||
            frontMm < 0 || backMm < 0 || frontMm > 2000 || backMm > 2000)
        {
            ZoneStatusText.Text = "EXTRUDE: Front/Back phải trong 0..2000 mm.";
            return;
        }
        zone.ExtrudeFrontMeters = frontMm / 1000.0;
        zone.ExtrudeBackMeters = backMm / 1000.0;
        ZoneStatusText.Text = $"{zone.Id}: VOLUME Front +{frontMm:0} mm / Back -{backMm:0} mm.";
        UpdateZoneCoordinateInfo(zone);
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void ZoneTransformButton_Click(object sender, RoutedEventArgs e)
    {
        if (!_surfaceDesignMode || _staticMap is null || ZoneList.SelectedItem is not ZoneDefinition zone)
            return;
        var surface = _project.Surfaces.FirstOrDefault(s => s.Id == zone.SurfaceId);
        if (surface is null) return;
        if (!TryGetRectangleZone(zone, out var cu, out var cv, out var w, out var h)) return;
        if (sender is not System.Windows.Controls.Button b || b.Tag is not string op) return;
        var step = GetComboStep(ZoneMoveStepCombo, 0.01);
        if (op == "ZU-") cu -= step;
        else if (op == "ZU+") cu += step;
        else if (op == "ZV-") cv -= step;
        else if (op == "ZV+") cv += step;
        else return;
        SetRectangleZone(zone, cu, cv, w, h, surface);
        LoadZoneFields(zone);
        RenderSelectedZone();
        ZoneStatusText.Text = $"{zone.Id}: moved in Surface UV; center=({cu:0.###},{cv:0.###}) m.";
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private static void SetRectangleZone(ZoneDefinition zone, double centerU, double centerV, double width, double height, SurfaceDefinition surface)
    {
        width = Math.Clamp(width, 0.01, Math.Max(0.01, surface.WidthMeters));
        height = Math.Clamp(height, 0.01, Math.Max(0.01, surface.HeightMeters));
        centerU = Math.Clamp(centerU, width / 2.0, Math.Max(width / 2.0, surface.WidthMeters - width / 2.0));
        centerV = Math.Clamp(centerV, height / 2.0, Math.Max(height / 2.0, surface.HeightMeters - height / 2.0));
        var u0 = centerU - width / 2.0; var u1 = centerU + width / 2.0;
        var v0 = centerV - height / 2.0; var v1 = centerV + height / 2.0;
        zone.Type = ZoneType.Rectangle;
        zone.Polygon = new List<UvPoint> { new(u0,v0), new(u1,v0), new(u1,v1), new(u0,v1) };
    }

    private static bool TryGetRectangleZone(ZoneDefinition zone, out double centerU, out double centerV, out double width, out double height)
    {
        centerU = centerV = width = height = 0;
        if (zone.Polygon.Count < 4) return false;
        var minU = zone.Polygon.Min(p => p.U); var maxU = zone.Polygon.Max(p => p.U);
        var minV = zone.Polygon.Min(p => p.V); var maxV = zone.Polygon.Max(p => p.V);
        width = maxU - minU; height = maxV - minV;
        centerU = (minU + maxU) * 0.5; centerV = (minV + maxV) * 0.5;
        return width > 0 && height > 0;
    }

    private void LoadZoneFields(ZoneDefinition zone)
    {
        if (!TryGetRectangleZone(zone, out var cu, out var cv, out var w, out var h)) return;
        var ci = System.Globalization.CultureInfo.InvariantCulture;
        ZoneUText.Text = cu.ToString("0.###", ci);
        ZoneVText.Text = cv.ToString("0.###", ci);
        ZoneWidthText.Text = w.ToString("0.###", ci);
        ZoneHeightText.Text = h.ToString("0.###", ci);
        ZoneExtrudeFrontText.Text = (zone.ExtrudeFrontMeters * 1000.0).ToString("0", ci);
        ZoneExtrudeBackText.Text = (zone.ExtrudeBackMeters * 1000.0).ToString("0", ci);
    }

    private void StartDrawRectZoneButton_Click(object sender, RoutedEventArgs e) => StartZoneDraw(ZoneDrawMode.Rectangle);

    private void StartDrawPolygonZoneButton_Click(object sender, RoutedEventArgs e) => StartZoneDraw(ZoneDrawMode.Polygon);

    private void FinishZoneDrawButton_Click(object sender, RoutedEventArgs e)
    {
        if (_zoneDrawMode == ZoneDrawMode.Polygon)
        {
            if (_zoneDrawUv.Count < 3)
            {
                ZoneStatusText.Text = "DRAW POLYGON: cần ít nhất 3 điểm trên Surface.";
                return;
            }
            CommitDirectDrawZone();
            return;
        }
        if (_zoneDrawMode == ZoneDrawMode.Rectangle)
        {
            ZoneStatusText.Text = "DRAW RECT: click 2 điểm đối diện trên Surface.";
            return;
        }
        ZoneStatusText.Text = "Không có Zone đang vẽ.";
    }

    private void CancelZoneDrawButton_Click(object sender, RoutedEventArgs e) => CancelZoneDraw();

    private void StartZoneDraw(ZoneDrawMode mode)
    {
        if (!_surfaceDesignMode || _staticMap is null)
        {
            ZoneStatusText.Text = "DRAW ZONE: hãy BUILD/LOAD Static Map và vào SURFACE DESIGN trước.";
            return;
        }
        if (SurfaceList.SelectedItem is not SurfaceDefinition surface ||
            !string.Equals(surface.Frame, "MID360_SENSOR", StringComparison.Ordinal))
        {
            ZoneStatusText.Text = "DRAW ZONE: chọn Surface MID360_SENSOR trước.";
            return;
        }
        _zoneDrawMode = mode;
        _zoneDrawUv.Clear();
        _zoneDrawHoverUv = null;
        UpdateZoneDrawPreview();
        ZoneStatusText.Text = mode == ZoneDrawMode.Rectangle
            ? "DRAW RECT: click 2 góc trên Surface (2 mặt đều dùng được). SNAP edge/vertex đang hỗ trợ."
            : "DRAW POLYGON: click các đỉnh trên Surface (2 mặt đều dùng được), sau đó FINISH. SNAP edge/vertex đang hỗ trợ.";
    }

    private void CancelZoneDraw()
    {
        _zoneDrawMode = ZoneDrawMode.None;
        _zoneDrawUv.Clear();
        _zoneDrawHoverUv = null;
        ZoneDrawPreviewModel.Geometry = null;
        ZoneDrawVertexModel.Geometry = null;
        if (ZoneStatusText is not null)
            ZoneStatusText.Text = "Zone draw: canceled.";
    }

    private bool TryGetSurfaceUvFromViewer(SurfaceDefinition surface, System.Windows.Point mouse,
        out UvPoint uv, out Point3D sensorPoint)
    {
        uv = default;
        sensorPoint = default;
        if (!string.Equals(surface.Frame, "MID360_SENSOR", StringComparison.Ordinal))
            return false;

        // Build the camera ray in DISPLAY coordinates, then inverse-transform the
        // complete ray into MID360_SENSOR. Surface intersection therefore happens
        // entirely in Sensor coordinates and is explicitly TWO-SIDED.
        if (!TryGetSensorRayFromViewer(mouse, out var rayOriginSensor, out var rayDirectionSensor))
            return false;
        return SurfaceGeometry.TryIntersectRay(surface, rayOriginSensor, rayDirectionSensor, out sensorPoint, out uv);
    }

    private bool TryGetSensorRayFromViewer(System.Windows.Point mouse, out Point3D originSensor, out Point3D directionSensor)
    {
        originSensor = default; directionSensor = default;
        if (Viewport.ActualWidth < 2 || Viewport.ActualHeight < 2) return false;

        MediaPoint3D position;
        MediaVector3D look, up;
        double halfWidth, halfHeight;
        bool perspective;
        double fieldOfViewDeg = 45.0;
        if (Viewport.Camera is PerspectiveCamera pc)
        {
            position = pc.Position; look = pc.LookDirection; up = pc.UpDirection;
            perspective = true; fieldOfViewDeg = pc.FieldOfView;
            halfWidth = halfHeight = 0;
        }
        else if (Viewport.Camera is OrthographicCamera oc)
        {
            position = oc.Position; look = oc.LookDirection; up = oc.UpDirection;
            perspective = false;
            halfWidth = Math.Max(1e-6, oc.Width * 0.5);
            halfHeight = halfWidth * Viewport.ActualHeight / Math.Max(1.0, Viewport.ActualWidth);
        }
        else return false;

        if (look.LengthSquared < 1e-12 || up.LengthSquared < 1e-12) return false;
        look.Normalize(); up.Normalize();
        var right = MediaVector3D.CrossProduct(look, up);
        if (right.LengthSquared < 1e-12) return false;
        right.Normalize();
        // Rebuild an orthogonal up axis to prevent camera numerical drift.
        up = MediaVector3D.CrossProduct(right, look); up.Normalize();

        var nx = 2.0 * mouse.X / Viewport.ActualWidth - 1.0;
        var ny = 1.0 - 2.0 * mouse.Y / Viewport.ActualHeight;
        MediaPoint3D roDisplay;
        MediaVector3D rdDisplay;
        if (perspective)
        {
            var tanY = Math.Tan(fieldOfViewDeg * Math.PI / 360.0);
            var aspect = Viewport.ActualWidth / Math.Max(1.0, Viewport.ActualHeight);
            rdDisplay = look + right * (nx * aspect * tanY) + up * (ny * tanY);
            rdDisplay.Normalize();
            roDisplay = position;
        }
        else
        {
            roDisplay = position + right * (nx * halfWidth) + up * (ny * halfHeight);
            rdDisplay = look;
        }

        var ro = _coordinateEngine.ToSensor(new Point3D((float)roDisplay.X, (float)roDisplay.Y, (float)roDisplay.Z, 0, 0));
        var p1Display = roDisplay + rdDisplay;
        var p1 = _coordinateEngine.ToSensor(new Point3D((float)p1Display.X, (float)p1Display.Y, (float)p1Display.Z, 0, 0));
        var d = new Vector3(p1.X - ro.X, p1.Y - ro.Y, p1.Z - ro.Z);
        if (d.LengthSquared() < 1e-12f) return false;
        d = Vector3.Normalize(d);
        originSensor = ro;
        directionSensor = new Point3D(d.X, d.Y, d.Z, 0, 0);
        return true;
    }

    private UvPoint ApplyZoneSnap(SurfaceDefinition surface, UvPoint raw, out string label)
    {
        label = "FREE";
        if (ZoneSnapCheck?.IsChecked != true) return raw;
        var tol = GetComboStep(ZoneSnapDistanceCombo, 0.02);
        var best = raw; var bestDist = double.PositiveInfinity; var bestLabel = "FREE";

        void Consider(UvPoint q, string qLabel)
        {
            var du = q.U - raw.U; var dv = q.V - raw.V;
            var d = Math.Sqrt(du * du + dv * dv);
            if (d <= tol && d < bestDist) { best = q; bestDist = d; bestLabel = qLabel; }
        }

        // Surface corners and vertices of existing zones are the strongest snap targets.
        Consider(new UvPoint(0, 0), "CORNER");
        Consider(new UvPoint(surface.WidthMeters, 0), "CORNER");
        Consider(new UvPoint(surface.WidthMeters, surface.HeightMeters), "CORNER");
        Consider(new UvPoint(0, surface.HeightMeters), "CORNER");
        foreach (var zone in _project.Zones.Where(z => z.SurfaceId == surface.Id))
            foreach (var p in zone.Polygon) Consider(p, $"{zone.Id} VERTEX");
        foreach (var p in _zoneDrawUv) Consider(p, "CURRENT VERTEX");

        // Edge snap is independent in U/V and is useful for rectangles/polygons.
        if (bestDist == double.PositiveInfinity)
        {
            if (Math.Abs(raw.U) <= tol) { best = new UvPoint(0, raw.V); bestLabel = "EDGE U=0"; }
            else if (Math.Abs(raw.U - surface.WidthMeters) <= tol) { best = new UvPoint(surface.WidthMeters, raw.V); bestLabel = "EDGE U=MAX"; }
            else if (Math.Abs(raw.V) <= tol) { best = new UvPoint(raw.U, 0); bestLabel = "EDGE V=0"; }
            else if (Math.Abs(raw.V - surface.HeightMeters) <= tol) { best = new UvPoint(raw.U, surface.HeightMeters); bestLabel = "EDGE V=MAX"; }
        }
        label = bestLabel;
        return best;
    }

    private void UpdateZoneDrawPreview()
    {
        if (_zoneDrawMode == ZoneDrawMode.None || SurfaceList.SelectedItem is not SurfaceDefinition surface)
        {
            ZoneDrawPreviewModel.Geometry = null;
            ZoneDrawVertexModel.Geometry = null;
            return;
        }
        var points = new List<UvPoint>(_zoneDrawUv);
        if (_zoneDrawHoverUv.HasValue) points.Add(_zoneDrawHoverUv.Value);
        if (points.Count == 0) { ZoneDrawPreviewModel.Geometry = null; ZoneDrawVertexModel.Geometry = null; return; }

        List<UvPoint> shape; bool close;
        if (_zoneDrawMode == ZoneDrawMode.Rectangle && points.Count >= 2)
        {
            var a=points[0]; var b=points[^1];
            shape=new List<UvPoint>{new(Math.Min(a.U,b.U),Math.Min(a.V,b.V)),new(Math.Max(a.U,b.U),Math.Min(a.V,b.V)),new(Math.Max(a.U,b.U),Math.Max(a.V,b.V)),new(Math.Min(a.U,b.U),Math.Max(a.V,b.V))};
            close=true;
        }
        else { shape=points; close=false; }

        const float lift=0.003f;
        // TWO-SIDED preview: duplicate handles + outline at +N and -N.
        // This is display-only; saved Zone UV remains exactly on the Sensor Surface.
        var vertexDisplay=new Vector3Collection();
        foreach(var side in new[]{1f,-1f})
        foreach(var uv in shape)
        {
            var p0=SurfaceGeometry.FromUv(surface,uv.U,uv.V); var np=SurfaceGeometry.NormalAtUv(surface,uv.U,uv.V);
            var sn=Vector3.Normalize(new Vector3(np.X,np.Y,np.Z)); var sp=new Vector3(p0.X,p0.Y,p0.Z)+sn*(lift*side);
            vertexDisplay.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0)));
        }
        ZoneDrawVertexModel.Geometry=new PointGeometry3D{Positions=vertexDisplay};
        if(shape.Count>=2)
        {
            var front=BuildZonePolylineDisplay(surface,shape,close,+lift);
            var back =BuildZonePolylineDisplay(surface,shape,close,-lift);
            var line=new Vector3Collection(); var idx=new IntCollection();
            void AddSide(Vector3Collection sideLine)
            {
                var baseIndex=line.Count; foreach(var q in sideLine) line.Add(q);
                for(int i=0;i+1<sideLine.Count;i++){idx.Add(baseIndex+i);idx.Add(baseIndex+i+1);}
                if(close&&sideLine.Count>2){idx.Add(baseIndex+sideLine.Count-1);idx.Add(baseIndex);}
            }
            AddSide(front); AddSide(back);
            ZoneDrawPreviewModel.Geometry=new LineGeometry3D{Positions=line,Indices=idx};
        }
        else ZoneDrawPreviewModel.Geometry=null;
    }

    private void CommitDirectDrawZone()
    {
        if (SurfaceList.SelectedItem is not SurfaceDefinition surface) { CancelZoneDraw(); return; }
        List<UvPoint> polygon;
        ZoneType type;
        if (_zoneDrawMode == ZoneDrawMode.Rectangle)
        {
            if (_zoneDrawUv.Count < 2) return;
            var a = _zoneDrawUv[0]; var b = _zoneDrawUv[1];
            var u0 = Math.Min(a.U,b.U); var u1 = Math.Max(a.U,b.U);
            var v0 = Math.Min(a.V,b.V); var v1 = Math.Max(a.V,b.V);
            if (u1-u0 < 0.005 || v1-v0 < 0.005)
            {
                ZoneStatusText.Text = "DRAW RECT: kích thước quá nhỏ (<5 mm).";
                _zoneDrawUv.Clear(); UpdateZoneDrawPreview(); return;
            }
            polygon = new List<UvPoint> { new(u0,v0), new(u1,v0), new(u1,v1), new(u0,v1) };
            type = ZoneType.Rectangle;
        }
        else
        {
            if (_zoneDrawUv.Count < 3) return;
            polygon = _zoneDrawUv.ToList();
            type = ZoneType.Polygon;
        }

        var maxRoundTrip = polygon.Max(p => SurfaceGeometry.RoundTripErrorMeters(surface, p));
        if (maxRoundTrip > 0.0005)
        {
            ZoneStatusText.Text = $"ZONE REJECTED: Sensor↔UV round-trip error {maxRoundTrip*1000.0:0.###} mm.";
            return;
        }
        if (polygon.Any(p => !SurfaceGeometry.IsUvInsideSurface(surface, p.U, p.V, 0.001)))
        {
            ZoneStatusText.Text = "ZONE REJECTED: có vertex nằm ngoài Surface.";
            return;
        }

        var next = 1;
        while (_project.Zones.Any(z => z.Id.Equals($"Z{next:00}", StringComparison.OrdinalIgnoreCase))) next++;
        var zone = new ZoneDefinition
        {
            Id = $"Z{next:00}", Name = $"Zone {next:00}", SurfaceId = surface.Id,
            Type = type, Enabled = true, Polygon = polygon
        };
        _project.Zones.Add(zone);
        _zoneDrawMode = ZoneDrawMode.None;
        _zoneDrawUv.Clear(); _zoneDrawHoverUv = null;
        ZoneDrawPreviewModel.Geometry = null; ZoneDrawVertexModel.Geometry = null;
        RefreshZones();
        ZoneList.SelectedItem = zone;
        RenderSelectedZone();
        ZoneStatusText.Text = $"{zone.Id}: saved as Surface UV on {surface.Id} | max Sensor↔UV error={maxRoundTrip*1000.0:0.###} mm.";
        _ = _projectStore.SaveAsync(_project, Path.GetFullPath("projects/Machine_A.project.json"));
    }

    private void RenderAllZones()
    {
        var linePos=new Vector3Collection(); var lineIdx=new IntCollection();
        var facePos=new Vector3Collection(); var faceNorm=new Vector3Collection(); var faceIdx=new IntCollection();
        const float lift=0.0015f;
        var visibleZones = _project.Zones.Where(z=>z.Enabled && z.Polygon.Count>=3).ToArray();
        var surfaceById = _project.Surfaces.ToDictionary(s=>s.Id,StringComparer.OrdinalIgnoreCase);
        // Scale overview rendering with Zone count. Selected Zone still keeps full detail.
        // Transparent meshes and double-sided curved outlines are expensive in SharpDX.
        var drawOverviewFaces = visibleZones.Length <= 8;
        var drawBothSides = visibleZones.Length <= 12;
        var overviewTargetSegment = visibleZones.Length > 24 ? 0.12 : visibleZones.Length > 12 ? 0.09 : 0.06;
        var overviewMaxSegments = visibleZones.Length > 24 ? 12 : visibleZones.Length > 12 ? 18 : 32;
        foreach(var zone in visibleZones)
        {
            surfaceById.TryGetValue(zone.SurfaceId,out var surface);
            if(surface is null || !string.Equals(surface.Frame,"MID360_SENSOR",StringComparison.Ordinal)) continue;
            if(surface.CurvedKind==CurvedSurfaceKind.OcctBSpline && !SurfaceGeometry.IsOcctAvailable(out _)) continue;
            var sideCount = drawBothSides ? 2 : 1;
            for (int sideIndex = 0; sideIndex < sideCount; sideIndex++)
            {
                var side = sideIndex == 0 ? 1f : -1f;
                var pl=BuildZonePolylineDisplay(surface,zone.Polygon,true,lift*side,overviewTargetSegment,overviewMaxSegments); var b=linePos.Count; foreach(var q in pl)linePos.Add(q);
                for(int i=0;i+1<pl.Count;i++){lineIdx.Add(b+i);lineIdx.Add(b+i+1);} if(pl.Count>2){lineIdx.Add(b+pl.Count-1);lineIdx.Add(b);}
            }
            if (!drawOverviewFaces) continue;
            var tri=TriangulateZoneUv(zone.Polygon); if(tri.Count==0) continue;
            var baseVertex=facePos.Count;
            foreach(var uv in zone.Polygon)
            {
                var p0=SurfaceGeometry.FromUv(surface,uv.U,uv.V);var np=SurfaceGeometry.NormalAtUv(surface,uv.U,uv.V);var sn=Vector3.Normalize(new Vector3(np.X,np.Y,np.Z));
                var sp=new Vector3(p0.X,p0.Y,p0.Z)+sn*lift;facePos.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0)));faceNorm.Add(SensorDirectionToDisplay(sn));
            }
            foreach(var ti in tri)faceIdx.Add(baseVertex+ti);
        }
        AllZoneOutlineModel.Geometry=linePos.Count==0?null:new LineGeometry3D{Positions=linePos,Indices=lineIdx};
        AllZoneFaceModel.Geometry=facePos.Count==0?null:new MeshGeometry3D{Positions=facePos,Normals=faceNorm,Indices=faceIdx};
    }

    private void UpdateZoneCoordinateInfo(ZoneDefinition zone)
    {
        var surface=_project.Surfaces.FirstOrDefault(s=>s.Id==zone.SurfaceId);
        if(surface is null||zone.Polygon.Count<3){ZoneCoordinateText.Text="--";return;}
        var lines=new List<string>(); lines.Add($"Frame   MID360_SENSOR | Surface {surface.Id} | {surface.Type}");
        double maxErr=0;
        for(int i=0;i<zone.Polygon.Count;i++)
        {
            var uv=zone.Polygon[i]; var p=SurfaceGeometry.FromUv(surface,uv.U,uv.V); maxErr=Math.Max(maxErr,SurfaceGeometry.RoundTripErrorMeters(surface,uv));
            lines.Add($"P{i+1} UV({uv.U:0.###},{uv.V:0.###}) Xs {p.X:+0.000;-0.000;0.000} Ys {p.Y:+0.000;-0.000;0.000} Zs {p.Z:+0.000;-0.000;0.000}");
        }
        var cu=zone.Polygon.Average(q=>q.U); var cv=zone.Polygon.Average(q=>q.V); var cp=SurfaceGeometry.FromUv(surface,cu,cv);
        var range=Math.Sqrt(cp.X*cp.X+cp.Y*cp.Y+cp.Z*cp.Z);
        var perimeter=0.0;
        for(int i=0;i<zone.Polygon.Count;i++){var a=zone.Polygon[i];var b=zone.Polygon[(i+1)%zone.Polygon.Count];perimeter+=SurfaceGeometry.ApproxSurfacePathLength(surface,a,b,surface.Type==SurfaceType.Curved?32:1);}
        lines.Add($"Center UV({cu:0.###},{cv:0.###}) → Xs {cp.X:+0.000;-0.000;0.000} Ys {cp.Y:+0.000;-0.000;0.000} Zs {cp.Z:+0.000;-0.000;0.000} | R {range:0.000} m");
        if(zone.Type==ZoneType.Rectangle&&TryGetRectangleZone(zone,out var rcu,out var rcv,out var w,out var h))
        {
            var left=new UvPoint(rcu-w/2,rcv); var right=new UvPoint(rcu+w/2,rcv); var bottom=new UvPoint(rcu,rcv-h/2); var top=new UvPoint(rcu,rcv+h/2);
            var actualW=SurfaceGeometry.ApproxSurfacePathLength(surface,left,right,surface.Type==SurfaceType.Curved?48:1);
            var actualH=SurfaceGeometry.ApproxSurfacePathLength(surface,bottom,top,surface.Type==SurfaceType.Curved?48:1);
            lines.Add($"Actual surface size ≈ {actualW:0.000} × {actualH:0.000} m | perimeter {perimeter:0.000} m");
        }
        else lines.Add($"Vertices {zone.Polygon.Count} | actual surface perimeter ≈ {perimeter:0.000} m");
        lines.Add($"Zone Volume: local normal -{_project.Detection.ZoneBackMm:0} / +{_project.Detection.ZoneFrontMm:0} mm");
        lines.Add($"Sensor↔UV max error {maxErr*1000.0:0.###} mm {(maxErr<=0.0005?"PASS":"CHECK")}");
        ZoneCoordinateText.Text=string.Join("\n",lines);
    }

    private void RenderSelectedZoneVolume(SurfaceDefinition surface, ZoneDefinition zone)
    {
        // Rendered Zone volume uses the same SurfaceGeometry as detection.
        // Curved surfaces are tessellated in UV and offset along the local normal.
        if (zone.Polygon.Count < 3) { ZoneVolumeModel.Geometry=null; ZoneVolumeOutlineModel.Geometry=null; return; }
        double front = Math.Max(0.001, _project.Detection.ZoneFrontMm / 1000.0);
        double back = Math.Max(0.001, _project.Detection.ZoneBackMm / 1000.0);
        var pos=new Vector3Collection(); var norms=new Vector3Collection(); var idx=new IntCollection();

        Point3D Offset(UvPoint uv,double d)
        {
            var p0=SurfaceGeometry.FromUv(surface,uv.U,uv.V);
            var nn=SurfaceGeometry.NormalAtUv(surface,uv.U,uv.V);
            var v=Vector3.Normalize(new Vector3(nn.X,nn.Y,nn.Z));
            return new Point3D((float)(p0.X+v.X*d),(float)(p0.Y+v.Y*d),(float)(p0.Z+v.Z*d),0,0);
        }
        UvPoint Mid(UvPoint x,UvPoint y)=>new((x.U+y.U)*0.5,(x.V+y.V)*0.5);
        void AddMeshTri(UvPoint a,UvPoint b,UvPoint c,double d,bool flip)
        {
            int k=pos.Count;
            foreach(var uv in new[]{a,b,c})
            {
                var pp=Offset(uv,d); pos.Add(SensorPointToDisplay(pp));
                var nn=SurfaceGeometry.NormalAtUv(surface,uv.U,uv.V);
                var dn=SensorDirectionToDisplay(Vector3.Normalize(new Vector3(nn.X,nn.Y,nn.Z)))*(flip?-1f:1f);
                norms.Add(dn);
            }
            if(!flip){idx.Add(k);idx.Add(k+1);idx.Add(k+2);} else {idx.Add(k+2);idx.Add(k+1);idx.Add(k);}
        }
        void SubTri(UvPoint a,UvPoint b,UvPoint c,double d,bool flip,int depth)
        {
            if(depth<=0){AddMeshTri(a,b,c,d,flip);return;}
            var ab=Mid(a,b); var bc=Mid(b,c); var ca=Mid(c,a);
            SubTri(a,ab,ca,d,flip,depth-1); SubTri(ab,b,bc,d,flip,depth-1);
            SubTri(ca,bc,c,d,flip,depth-1); SubTri(ab,bc,ca,d,flip,depth-1);
        }

        // Triangulate the Zone in UV, then subdivide each triangle. Depth 3 = 64 curved
        // micro-triangles per original triangle: smooth enough for Cylinder/OCCT interaction UI.
        var tri=TriangulateZoneUv(zone.Polygon);
        int depth=surface.Type==SurfaceType.Plane?0:3;
        for(int i=0;i+2<tri.Count;i+=3)
        {
            var a0=zone.Polygon[tri[i]]; var b0=zone.Polygon[tri[i+1]]; var c0=zone.Polygon[tri[i+2]];
            SubTri(a0,b0,c0,+front,false,depth);
            SubTri(a0,b0,c0,-back,true,depth);
        }

        // Curved side walls: sample each polygon edge in UV and connect corresponding
        // front/back samples. This closes the volume even on strongly curved OCCT surfaces.
        for(int e=0;e<zone.Polygon.Count;e++)
        {
            var a0=zone.Polygon[e]; var b0=zone.Polygon[(e+1)%zone.Polygon.Count];
            var edgeLen=SurfaceGeometry.ApproxSurfacePathLength(surface,a0,b0,surface.Type==SurfaceType.Plane?1:48);
            int seg=surface.Type==SurfaceType.Plane?1:Math.Clamp((int)Math.Ceiling(edgeLen/0.025),2,64);
            for(int j=0;j<seg;j++)
            {
                double t0=j/(double)seg,t1=(j+1)/(double)seg;
                var ua=new UvPoint(a0.U+(b0.U-a0.U)*t0,a0.V+(b0.V-a0.V)*t0);
                var ub=new UvPoint(a0.U+(b0.U-a0.U)*t1,a0.V+(b0.V-a0.V)*t1);
                var fa=Offset(ua,+front); var fb=Offset(ub,+front); var ba=Offset(ua,-back); var bb=Offset(ub,-back);
                int k=pos.Count;
                foreach(var pp in new[]{fa,fb,bb,ba}){pos.Add(SensorPointToDisplay(pp));norms.Add(new Vector3(0,0,1));}
                idx.Add(k);idx.Add(k+1);idx.Add(k+2); idx.Add(k);idx.Add(k+2);idx.Add(k+3);
            }
        }
        ZoneVolumeModel.Geometry=new MeshGeometry3D{Positions=pos,Normals=norms,Indices=idx};

        var lp=new Vector3Collection(); var li=new IntCollection();
        void Edge(Point3D a,Point3D b){var k=lp.Count;lp.Add(SensorPointToDisplay(a));lp.Add(SensorPointToDisplay(b));li.Add(k);li.Add(k+1);}
        for(int e=0;e<zone.Polygon.Count;e++)
        {
            var a0=zone.Polygon[e]; var b0=zone.Polygon[(e+1)%zone.Polygon.Count];
            var edgeLen=SurfaceGeometry.ApproxSurfacePathLength(surface,a0,b0,surface.Type==SurfaceType.Plane?1:48);
            int seg=surface.Type==SurfaceType.Plane?1:Math.Clamp((int)Math.Ceiling(edgeLen/0.025),2,64);
            for(int j=0;j<seg;j++)
            {
                double t0=j/(double)seg,t1=(j+1)/(double)seg;
                var ua=new UvPoint(a0.U+(b0.U-a0.U)*t0,a0.V+(b0.V-a0.V)*t0);
                var ub=new UvPoint(a0.U+(b0.U-a0.U)*t1,a0.V+(b0.V-a0.V)*t1);
                Edge(Offset(ua,+front),Offset(ub,+front)); Edge(Offset(ua,-back),Offset(ub,-back));
                if(j==0) Edge(Offset(ua,+front),Offset(ua,-back));
            }
        }
        ZoneVolumeOutlineModel.Geometry=new LineGeometry3D{Positions=lp,Indices=li};
    }

    private void RenderSelectedZone()
    {
        RenderAllZones();
        if(ZoneList.SelectedItem is not ZoneDefinition zone){ClearZoneRender();return;}
        var surface=_project.Surfaces.FirstOrDefault(s=>s.Id==zone.SurfaceId);
        if(surface is null||zone.Polygon.Count<3){ClearZoneRender();return;}
        RenderSelectedZoneVolume(surface, zone);
        const float lift=0.003f;
        // Selected Zone is rendered on BOTH sides of the Surface. Geometry data
        // itself stays single-copy UV in MID360_SENSOR; only the display overlay is doubled.
        var frontLine=BuildZonePolylineDisplay(surface,zone.Polygon,true,+lift);
        var backLine =BuildZonePolylineDisplay(surface,zone.Polygon,true,-lift);
        var line=new Vector3Collection(); var lineIdx=new IntCollection();
        void AddOutlineSide(Vector3Collection sideLine)
        {
            var baseIndex=line.Count; foreach(var q in sideLine) line.Add(q);
            for(int i=0;i+1<sideLine.Count;i++){lineIdx.Add(baseIndex+i);lineIdx.Add(baseIndex+i+1);}
            if(sideLine.Count>2){lineIdx.Add(baseIndex+sideLine.Count-1);lineIdx.Add(baseIndex);}
        }
        AddOutlineSide(frontLine); AddOutlineSide(backLine);
        ZoneOutlineModel.Geometry=new LineGeometry3D{Positions=line,Indices=lineIdx};

        var vertices=new Vector3Collection();
        foreach(var side in new[]{1f,-1f})
        foreach(var uv in zone.Polygon)
        {
            var p0=SurfaceGeometry.FromUv(surface,uv.U,uv.V); var np=SurfaceGeometry.NormalAtUv(surface,uv.U,uv.V);
            var sn=Vector3.Normalize(new Vector3(np.X,np.Y,np.Z)); var sp=new Vector3(p0.X,p0.Y,p0.Z)+sn*(lift*side);
            vertices.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0)));
        }
        ZoneVertexModel.Geometry=new PointGeometry3D{Positions=vertices};

        // Keep one translucent fill mesh. Because Surface itself is now highly
        // transparent and CullMode=None, the two-sided outlines/handles remain
        // readable from either side without duplicating detection geometry.
        if(surface.Type==SurfaceType.Curved) ZoneFaceModel.Geometry=BuildCurvedZoneFace(surface,zone,lift);
        else
        {
            var faceVertices=new Vector3Collection(); var norms=new Vector3Collection();
            var np=SurfaceGeometry.NormalAtUv(surface,0,0); var dn=SensorDirectionToDisplay(Vector3.Normalize(new Vector3(np.X,np.Y,np.Z)));
            foreach(var uv in zone.Polygon)
            {
                var p0=SurfaceGeometry.FromUv(surface,uv.U,uv.V); var sn=Vector3.Normalize(new Vector3(np.X,np.Y,np.Z));
                var sp=new Vector3(p0.X,p0.Y,p0.Z)+sn*lift; faceVertices.Add(SensorPointToDisplay(new Point3D(sp.X,sp.Y,sp.Z,0,0))); norms.Add(dn);
            }
            var tri=TriangulateZoneUv(zone.Polygon); var idx=new IntCollection(); foreach(var ti in tri)idx.Add(ti);
            ZoneFaceModel.Geometry=tri.Count==0?null:new MeshGeometry3D{Positions=faceVertices,Normals=norms,Indices=idx};
        }
        UpdateZoneCoordinateInfo(zone);
    }

    private void ClearZoneRender()
    {
        ZoneOutlineModel.Geometry = null;
        ZoneFaceModel.Geometry = null;
        ZoneVertexModel.Geometry = null;
        ZoneVolumeModel.Geometry = null;
        ZoneVolumeOutlineModel.Geometry = null;
    }

}
