# Single HMI build fix

WaveMotionControl is intentionally built as a Library in the combined HMI. The original standalone `Program.cs` is excluded from compilation so that the WPF host remains the only application entry point.

No RS485/Modbus/16PR communication code was changed.
