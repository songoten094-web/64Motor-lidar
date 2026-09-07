# Livox + Wave Motion HMI v1.0 — Single Application

Một HMI duy nhất kết hợp:

- Livox MID-360 static-touch v3.4.4 (5 frame confirm / 5 frame clear)
- WaveMotionControlv2 nguyên kiến trúc RS485/Modbus/EM2RS/16PR
- Liên kết nội bộ cùng process: `Zone ACTIVE ↑ -> motor Zone FAST -> 30 s -> AUTO normal`

## Nguyên tắc tích hợp

Không có UDP/COM trung gian giữa hai HMI. Không có HMI thứ hai.

WaveMotion vẫn đi theo đường gốc:

`WaveMotion UI -> IRs485Service -> Em2RsModbusService -> line locks/polling -> Modbus RTU -> EM2RS -> 16PR`

Phần tích hợp **không tự ghi register Modbus**. Nó chỉ gọi API mới `TriggerZoneFastAsync`; implementation mới tái sử dụng các hàm 16PR, QuickStop, Trigger, polling pause/resume và line-lock hiện có của `Em2RsModbusService`.

## Hiệu ứng Zone mới

- Livox Zone phải ACTIVE sau 5 frame liên tiếp.
- Chỉ sự kiện `Free -> Active` được gửi sang motor.
- Mỗi Zone được map tới một `ClusterId + WaveZoneColumn`.
- Tất cả motor trong cột/Zone đó được đổi bảng 16PR sang tốc độ `2.0X` mặc định.
- Chạy FAST 30 giây.
- Hết 30 giây: nạp lại bảng 16PR tốc độ AUTO gốc và START lại.
- Giữ tay trong Zone: không retrigger.
- Cùng Zone trigger lại khi đang trong cửa sổ FAST: bỏ qua, không reset timer.
- Zone khác có thể trigger độc lập; chỉ bước reprogram ngắn được serialize để tránh tranh chấp truyền thông.

## Mapping Zone -> motor

Sửa:

`integration/zone_motor_map.json`

Ví dụ:

```json
{ "LivoxZoneId": "Z03", "ClusterId": 1, "WaveZoneColumn": 3 }
```

`WaveZoneColumn` là 1-based trong file cấu hình. Nội bộ WaveMotion vẫn dùng zero-based như code gốc.

## Giao diện

MainWindow có 2 tab trong **cùng một process**:

1. `LIDAR / ZONE` — viewer MID-360, static map, Surface, Zone, touch.
2. `WAVE MOTION / MOTOR` — UI WinForms gốc của WaveMotionControlv2 được host trực tiếp trong WPF.

WaveMotion không bị viết lại sang WPF; do đó các trang MAIN/AUTO/MANUAL/SETTING/STATUS và logic hiện tại được giữ nguyên.

## Build

Yêu cầu Visual Studio 2022 + .NET 9 SDK + Desktop development with C++.

Trước tiên build bridge Livox:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\build_native.ps1
```

Sau đó mở:

`LivoxWaveHmi.sln`

và Rebuild Solution.

App chính:

`src/LivoxHmi.App/LivoxHmi.App.csproj`

WaveMotion được build thành library và chạy bên trong app chính, vì vậy chỉ có một HMI chính.

## Những file WaveMotion liên quan yêu cầu mới

Thay đổi tối thiểu:

- `IRs485Service.cs`: thêm đúng 1 API `TriggerZoneFastAsync`.
- `Em2RsModbusService.cs`: chỉ thêm keyword `partial`; toàn bộ code truyền tin cũ giữ nguyên.
- `Em2RsModbusService.ZoneFast.cs`: logic FAST 30 s mới, dùng lại các primitive truyền thông cũ.
- `DemoRs485Service.cs`: chỉ thêm keyword `partial`.
- `DemoRs485Service.ZoneFast.cs`: implementation demo.

Các register, FC03/FC06/FC10, COM mapping, slave ID, polling, semaphore/line lock, HOME, Manual và logic 16PR gốc không bị thay bằng cơ chế khác.
