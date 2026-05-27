# STM32F446 Autonomous Mapping Robot

This repository contains the STM32 firmware and host-side tools for a small
differential-drive robot that can be manually driven over Bluetooth, build a
live occupancy grid from LIDAR data, stream the map to a PC, and run simple
grid-based navigation between configured maze cells.

## Current Features

- FreeRTOS-based STM32F446 application.
- RPLIDAR-style serial parsing on `USART1` with DMA buffering.
- Bluetooth command/control on `USART6` at `921600` baud.
- Differential motor PWM and encoder feedback through the `motor_control`
  module.
- MPU6500 gyro polling and heading integration.
- ADC-based speed adjustment for drive and turn PWM.
- OLED debug pages for LIDAR, MPU, ADC/PWM, and encoder state.
- Occupancy grid mapping with map row streaming:
  - `?` unknown
  - `.` free
  - `#` occupied
- Scan buffering, pose history lookup, scan matching, and mapping diagnostics.
- Navigation over a 5 x 5 maze-cell abstraction with start/goal commands.
- Front obstacle detection and dynamic replanning.
- Windows Qt map viewer and smaller serial debug scripts.

## Repository Layout

```text
Core/Inc/
  bluetooth_control.h    Bluetooth command API
  lidar_pipeline.h       LIDAR parser/service API
  mapping_grid.h         Occupancy grid API
  motor_control.h        Motor and encoder API
  mpu6500.h              MPU6500 API

Core/Src/
  main.c                 Application state machine, mapping, navigation, UI
  bluetooth_control.c    USART6 line parser and command queue
  lidar_pipeline.c       USART1 DMA receive path and LIDAR point queue
  mapping_grid.c         Occupancy grid insertion/scoring/statistics
  motor_control.c        PWM drive, turn, stop, encoder state
  mpu6500.c              Gyro/accelerometer driver

test_win_qt.py           Windows Qt map/navigation viewer
test.py                  Tkinter map viewer
test_mac.py              macOS serial/map viewer
pose_debug.py            Odom/heading debug console
lidar_debug_viewer.py    LIDAR debug helper
```

## Firmware Commands

Commands are sent over Bluetooth serial. Numeric commands are accepted without a
line ending for `0` to `3`; other commands should normally end with `\r\n`.

| Command | Meaning |
| --- | --- |
| `0` | Stop current manual motion |
| `1` | Drive forward |
| `2` | Turn left |
| `3` | Turn right |
| `91` | Start mapping |
| `94` | Start odometry debug output |
| `95` | Stop odometry debug output |
| `98` | Start navigation |
| `99` | Stop navigation |
| `Sxy` | Set navigation start cell, for example `S00` |
| `Gxy` | Set navigation goal cell, for example `G44` |
| `L90` | Turn left by 90 degrees |
| `R90` | Turn right by 90 degrees |
| `SHOW` | Request a full map stream |
| `CLEAR` | Clear the current scan/map data |
| `MAP STOP` | Stop mapping/navigation state |

Common firmware telemetry:

- `MAP START/STOP/CLEAR ...` map header and map revision.
- `MAP ROW y=<row> rev=<rev> data=<cells>` one grid row.
- `MAP STAT ...` occupancy and pose summary.
- `MAP DIAG ...` scan processing counters.
- `MAP PERF ...` scan matching and insertion details.
- `POSE x,y,heading state=<motion>` current pose in millimeters and centidegrees.
- `NAV STAT ...` navigation target, state, and front obstacle status.
- `ODOM ...` encoder and gyro debug data.

## Host Tools

Windows Qt viewer:

```powershell
py -m pip install PySide6 pyqtgraph numpy pyserial
py test_win_qt.py --port COM6
```

Odometry debug console:

```powershell
py -m pip install pyserial
py pose_debug.py
```

Older Tkinter viewers are still present for quick serial checks:

```bash
python3 test.py --port COM6
python3 test_mac.py --port /dev/cu.ccc
```

## Build

The project is generated for STM32CubeIDE. On this Mac, a command-line build can
be run from `Debug/` with the CubeIDE ARM toolchain on `PATH`:

```bash
cd Debug
PATH="/Applications/STMicroelectronics/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macosaarch64_1.0.0.202602081740/tools/bin:$PATH" make all -j4
```

The output firmware is `Debug/final_project.elf`.

## Navigation Notes

- The maze abstraction is 5 x 5 cells.
- Each maze cell is currently `700 mm`.
- The default logical start is cell `0,0`; the default goal is `4,4`.
- The robot pose is maintained in millimeters and heading centidegrees.
- Navigation uses the live pose for replanning rather than snapping the robot to
  a waypoint center.
- Manual and navigation speeds remain ADC-controlled, with autonomous movement
  capped in firmware for safer testing.

## Practical Test Flow

1. Flash the firmware from STM32CubeIDE.
2. Pair the Bluetooth serial module with the PC.
3. Open the viewer, usually `test_win_qt.py --port COM6`.
4. Send/set start and goal cells, for example `S00` and `G44`.
5. Send `91` to start mapping.
6. Send `98` to start navigation.
7. Use `0` or `99` to stop motion/navigation during testing.

## Notes

- Generated build products under `Debug/` are ignored by git.
- The technical-report DOCX source is intentionally not kept in this firmware
  repository; report writing should be done separately on the desktop.
