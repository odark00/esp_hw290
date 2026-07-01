# ESP32 + HW-290 (GY-87) 10-DOF Logger

Reads a **HW-290 / GY-87** sensor module with an ESP32, streams the readings
over USB serial as CSV, and logs them on your laptop — with optional real-time
plots.

## Hardware

| Sensor | Chip | I²C addr | Provides |
|--------|------|----------|----------|
| Accel + Gyro | MPU6050 | `0x68` | acceleration (g), angular rate (°/s) |
| Magnetometer | QMC5883P* | `0x2C` | magnetic field (µT) |
| Barometer | BMP180 | `0x77` | temperature (°C), pressure (hPa), altitude (m) |

\* The HW-290 silkscreen says "HMC5883L", but many boards (including this one)
actually ship a **QMC5883P** at `0x2C`. The firmware auto-detects HMC5883L
(`0x1E`), QMC5883L (`0x0D`), or QMC5883P (`0x2C`).

### Wiring

The module connects to the ESP32 over I²C:

| HW-290 | ESP32 |
|--------|-------|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

The magnetometer sits on the MPU6050's auxiliary I²C bus, so the firmware
enables the MPU6050's **bypass mode** to make it visible on the main bus.

## What the firmware does

On boot ([src/main.cpp](src/main.cpp)) it:

1. Initializes the MPU6050 (±2 g, ±250 °/s) and enables I²C bypass.
2. Scans the I²C bus and prints found addresses (diagnostic).
3. Auto-detects and configures the magnetometer.
4. Reads the BMP180 factory calibration.

Then every 200 ms (~5 Hz) it prints one CSV row over serial at **115200 baud**:

```
t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT,temp_C,press_hPa,alt_m
```

Lines starting with `#` are diagnostics (sensor detection, I²C scan), e.g.:

```
# i2c scan: 0x2C 0x68 0x77
# mag=QMC5883P chipid=0x80
# baro=BMP180
```

## Flashing the firmware

Uses [PlatformIO](https://platformio.org/).

```bash
# Build and upload (port auto-detected)
pio run -t upload
```

The board's serial port shuffles between `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.
on replug — PlatformIO auto-detects it. To watch raw serial output:

```bash
pio device monitor        # 115200 baud, Ctrl-C to exit
```

> Only one program can hold the serial port at a time. Don't run
> `pio device monitor` and the Python logger simultaneously.

## Logging on the laptop

The ESP32 can't write to your laptop's disk — [tools/logger.py](tools/logger.py)
reads the serial stream and writes a timestamped CSV (`imu_YYYYMMDD_HHMMSS.csv`),
prepending a `host_time` column.

### Install dependencies

```bash
pip install -r requirements.txt   # pyserial + matplotlib
```

Or minimally, `pip install pyserial` if you only need CSV logging (no `--plot`).

### Run — CSV only

```bash
python3 tools/logger.py --port /dev/ttyUSB0
```

### Run — with real-time plots

```bash
python3 tools/logger.py --port /dev/ttyUSB0 --plot
```

Opens a 2×2 live dashboard (accel / gyro / magnetometer / barometer) on a
scrolling time window, while still logging to CSV. Close the window to stop.

### Options

| Flag | Default | Meaning |
|------|---------|---------|
| `--port` | `/dev/ttyUSB1` | serial device (check `ls /dev/ttyUSB*`) |
| `--baud` | `115200` | must match the firmware |
| `--out`  | `imu_<timestamp>.csv` | output CSV path |
| `--plot` | off | show live graphs |
| `--window` | `30` | seconds of history shown when plotting |

## Output columns

| Column | Unit | Notes |
|--------|------|-------|
| `t_ms` | ms | ESP32 uptime (`millis()`) |
| `ax_g` `ay_g` `az_g` | g | ~1.0 total at rest (gravity) |
| `gx_dps` `gy_dps` `gz_dps` | °/s | ~0 at rest |
| `mx_uT` `my_uT` `mz_uT` | µT | Earth field ~25–65 µT total |
| `temp_C` | °C | from BMP180 |
| `press_hPa` | hPa | ~1013 at sea level |
| `alt_m` | m | derived from pressure |

## Troubleshooting

- **`could not open port` on upload** — board unplugged or port changed; check
  `ls /dev/ttyUSB*`.
- **`mag=NONE`** — magnetometer not detected. Check the `# i2c scan` line: if a
  new address appears, the board has a different mag chip; tell it apart via the
  scan and extend `magInit()` in [src/main.cpp](src/main.cpp).
- **Magnetometer reads all zeros** — usually MPU6050 bypass not enabled, or a
  wiring issue on SDA/SCL.
- **Permission denied on `/dev/ttyUSB*`** — add your user to the `dialout`
  group: `sudo usermod -aG dialout $USER` (then log out/in).
