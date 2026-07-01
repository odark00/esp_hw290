#!/usr/bin/env python3
"""Read CSV rows streamed by the ESP32 over serial, log them to a file, and
optionally show live scrolling plots.

Usage:
    pip install pyserial            # logging only
    pip install pyserial matplotlib # for --plot

    python3 tools/logger.py                       # auto name, /dev/ttyUSB1
    python3 tools/logger.py --port /dev/ttyUSB0
    python3 tools/logger.py --out run1.csv
    python3 tools/logger.py --plot                # live graphs + CSV
    python3 tools/logger.py --plot --window 40    # 40 s of history

The ESP32 prints a one-line CSV header followed by data rows; diagnostic
lines start with '#' and are echoed to the console but not written to the CSV.
A host timestamp column is prepended to every row.
"""
import argparse
import csv
import threading
import time
from collections import deque

import serial

DEFAULT_HEADER = [
    "t_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
    "mx_uT", "my_uT", "mz_uT", "temp_C", "press_hPa", "alt_m",
]


class Buffers:
    """Rolling numeric history shared between the reader thread and the plot."""

    def __init__(self, maxlen):
        self.lock = threading.Lock()
        self.t = deque(maxlen=maxlen)
        self.series = {k: deque(maxlen=maxlen) for k in DEFAULT_HEADER[1:]}

    def append(self, cols):
        try:
            vals = [float(c) for c in cols[:len(DEFAULT_HEADER)]]
        except ValueError:
            return
        if len(vals) < len(DEFAULT_HEADER):
            return
        with self.lock:
            self.t.append(vals[0] / 1000.0)  # ms -> s
            for k, v in zip(DEFAULT_HEADER[1:], vals[1:]):
                self.series[k].append(v)

    def snapshot(self, keys):
        with self.lock:
            t = list(self.t)
            data = {k: list(self.series[k]) for k in keys}
        return t, data


def reader_loop(ser, out, bufs, stop):
    """Read serial, write CSV, and (if bufs) feed the plot buffers."""
    rows = 0
    header_written = False
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        while not stop.is_set():
            line = ser.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print(line)
                continue

            cols = line.split(",")
            if line.startswith("t_ms"):
                w.writerow(["host_time"] + cols)
                header_written = True
                continue
            if not header_written:
                w.writerow(["host_time"] + DEFAULT_HEADER)
                header_written = True

            w.writerow([time.strftime("%Y-%m-%d %H:%M:%S")] + cols)
            f.flush()
            if bufs is not None:
                bufs.append(cols)
            rows += 1
            if bufs is None and rows % 10 == 0:
                print(f"\r{rows} rows", end="", flush=True)
    print(f"\nStopped. Wrote {rows} rows to {out}")


def run_plot(bufs, window):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    groups = [
        ("Accelerometer (g)", ["ax_g", "ay_g", "az_g"], ["X", "Y", "Z"]),
        ("Gyroscope (deg/s)", ["gx_dps", "gy_dps", "gz_dps"], ["X", "Y", "Z"]),
        ("Magnetometer (uT)", ["mx_uT", "my_uT", "mz_uT"], ["X", "Y", "Z"]),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    fig.canvas.manager.set_window_title("ESP32 HW-290 live")
    axes = axes.ravel()
    lines = {}
    for ax, (title, keys, labels) in zip(axes, groups):
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        for key, lbl in zip(keys, labels):
            (ln,) = ax.plot([], [], label=lbl)
            lines[key] = ln
        ax.legend(loc="upper left", fontsize=8)

    # 4th panel: pressure (left) + temperature (right)
    axp = axes[3]
    axp.set_title("Barometer")
    axp.grid(True, alpha=0.3)
    (ln_press,) = axp.plot([], [], color="tab:blue", label="press (hPa)")
    axt = axp.twinx()
    (ln_temp,) = axt.plot([], [], color="tab:red", label="temp (C)")
    axp.set_ylabel("hPa", color="tab:blue")
    axt.set_ylabel("C", color="tab:red")
    lines["press_hPa"] = ln_press
    lines["temp_C"] = ln_temp

    all_keys = list(lines.keys())

    def update(_frame):
        t, data = bufs.snapshot(all_keys)
        if not t:
            return list(lines.values())
        t0 = t[-1]
        xs = [ti - t0 for ti in t]  # seconds relative to newest sample
        for key, ln in lines.items():
            ln.set_data(xs, data[key])
        for ax in list(axes) + [axt]:
            ax.set_xlim(-window, 0)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        return list(lines.values())

    for ax in axes[2:]:
        ax.set_xlabel("time (s, 0 = now)")
    fig.tight_layout()
    # Keep a reference so the animation isn't garbage-collected.
    anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    plt.show()
    return anim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default=None)
    ap.add_argument("--plot", action="store_true", help="show live graphs")
    ap.add_argument("--window", type=float, default=30.0,
                    help="seconds of history to display (with --plot)")
    args = ap.parse_args()

    out = args.out or time.strftime("imu_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(args.port, args.baud, timeout=2)
    print(f"Logging {args.port} @ {args.baud} -> {out}   (Ctrl-C to stop)")

    if not args.plot:
        try:
            reader_loop(ser, out, None, threading.Event())
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
        return

    # plot mode: serial in a background thread, matplotlib on the main thread
    approx_hz = 5.0  # ESP32 emits ~5 rows/s (200 ms loop)
    bufs = Buffers(maxlen=int(args.window * approx_hz) + 10)
    stop = threading.Event()
    th = threading.Thread(target=reader_loop, args=(ser, out, bufs, stop),
                          daemon=True)
    th.start()
    try:
        run_plot(bufs, args.window)
    finally:
        stop.set()
        ser.close()


if __name__ == "__main__":
    main()
