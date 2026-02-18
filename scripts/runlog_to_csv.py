#!/usr/bin/env python3
import argparse
import csv
import math
import re
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None

RUN_RE = re.compile(
    r"(?:Run|Entry)\s+(?P<index>\d+):\s*"
    r"Dist(?:\(m\))?:\s*(?P<dist>-?\d+(?:\.\d+)?)\s*(?:m)?\s*,\s*"
    r"V(?:\(m/s\))?:\s*(?P<speed>-?\d+(?:\.\d+)?)\s*(?:m/s)?\s*,\s*"
    r"Target:\s*(?P<target>-?\d+(?:\.\d+)?)\s*(?:m/s)?\s*,\s*"
    r"PWM\s+L/R:\s*(?P<pwm_l>-?\d+)\s*/\s*(?P<pwm_r>-?\d+)\s*,\s*"
    r"(?:(?:AngleErr\(deg\):\s*(?P<angle_deg>-?\d+(?:\.\d+)?))|"
    r"(?:AngleErr\(rad\):\s*(?P<angle_rad>-?\d+(?:\.\d+)?))|"
    r"(?:CurvR\(m\):\s*(?P<radius_old>-?\d+(?:\.\d+)?)))"
)

DONE_RE = re.compile(r"===\s*(?:All\s+\d+\s+entries\s+output\s+complete|Run\s+log\s+output\s+complete)\s*===")


def parse_lines(lines):
    rows = []
    prev_dist = None

    for line in lines:
        match = RUN_RE.search(line)
        if not match:
            continue

        index = int(match.group("index"))
        dist_m = float(match.group("dist"))
        speed_m_s = float(match.group("speed"))
        target_m_s = float(match.group("target"))
        pwm_l = int(match.group("pwm_l"))
        pwm_r = int(match.group("pwm_r"))

        if match.group("angle_deg") is not None:
            angle_err_deg = float(match.group("angle_deg"))
            angle_err_rad = math.radians(angle_err_deg)
        elif match.group("angle_rad") is not None:
            angle_err_rad = float(match.group("angle_rad"))
            angle_err_deg = math.degrees(angle_err_rad)
        else:
            angle_err_deg = 0.0
            angle_err_rad = 0.0

        if prev_dist is None:
            segment_dist_m = dist_m
        else:
            segment_dist_m = dist_m - prev_dist
        prev_dist = dist_m

        radius_old = match.group("radius_old")
        if radius_old is not None:
            radius_m = float(radius_old)
        else:
            if abs(angle_err_rad) < 1e-6:
                radius_m = 0.0
            else:
                radius_m = segment_dist_m / angle_err_rad

        rows.append(
            {
                "index": index,
                "dist_m": dist_m,
                "segment_dist_m": segment_dist_m,
                "speed_m_s": speed_m_s,
                "target_m_s": target_m_s,
                "pwm_l": pwm_l,
                "pwm_r": pwm_r,
                "angle_err_deg": angle_err_deg,
                "angle_err_rad": angle_err_rad,
                "radius_m": radius_m,
            }
        )

    return rows


def write_csv(out_path: Path, rows):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(
            file_obj,
            fieldnames=[
                "index",
                "dist_m",
                "segment_dist_m",
                "speed_m_s",
                "target_m_s",
                "pwm_l",
                "pwm_r",
                "angle_err_deg",
                "angle_err_rad",
                "radius_m",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def read_lines_from_serial(port: str, baudrate: int, timeout_s: float):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Please run: pip install pyserial")

    lines = []
    print(f"serial_monitor_start port={port} baud={baudrate}")
    print("waiting for log lines... (Ctrl+C to stop)")

    with serial.Serial(port=port, baudrate=baudrate, timeout=timeout_s) as serial_port:
        while True:
            raw = serial_port.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            print(line)
            lines.append(line)

            if DONE_RE.search(line):
                print("log completion marker detected")
                break

    return lines


def main():
    parser = argparse.ArgumentParser(
        description="Convert Aslan serial run log text to CSV and compute radius from 10mm angle error"
    )
    parser.add_argument("--input", "-i", help="Input text file path")
    parser.add_argument("--output", "-o", help="Output CSV file path")
    parser.add_argument("--serial", "-s", default=None, help="Serial port name (example: COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout seconds")
    parser.add_argument("--save-raw", default=None, help="Optional path to save captured serial text")
    args = parser.parse_args()

    if not args.input and not args.serial:
        args.serial = "COM5"

    if args.output:
        out_path = Path(args.output)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = Path("scripts") / f"runlog_{timestamp}.csv"

    if args.input:
        in_path = Path(args.input)
        lines = in_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    else:
        lines = read_lines_from_serial(args.serial, args.baud, args.timeout)
        if args.save_raw:
            raw_path = Path(args.save_raw)
            raw_path.parent.mkdir(parents=True, exist_ok=True)
            raw_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            print(f"saved_raw_log={raw_path}")

    rows = parse_lines(lines)

    write_csv(out_path, rows)

    print(f"parsed_rows={len(rows)}")
    print(f"output_csv={out_path}")


if __name__ == "__main__":
    main()
