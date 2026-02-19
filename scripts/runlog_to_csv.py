#!/usr/bin/env python3
import argparse
import csv
import math
import re
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None

DEFAULT_OUTPUT_DIR = Path(r"C:\Users\reoch\reRo\Aslan")
DEFAULT_CSV_NAME = "runlog_auto.csv"
DEFAULT_MATLAB_NAME = "Distance, Theta.txt"

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

COMPACT_RE = re.compile(
    r"(?:Run|Entry)\s+(?P<index>\d+):\s*"
    r"Dist(?:\(m\))?:\s*(?P<dist>-?\d+(?:\.\d+)?)\s*(?:m)?\s*,\s*"
    r"(?:AngleErr\(deg\):\s*(?P<angle_deg>-?\d+(?:\.\d+)?)|"
    r"AngleErr\(rad\):\s*(?P<angle_rad>-?\d+(?:\.\d+)?))"
)

DONE_RE = re.compile(r"===\s*(?:All\s+\d+\s+entries\s+output\s+complete|Run\s+log\s+output\s+complete|Run\s+compact\s+log\s+output\s+complete)\s*===")


def parse_lines(lines):
    rows = []
    prev_dist = None

    for line in lines:
        match = RUN_RE.search(line)
        compact_match = None
        if not match:
            compact_match = COMPACT_RE.search(line)
            if not compact_match:
                continue

        if match:
            index = int(match.group("index"))
            dist_m = float(match.group("dist"))
            speed_m_s = float(match.group("speed"))
            target_m_s = float(match.group("target"))
            pwm_l = int(match.group("pwm_l"))
            pwm_r = int(match.group("pwm_r"))
        else:
            index = int(compact_match.group("index"))
            dist_m = float(compact_match.group("dist"))
            speed_m_s = 0.0
            target_m_s = 0.0
            pwm_l = 0
            pwm_r = 0

        if match and match.group("angle_deg") is not None:
            angle_err_deg = float(match.group("angle_deg"))
            angle_err_rad = math.radians(angle_err_deg)
        elif match and match.group("angle_rad") is not None:
            angle_err_rad = float(match.group("angle_rad"))
            angle_err_deg = math.degrees(angle_err_rad)
        elif compact_match and compact_match.group("angle_deg") is not None:
            angle_err_deg = float(compact_match.group("angle_deg"))
            angle_err_rad = math.radians(angle_err_deg)
        elif compact_match and compact_match.group("angle_rad") is not None:
            angle_err_rad = float(compact_match.group("angle_rad"))
            angle_err_deg = math.degrees(angle_err_rad)
        else:
            angle_err_deg = 0.0
            angle_err_rad = 0.0

        if prev_dist is None:
            segment_dist_m = dist_m
        else:
            segment_dist_m = dist_m - prev_dist
        prev_dist = dist_m

        radius_old = match.group("radius_old") if match else None
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


def append_trajectory(rows, straight_radius=1000.0):
    x_m = 0.0
    y_m = 0.0
    heading_rad = 0.0

    for row in rows:
        distance = row["segment_dist_m"]
        theta = row["angle_err_rad"]

        x_m += distance * math.cos(heading_rad + theta / 2.0)
        y_m += distance * math.sin(heading_rad + theta / 2.0)
        heading_rad += theta

        if abs(theta) < 1e-9:
            radius_for_speed_plan_m = straight_radius
        else:
            radius_for_speed_plan_m = abs(distance / theta)
            if radius_for_speed_plan_m >= straight_radius:
                radius_for_speed_plan_m = straight_radius

        row["x_m"] = x_m
        row["y_m"] = y_m
        row["heading_rad"] = heading_rad
        row["radius_for_speed_plan_m"] = radius_for_speed_plan_m


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
                "x_m",
                "y_m",
                "heading_rad",
                "radius_for_speed_plan_m",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def write_matlab_distance_theta(out_path: Path, rows):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as file_obj:
        for row in rows:
            distance = row["segment_dist_m"]
            theta = row["angle_err_rad"]
            file_obj.write(f"{distance:.9f} {theta:.9f}\n")


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
    parser.add_argument(
        "--matlab-output",
        default=None,
        help="Optional MATLAB text output path (default: same folder as CSV, file name 'Distance, Theta.txt')",
    )
    args = parser.parse_args()

    if not args.input and not args.serial:
        args.serial = "COM5"

    if args.output:
        out_path = Path(args.output)
    else:
        out_path = DEFAULT_OUTPUT_DIR / DEFAULT_CSV_NAME

    if args.matlab_output:
        matlab_out_path = Path(args.matlab_output)
    else:
        matlab_out_path = DEFAULT_OUTPUT_DIR / DEFAULT_MATLAB_NAME

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
    append_trajectory(rows)

    if rows:
        theta_values = [abs(row["angle_err_rad"]) for row in rows]
        nonzero_theta_count = sum(1 for value in theta_values if value > 1e-9)
        max_abs_theta = max(theta_values)
    else:
        nonzero_theta_count = 0
        max_abs_theta = 0.0

    write_csv(out_path, rows)
    write_matlab_distance_theta(matlab_out_path, rows)

    print(f"parsed_rows={len(rows)}")
    print(f"nonzero_theta_rows={nonzero_theta_count}")
    print(f"max_abs_theta_rad={max_abs_theta:.9f}")
    if len(rows) > 0 and nonzero_theta_count == 0:
        print("WARNING: all theta values are zero. Check firmware AngleErr(rad) output and IMU data path.")
    print(f"output_csv={out_path}")
    print(f"output_matlab_txt={matlab_out_path}")


if __name__ == "__main__":
    main()
