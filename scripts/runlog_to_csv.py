#!/usr/bin/env python3
import argparse
import csv
import math
import re
<<<<<<< HEAD
=======
from dataclasses import dataclass
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)
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

<<<<<<< HEAD
COMPACT_RE = re.compile(
    r"(?:Run|Entry)\s+(?P<index>\d+):\s*"
    r"Dist(?:\(m\))?:\s*(?P<dist>-?\d+(?:\.\d+)?)\s*(?:m)?\s*,\s*"
    r"(?:AngleErr\(deg\):\s*(?P<angle_deg>-?\d+(?:\.\d+)?)|"
    r"AngleErr\(rad\):\s*(?P<angle_rad>-?\d+(?:\.\d+)?))"
)

DONE_RE = re.compile(r"===\s*(?:All\s+\d+\s+entries\s+output\s+complete|Run\s+log\s+output\s+complete|Run\s+compact\s+log\s+output\s+complete)\s*===")
=======
RUN_MIN_RE = re.compile(
    r"(?:Run|Entry)\s+(?P<index>\d+):\s*"
    r"Dist(?:\(m\))?:?\s*(?P<dist>-?\d+(?:\.\d+)?)\s*(?:m)?\s*,\s*"
    r"AngleErr\((?P<angle_unit>rad|deg)\):\s*(?P<angle>-?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)

DONE_RE = re.compile(r"===\s*(?:All\s+\d+\s+entries\s+output\s+complete|Run\s+log\s+output\s+complete)\s*===")
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)


@dataclass
class Part:
    kind: str
    length_m: float
    theta_rad: float
    radius_m: float | None


def moving_average(values, window):
    if window <= 1:
        return values[:]
    n = len(values)
    half = window // 2
    out = [0.0] * n
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)

    for idx in range(n):
        left = max(0, idx - half)
        right = min(n, idx + half + 1)
        total = prefix[right] - prefix[left]
        out[idx] = total / (right - left)
    return out


def part_from_group(label, length_m, theta_rad):
    if label == "straight":
        return Part(kind="straight", length_m=length_m, theta_rad=theta_rad, radius_m=None)
    if abs(theta_rad) < 1e-12:
        radius_m = None
    else:
        radius_m = abs(length_m / theta_rad)
    return Part(kind=label, length_m=length_m, theta_rad=theta_rad, radius_m=radius_m)


def merge_parts(first, second):
    kind = first.kind if first.kind == second.kind else first.kind
    length_m = first.length_m + second.length_m
    theta_rad = first.theta_rad + second.theta_rad
    if kind == "straight" or abs(theta_rad) < 1e-12:
        radius_m = None
    else:
        radius_m = abs(length_m / theta_rad)
    return Part(kind=kind, length_m=length_m, theta_rad=theta_rad, radius_m=radius_m)


def build_parts(rows, k_straight_threshold, smooth_window, min_part_m):
    if not rows:
        return []

    curvatures = []
    for row in rows:
        ds_m = float(row["segment_dist_m"])
        dtheta_rad = float(row["angle_err_rad"])
        if ds_m <= 1e-12:
            curvatures.append(0.0)
        else:
            curvatures.append(dtheta_rad / ds_m)

    k_smooth = moving_average(curvatures, smooth_window)

    labels = []
    for value in k_smooth:
        if abs(value) < k_straight_threshold:
            labels.append("straight")
        elif value > 0:
            labels.append("left")
        else:
            labels.append("right")

    parts = []
    current_label = labels[0]
    current_length = 0.0
    current_theta = 0.0

    for row, label in zip(rows, labels):
        if label != current_label:
            parts.append(part_from_group(current_label, current_length, current_theta))
            current_label = label
            current_length = 0.0
            current_theta = 0.0

        current_length += float(row["segment_dist_m"])
        current_theta += float(row["angle_err_rad"])

    parts.append(part_from_group(current_label, current_length, current_theta))

    merged = []
    for part in parts:
        if not merged:
            merged.append(part)
            continue
        if merged[-1].kind == part.kind:
            merged[-1] = merge_parts(merged[-1], part)
        else:
            merged.append(part)

    reduced = []
    for part in merged:
        if not reduced:
            reduced.append(part)
            continue
        if part.length_m < min_part_m:
            reduced[-1] = merge_parts(reduced[-1], part)
        else:
            reduced.append(part)

    if len(reduced) >= 2 and reduced[0].length_m < min_part_m:
        reduced[1] = merge_parts(reduced[0], reduced[1])
        reduced = reduced[1:]

    return reduced


def part_text(index, part, large_theta_deg, sharp_radius_cm):
    length_cm = part.length_m * 100.0
    theta_deg = math.degrees(part.theta_rad)

    if part.kind == "straight":
        return f"{index:02d}. STRAIGHT  L={length_cm:.1f} cm"

    direction = "LEFT" if part.kind == "left" else "RIGHT"
    radius_cm = None if part.radius_m is None else part.radius_m * 100.0

    if radius_cm is not None and radius_cm <= sharp_radius_cm:
        curve_type = "SHARP CURVE"
    elif abs(theta_deg) >= large_theta_deg:
        curve_type = "LARGE CURVE"
    else:
        curve_type = "LARGE CURVE"

    if radius_cm is None:
        return f"{index:02d}. {direction} {curve_type}  L={length_cm:.1f} cm  θ={theta_deg:.1f} deg  R=unknown"
    return f"{index:02d}. {direction} {curve_type}  L={length_cm:.1f} cm  θ={theta_deg:.1f} deg  R≈{radius_cm:.1f} cm"


def write_course_outline(out_path, parts, large_theta_deg, sharp_radius_cm):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as file_obj:
        file_obj.write("# Estimated Course Outline\n")
        for idx, part in enumerate(parts, start=1):
            file_obj.write(part_text(idx, part, large_theta_deg, sharp_radius_cm) + "\n")


def parse_lines(lines):
    rows = []
    prev_dist = None

    for line in lines:
        match = RUN_RE.search(line)
<<<<<<< HEAD
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
=======
        if match:
            index = int(match.group("index"))
            dist_m = float(match.group("dist"))

            if match.group("angle_deg") is not None:
                angle_err_deg = float(match.group("angle_deg"))
                angle_err_rad = math.radians(angle_err_deg)
            elif match.group("angle_rad") is not None:
                angle_err_rad = float(match.group("angle_rad"))
                angle_err_deg = math.degrees(angle_err_rad)
            else:
                angle_err_deg = 0.0
                angle_err_rad = 0.0
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)
        else:
            match_min = RUN_MIN_RE.search(line)
            if not match_min:
                continue

            index = int(match_min.group("index"))
            dist_m = float(match_min.group("dist"))
            angle_value = float(match_min.group("angle"))
            angle_unit = (match_min.group("angle_unit") or "rad").lower()
            if angle_unit == "deg":
                angle_err_deg = angle_value
                angle_err_rad = math.radians(angle_value)
            else:
                angle_err_rad = angle_value
                angle_err_deg = math.degrees(angle_value)

        if prev_dist is None:
            segment_dist_m = dist_m
        else:
            segment_dist_m = dist_m - prev_dist
        prev_dist = dist_m

<<<<<<< HEAD
        radius_old = match.group("radius_old") if match else None
        if radius_old is not None:
            radius_m = float(radius_old)
        else:
            if abs(angle_err_rad) < 1e-6:
                radius_m = 0.0
            else:
                radius_m = segment_dist_m / angle_err_rad

=======
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)
        rows.append(
            {
                "index": index,
                "dist_m": dist_m,
                "segment_dist_m": segment_dist_m,
                "angle_err_rad": angle_err_rad,
            }
        )

    return rows


<<<<<<< HEAD
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
=======
def _to_float(value, default=0.0):
    if value is None:
        return default
    text = str(value).strip()
    if text == "":
        return default
    return float(text)


def parse_csv_input(in_path: Path):
    rows = []
    prev_dist = None

    with in_path.open("r", encoding="utf-8", errors="ignore", newline="") as file_obj:
        reader = csv.DictReader(file_obj)
        for index, row in enumerate(reader):
            if not row:
                continue

            dist_m = _to_float(row.get("dist_m"), 0.0)
            segment_dist_m = _to_float(row.get("segment_dist_m"), float("nan"))

            if math.isnan(segment_dist_m):
                if prev_dist is None:
                    segment_dist_m = max(0.0, dist_m)
                else:
                    segment_dist_m = max(0.0, dist_m - prev_dist)
            prev_dist = dist_m

            angle_err_rad = _to_float(row.get("angle_err_rad"), float("nan"))
            if math.isnan(angle_err_rad):
                angle_err_deg = _to_float(row.get("angle_err_deg"), 0.0)
                angle_err_rad = math.radians(angle_err_deg)

            rows.append(
                {
                    "index": int(_to_float(row.get("index"), index)),
                    "dist_m": dist_m,
                    "segment_dist_m": segment_dist_m,
                    "angle_err_rad": angle_err_rad,
                }
            )

    return rows


def write_distance_theta_txt(out_path: Path, rows):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as file_obj:
        for row in rows:
            file_obj.write(f"{row['segment_dist_m']:.6f},{row['angle_err_rad']:.9f}\n")
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)


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
        description="Convert Aslan serial run log text to Distance, Theta.txt (segment distance + segment angle error rad)"
    )
    parser.add_argument("--input", "-i", help="Input text file path")
    parser.add_argument("--output", "-o", help="Output text file path")
    parser.add_argument("--outline-output", default=r"C:\Users\reoch\reRo\Aslan\course_outline_estimated.txt", help="Estimated course outline output path")
    parser.add_argument("--serial", "-s", default=None, help="Serial port name (example: COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout seconds")
    parser.add_argument("--save-raw", default=None, help="Optional path to save captured serial text")
<<<<<<< HEAD
    parser.add_argument(
        "--matlab-output",
        default=None,
        help="Optional MATLAB text output path (default: same folder as CSV, file name 'Distance, Theta.txt')",
    )
=======
    parser.add_argument("--k-straight", type=float, default=0.8, help="Curvature threshold [1/m] treated as straight")
    parser.add_argument("--smooth-window", type=int, default=9, help="Moving-average window size for curvature")
    parser.add_argument("--min-part-cm", type=float, default=5.0, help="Very short parts below this length are merged")
    parser.add_argument("--large-theta-deg", type=float, default=100.0, help="Threshold degree for LARGE CURVE label")
    parser.add_argument("--sharp-radius-cm", type=float, default=20.0, help="Threshold radius [cm] for SHARP CURVE label (<=20cm)")
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)
    args = parser.parse_args()

    if not args.input and not args.serial:
        args.serial = "COM5"

    if args.output:
        out_path = Path(args.output)
    else:
<<<<<<< HEAD
        out_path = DEFAULT_OUTPUT_DIR / DEFAULT_CSV_NAME

    if args.matlab_output:
        matlab_out_path = Path(args.matlab_output)
    else:
        matlab_out_path = DEFAULT_OUTPUT_DIR / DEFAULT_MATLAB_NAME
=======
        out_path = Path(r"C:\Users\reoch\reRo\Aslan\Distance, Theta.txt")
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)

    if args.input:
        in_path = Path(args.input)
        if in_path.suffix.lower() == ".csv":
            rows = parse_csv_input(in_path)
        else:
            lines = in_path.read_text(encoding="utf-8", errors="ignore").splitlines()
            rows = parse_lines(lines)
    else:
        lines = read_lines_from_serial(args.serial, args.baud, args.timeout)
        if args.save_raw:
            raw_path = Path(args.save_raw)
            raw_path.parent.mkdir(parents=True, exist_ok=True)
            raw_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            print(f"saved_raw_log={raw_path}")
        rows = parse_lines(lines)

<<<<<<< HEAD
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
=======
    write_distance_theta_txt(out_path, rows)

    smooth_window = max(1, int(args.smooth_window))
    if smooth_window % 2 == 0:
        smooth_window += 1

    parts = build_parts(
        rows,
        k_straight_threshold=float(args.k_straight),
        smooth_window=smooth_window,
        min_part_m=max(0.0, float(args.min_part_cm) / 100.0),
    )
    outline_path = Path(args.outline_output)
    write_course_outline(
        outline_path,
        parts,
        large_theta_deg=float(args.large_theta_deg),
        sharp_radius_cm=float(args.sharp_radius_cm),
    )

    print(f"parsed_rows={len(rows)}")
    print(f"output_text={out_path}")
    print(f"outline_output={outline_path}")
>>>>>>> 0954dc8 (Refine IMU/log pipeline and add course outline export tools)
out_path}")
    print(f"outline_output={outline_path}")


if __name__ == "__main__":
    main()
