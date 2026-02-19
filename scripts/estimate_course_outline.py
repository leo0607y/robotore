#!/usr/bin/env python3
import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

# Custom constant multipliers for course outline tuning
LENGTH_MULTIPLIER = 1.0    # Multiplier for all segment lengths (e.g., 1.05 = 5% longer)
THETA_MULTIPLIER = 2.5    # Multiplier for all rotation angles (e.g., 1.1 = 10% more rotation)


@dataclass
class Sample:
    ds_m: float
    dtheta_rad: float


@dataclass
class Part:
    kind: str
    length_m: float
    theta_rad: float
    radius_m: Optional[float]


def _to_float(value, default=0.0):
    if value is None:
        return default
    text = str(value).strip()
    if text == "":
        return default
    return float(text)


def load_samples_from_csv(csv_path: Path) -> List[Sample]:
    samples: List[Sample] = []
    prev_dist_m: Optional[float] = None

    with csv_path.open("r", encoding="utf-8", errors="ignore", newline="") as file_obj:
        reader = csv.DictReader(file_obj)
        for row in reader:
            if not row:
                continue

            ds_m = _to_float(row.get("segment_dist_m"), 0.0)
            if ds_m <= 0.0:
                dist_m = _to_float(row.get("dist_m"), 0.0)
                if prev_dist_m is None:
                    ds_m = max(0.0, dist_m)
                else:
                    ds_m = max(0.0, dist_m - prev_dist_m)
                prev_dist_m = dist_m

            dtheta_rad = _to_float(row.get("angle_err_rad"), float("nan"))
            if math.isnan(dtheta_rad):
                dtheta_deg = _to_float(row.get("angle_err_deg"), 0.0)
                dtheta_rad = math.radians(dtheta_deg)

            if ds_m <= 0.0:
                continue
            samples.append(Sample(ds_m=ds_m, dtheta_rad=dtheta_rad))

    return samples


def load_samples(input_path: Path) -> List[Sample]:
    if input_path.suffix.lower() == ".csv":
        return load_samples_from_csv(input_path)

    from runlog_to_csv import parse_lines

    lines = input_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    rows = parse_lines(lines)
    samples: List[Sample] = []
    for row in rows:
        ds_m = float(row.get("segment_dist_m", 0.0))
        dtheta_rad = float(row.get("angle_err_rad", 0.0))
        if ds_m <= 0.0:
            continue
        samples.append(Sample(ds_m=ds_m, dtheta_rad=dtheta_rad))
    return samples


def moving_average(values: List[float], window: int) -> List[float]:
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


def build_parts(
    samples: List[Sample],
    k_straight_threshold: float,
    smooth_window: int,
    min_curve_theta_deg: float,
    min_part_m: float,
) -> List[Part]:
    if not samples:
        return []

    curvatures = []
    for sample in samples:
        if sample.ds_m <= 1e-12:
            curvatures.append(0.0)
        else:
            curvatures.append(sample.dtheta_rad / sample.ds_m)
    k_smooth = moving_average(curvatures, smooth_window)

    labels = []
    for value in k_smooth:
        if abs(value) < k_straight_threshold:
            labels.append("straight")
        elif value > 0:
            labels.append("left")
        else:
            labels.append("right")

    parts: List[Part] = []
    current_label = labels[0]
    current_length = 0.0
    current_theta = 0.0

    for sample, label in zip(samples, labels):
        if label != current_label:
            parts.append(part_from_group(current_label, current_length, current_theta))
            current_label = label
            current_length = 0.0
            current_theta = 0.0

        current_length += sample.ds_m
        current_theta += sample.dtheta_rad

    parts.append(part_from_group(current_label, current_length, current_theta))

    min_curve_theta_rad = math.radians(min_curve_theta_deg)
    normalized: List[Part] = []
    for part in parts:
        if part.kind in ("left", "right") and abs(part.theta_rad) < min_curve_theta_rad:
            normalized.append(Part(kind="straight", length_m=part.length_m, theta_rad=part.theta_rad, radius_m=None))
        else:
            normalized.append(part)

    merged: List[Part] = []
    for part in normalized:
        if not merged:
            merged.append(part)
            continue
        if merged[-1].kind == part.kind:
            merged[-1] = merge_parts(merged[-1], part)
        else:
            merged.append(part)

    reduced: List[Part] = []
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


def part_from_group(label: str, length_m: float, theta_rad: float) -> Part:
    if label == "straight":
        return Part(kind="straight", length_m=length_m, theta_rad=theta_rad, radius_m=None)
    if abs(theta_rad) < 1e-12:
        radius_m = None
    else:
        radius_m = abs(length_m / theta_rad)
    kind = "left" if label == "left" else "right"
    return Part(kind=kind, length_m=length_m, theta_rad=theta_rad, radius_m=radius_m)


def merge_parts(first: Part, second: Part) -> Part:
    kind = first.kind if first.kind == second.kind else first.kind
    length_m = first.length_m + second.length_m
    theta_rad = first.theta_rad + second.theta_rad

    if kind == "straight":
        radius_m = None
    elif abs(theta_rad) < 1e-12:
        radius_m = None
    else:
        radius_m = abs(length_m / theta_rad)

    return Part(kind=kind, length_m=length_m, theta_rad=theta_rad, radius_m=radius_m)


def part_text(index: int, part: Part) -> str:
    # Apply custom constant multipliers
    length_cm = part.length_m * 100.0 * LENGTH_MULTIPLIER
    theta_deg = math.degrees(part.theta_rad) * THETA_MULTIPLIER

    if part.kind == "straight":
        return f"{index:02d}. STRAIGHT  L={length_cm:.1f} cm"

    direction = "LEFT" if part.kind == "left" else "RIGHT"
    if part.radius_m is None:
        return f"{index:02d}. {direction} CURVE  L={length_cm:.1f} cm  θ={theta_deg:.1f} deg  R=unknown"

    # Recalculate radius with adjusted length and angle
    if abs(theta_deg) < 1e-6:
        radius_cm = None
    else:
        radius_cm = abs((part.length_m * LENGTH_MULTIPLIER) / math.radians(theta_deg))
    
    if radius_cm is None:
        return f"{index:02d}. {direction} CURVE  L={length_cm:.1f} cm  θ={theta_deg:.1f} deg  R=unknown"
    
    return f"{index:02d}. {direction} CURVE  L={length_cm:.1f} cm  θ={theta_deg:.1f} deg  R≈{radius_cm:.1f} cm"


def write_summary(output_path: Path, parts: List[Part]):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as file_obj:
        file_obj.write("# Estimated Course Outline\n")
        for idx, part in enumerate(parts, start=1):
            file_obj.write(part_text(idx, part) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Estimate rough course outline (straight length and curve radius) from run log CSV or raw text"
    )
    parser.add_argument("input", help="Input file path (.csv from runlog_to_csv.py or raw serial text)")
    parser.add_argument("--output", "-o", default=None, help="Optional output text path")
    parser.add_argument(
        "--k-straight",
        type=float,
        default=0.8,
        help="Curvature threshold [1/m] treated as straight after smoothing (default: 0.8)",
    )
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=9,
        help="Moving-average window size for curvature smoothing (default: 9)",
    )
    parser.add_argument(
        "--min-curve-theta-deg",
        type=float,
        default=8.0,
        help="Curve segments with |theta| below this degree are folded into straight (default: 8)",
    )
    parser.add_argument(
        "--min-part-cm",
        type=float,
        default=5.0,
        help="Very short parts below this length are merged to neighbors (default: 5cm)",
    )
    parser.add_argument(
        "--length-multiplier",
        type=float,
        default=1.0,
        help="Custom multiplier for all segment lengths (default: 1.0, e.g. 1.05 = 5%% longer)",
    )
    parser.add_argument(
        "--theta-multiplier",
        type=float,
        default=1.0,
        help="Custom multiplier for all rotation angles (default: 1.0, e.g. 1.1 = 10%% more rotation)",
    )
    args = parser.parse_args()

    # Apply global multiplier constants from command line
    global LENGTH_MULTIPLIER, THETA_MULTIPLIER
    LENGTH_MULTIPLIER = args.length_multiplier
    THETA_MULTIPLIER = args.theta_multiplier

    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Input not found: {input_path}")

    smooth_window = max(1, args.smooth_window)
    if smooth_window % 2 == 0:
        smooth_window += 1

    samples = load_samples(input_path)
    parts = build_parts(
        samples=samples,
        k_straight_threshold=args.k_straight,
        smooth_window=smooth_window,
        min_curve_theta_deg=args.min_curve_theta_deg,
        min_part_m=max(0.0, args.min_part_cm / 100.0),
    )

    total_m = sum(sample.ds_m for sample in samples)
    print(f"samples={len(samples)} total_distance={total_m:.3f} m")
    print(f"constants: LENGTH_MULTIPLIER={LENGTH_MULTIPLIER} THETA_MULTIPLIER={THETA_MULTIPLIER}")
    print("--- estimated outline ---")
    for idx, part in enumerate(parts, start=1):
        print(part_text(idx, part))

    if args.output:
        out_path = Path(args.output)
        write_summary(out_path, parts)
        print(f"summary_output={out_path}")


if __name__ == "__main__":
    main()
