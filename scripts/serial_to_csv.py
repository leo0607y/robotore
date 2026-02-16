import argparse
import csv
import re
import time
from pathlib import Path

import serial

ENTRY_RE = re.compile(r"Entry\s+(\d+):\s+Dist\(m\):\s+([+-]?[0-9]*\.?[0-9]+),\s+Curvature Radius:\s+([+-]?[0-9]*\.?[0-9]+)")


def main():
    parser = argparse.ArgumentParser(description="Capture serial log output and save CSV.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(r"C:\Users\reoch\reRo\Aslan\log.csv"),
        help="Output CSV path",
    )
    parser.add_argument("--stop", default="output complete", help="Stop string to end capture")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout seconds")
    args = parser.parse_args()

    rows = []
    stop_lower = args.stop.lower()

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            lower = line.lower()
            if stop_lower in lower:
                break

            match = ENTRY_RE.search(line)
            if match:
                dist = float(match.group(2))
                radius = float(match.group(3))
                rows.append((dist, radius))

    with args.out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["dist_m", "radius_m"])
        writer.writerows(rows)

    print(f"Saved {len(rows)} rows to {args.out}")


if __name__ == "__main__":
    main()
