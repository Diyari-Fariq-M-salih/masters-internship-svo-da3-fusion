#!/usr/bin/env python3
"""Convert EuRoC ground-truth CSV to TUM trajectory format."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import yaml


def quat_wxyz_to_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(R))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(R)))
        if idx == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif idx == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


def load_t_bs(sensor_yaml: str | None) -> np.ndarray:
    if sensor_yaml is None:
        return np.eye(4, dtype=np.float64)
    with Path(sensor_yaml).open() as f:
        data = yaml.safe_load(f)
    values = data["T_BS"]["data"]
    return np.array(values, dtype=np.float64).reshape(4, 4)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input_csv", required=True, help="EuRoC state_groundtruth_estimate0/data.csv")
    parser.add_argument("--output_tum", required=True, help="Output TUM trajectory txt")
    parser.add_argument(
        "--sensor_yaml",
        default=None,
        help="Optional EuRoC sensor.yaml. If set, writes GT for that sensor frame using T_BS.",
    )
    args = parser.parse_args()

    output = Path(args.output_tum)
    output.parent.mkdir(parents=True, exist_ok=True)
    T_b_s = load_t_bs(args.sensor_yaml)

    count = 0
    with Path(args.input_csv).open(newline="") as src, output.open("w") as dst:
        dst.write("# timestamp tx ty tz qx qy qz qw\n")
        reader = csv.reader(src)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            timestamp = float(row[0]) * 1e-9
            p_w_b = np.array([float(v) for v in row[1:4]], dtype=np.float64)
            qw, qx, qy, qz = row[4:8]
            R_w_b = quat_wxyz_to_matrix(float(qw), float(qx), float(qy), float(qz))
            T_w_b = np.eye(4, dtype=np.float64)
            T_w_b[:3, :3] = R_w_b
            T_w_b[:3, 3] = p_w_b
            T_w_s = T_w_b @ T_b_s
            qx_out, qy_out, qz_out, qw_out = matrix_to_quat_xyzw(T_w_s[:3, :3])
            dst.write(
                f"{timestamp:.9f} "
                f"{T_w_s[0, 3]:.9f} {T_w_s[1, 3]:.9f} {T_w_s[2, 3]:.9f} "
                f"{qx_out:.9f} {qy_out:.9f} {qz_out:.9f} {qw_out:.9f}\n"
            )
            count += 1

    print(f"Wrote {count} poses to {output}")


if __name__ == "__main__":
    main()
