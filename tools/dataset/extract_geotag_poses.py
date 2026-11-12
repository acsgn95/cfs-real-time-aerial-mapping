#!/usr/bin/env python3
"""Converts the recorded geotags of a Wingtra flight into camera poses.

This is the `non-optimized` pose set of ADR-003: positions and attitudes as
the aircraft recorded them, without bundle adjustment. Output format is
that of tools/dataset/extract_metashape_poses.py, so all tools accept it:

    camera W H fx fy cx cy k1 k2 k3 p1 p2      (copied from a pose file)
    pose image x y z qw qx qy qz               (ECEF, q_E_C, w >= 0)

Geotags (WingtraHub CSV): longitude, latitude, ellipsoidal height (WGS 84 /
ETRS89), yaw, pitch, roll in degrees. The attitude convention was
determined against the Metashape poses of Dorfberg Flight 01 (2026-11-13):

    R_N_C = Rz(yaw) * Rx(roll) * Ry(pitch) * B

with N the local north-east-down frame and B the camera mounting. Only the
nominal mounting is used here (the camera is turned by 90 degrees about the
optical axis): a system knows it from its design, but not the small
misalignment of about 1.3 degrees nor the antenna lever arm, which a
calibration would give. With the fitted mounting the remaining attitude
difference to the Metashape poses is 0.53 degrees (median); with the
nominal mounting it includes the misalignment.

Usage:
    extract_geotag_poses.py <geotags.csv> <camera pose file> <output> [--compare]

The camera line is copied from the given pose file (normally the optimized
poses, whose calibration is the Metashape self-calibration). --compare
prints the position and attitude differences to that file's poses. Only the
Python standard library is used.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563
E2 = F_WGS84 * (2.0 - F_WGS84)

# Nominal mounting: camera x along body y (right wing), camera y along -body x.
B_NOMINAL = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rot(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == "y":
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def r_e_n(lat, lon):
    """Rotation from local north-east-down to ECEF."""
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return [[-sl * co, -so, -cl * co], [-sl * so, co, -cl * so], [cl, 0.0, -sl]]


def geodetic_to_ecef(lat, lon, h):
    n = A_WGS84 / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
    return [(n + h) * math.cos(lat) * math.cos(lon), (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - E2) + h) * math.sin(lat)]


def quaternion(r):
    """Hamilton quaternion (w, x, y, z) with w >= 0 of a rotation matrix."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        q = [(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s]
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        q = [(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s]
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        q = [(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s]
    n = math.sqrt(sum(v * v for v in q))
    q = [v / n for v in q]
    return [-v for v in q] if q[0] < 0 else q


def quat_matrix(q):
    w, a, b, c = q
    return [[1 - 2 * (b * b + c * c), 2 * (a * b - c * w), 2 * (a * c + b * w)],
            [2 * (a * b + c * w), 1 - 2 * (a * a + c * c), 2 * (b * c - a * w)],
            [2 * (a * c - b * w), 2 * (b * c + a * w), 1 - 2 * (a * a + b * b)]]


def read_pose_file(path):
    camera, poses = None, {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            t = line.split()
            if not t or t[0].startswith("#"):
                continue
            if t[0] == "camera":
                camera = line.strip()
            elif t[0] == "pose":
                poses[t[1]] = ([float(v) for v in t[2:5]], [float(v) for v in t[5:9]])
    if camera is None:
        sys.exit(f"no camera line in {path}")
    return camera, poses


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("geotags", type=Path)
    ap.add_argument("camera_poses", type=Path, help="pose file whose camera line is copied")
    ap.add_argument("output", type=Path)
    ap.add_argument("--compare", action="store_true", help="print differences to the camera pose file")
    args = ap.parse_args()

    camera, reference = read_pose_file(args.camera_poses)
    lines = []
    diffs = []
    with open(args.geotags, encoding="utf-8") as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            name = row[0]
            lon, lat, h, yaw, pitch, roll = (float(v) for v in row[1:7])
            lat_r, lon_r = math.radians(lat), math.radians(lon)
            r_n_c = matmul(matmul(matmul(rot("z", math.radians(yaw)), rot("x", math.radians(roll))),
                                  rot("y", math.radians(pitch))), B_NOMINAL)
            r_e_c = matmul(r_e_n(lat_r, lon_r), r_n_c)
            p = geodetic_to_ecef(lat_r, lon_r, h)
            q = quaternion(r_e_c)
            lines.append(f"pose {name} {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} "
                         f"{q[0]:.12f} {q[1]:.12f} {q[2]:.12f} {q[3]:.12f}")
            if args.compare and name in reference:
                ref_p, ref_q = reference[name]
                d_e = [p[i] - ref_p[i] for i in range(3)]
                d_n = [sum(r_e_n(lat_r, lon_r)[k][i] * d_e[k] for k in range(3)) for i in range(3)]
                rel = matmul(transpose(r_e_c), quat_matrix(ref_q))
                angle = math.degrees(math.acos(max(-1.0, min(1.0, (rel[0][0] + rel[1][1] + rel[2][2] - 1) / 2))))
                diffs.append((d_n, angle))

    header = [
        "# Camera poses from the recorded geotags (non-optimized, ADR-003).",
        "# Generated by tools/dataset/extract_geotag_poses.py; do not edit.",
        f"# Source: {args.geotags.name}; camera line from {args.camera_poses.name}.",
        "# Attitude: R_N_C = Rz(yaw) Rx(roll) Ry(pitch) B_nominal; no boresight or lever-arm calibration.",
        "#",
        "# camera width height fx fy cx cy k1 k2 k3 p1 p2",
        "# pose image x y z qw qx qy qz",
        "#   camera centre in ECEF (m) and q_E_C (camera to ECEF), Hamilton, w >= 0.",
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(header + [camera] + lines) + "\n", encoding="utf-8")
    print(f"{len(lines)} poses written to {args.output}")
    if diffs:
        n = len(diffs)
        mean = [sum(d[0][i] for d in diffs) / n for i in range(3)]
        std = [math.sqrt(sum((d[0][i] - mean[i]) ** 2 for d in diffs) / n) for i in range(3)]
        angles = sorted(d[1] for d in diffs)
        print("position difference to the reference (north, east, down):"
              f" mean {mean[0]:+.3f} {mean[1]:+.3f} {mean[2]:+.3f} m,"
              f" std {std[0]:.3f} {std[1]:.3f} {std[2]:.3f} m")
        print(f"attitude difference: median {angles[n // 2]:.2f} deg, mean {sum(angles) / n:.2f} deg,"
              f" 95 % {angles[int(0.95 * (n - 1))]:.2f} deg, max {angles[-1]:.2f} deg")


if __name__ == "__main__":
    main()
