#!/usr/bin/env python3
"""Compares the keyframe batch poses before and after REFINE_APP with
reference poses (ADR-019).

Input: a capture of tools/ground/tlm_capture.py with OST_BATCH_SELECTED
(poses from the telemetry) and OST_BATCH_READY (refined or passed on), and
a reference pose file (for Dorfberg the Metashape adjustment,
data/processed/dorfberg_f01/poses_optimized.txt). Images are matched by
their sequence number. Reported per batch view: attitude and position
error against the reference, before and after.

    tools/ground/compare_batch_poses.py run.jsonl data/processed/dorfberg_f01/poses_optimized.txt

Standard library only.
"""

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools/injector"))

from orthostream_injector import dataset, geodesy  # noqa: E402

SOURCES = {1: "TELEMETRY", 2: "REFINED", 3: "REFINE_FAILED"}


def views(r):
    """(image_seq, position, quaternion) of the keyframe and the sources."""
    out = []
    prefixes = ["KEYFRAME"] + [f"SOURCES_{i}" for i in range(r["SOURCE_COUNT"])]
    for p in prefixes:
        pos = (r[f"{p}_POSE_POSITION_ECEF_X"], r[f"{p}_POSE_POSITION_ECEF_Y"], r[f"{p}_POSE_POSITION_ECEF_Z"])
        q = (r[f"{p}_POSE_ATTITUDE_ECEF_FROM_CAMERA_W"], r[f"{p}_POSE_ATTITUDE_ECEF_FROM_CAMERA_X"],
             r[f"{p}_POSE_ATTITUDE_ECEF_FROM_CAMERA_Y"], r[f"{p}_POSE_ATTITUDE_ECEF_FROM_CAMERA_Z"])
        out.append((r[f"{p}_IMAGE_SEQ"], pos, q))
    return out


def summary(label, values, unit, scale=1.0, fmt=".3f"):
    if not values:
        return f"{label}: none"
    v = [x * scale for x in values]
    return (f"{label}: median {statistics.median(v):{fmt}} {unit}, "
            f"mean {statistics.mean(v):{fmt}} {unit}, max {max(v):{fmt}} {unit}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("capture")
    ap.add_argument("reference", help="reference pose file")
    args = ap.parse_args(argv)

    _, ref = dataset.read_pose_file(args.reference)
    by_seq = {dataset.image_number(name, -1): (p, q) for name, (p, q) in ref.items()}

    recs = [json.loads(line) for line in open(args.capture, encoding="utf-8")]
    selected = {r["KEYFRAME_ID"]: r for r in recs if r["packet"] == "OST_BATCH_SELECTED"}
    ready = [r for r in recs if r["packet"] == "OST_BATCH_READY"]
    if not ready:
        print("no OST_BATCH_READY packets", file=sys.stderr)
        return 1

    counts = {}
    kf_before, kf_after, all_before, all_after, pos_after, latency = [], [], [], [], [], []
    for r in ready:
        src = SOURCES.get(r["POSE_SOURCE"], str(r["POSE_SOURCE"]))
        counts[src] = counts.get(src, 0) + 1
        latency.append((r["TRACE_PUBLISH_UTC"] - r["KEYFRAME_CAPTURE_UTC"]) / 1e9)
        before = selected.get(r["KEYFRAME_ID"])
        if before is None or src != "REFINED":
            continue
        for k, ((seq, p1, q1), (_, p0, q0)) in enumerate(zip(views(r), views(before))):
            if seq not in by_seq:
                continue
            pr, qr = by_seq[seq]
            e0 = geodesy.q_angle_deg(q0, qr)
            e1 = geodesy.q_angle_deg(q1, qr)
            all_before.append(e0)
            all_after.append(e1)
            pos_after.append(math.dist(p1, pr))
            if k == 0:
                kf_before.append(e0)
                kf_after.append(e1)

    print(f"{len(selected)} batches selected, {len(ready)} published: "
          + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))
    print(summary("latency (capture to publication)", latency, "s", fmt=".1f"))
    print(summary("keyframe attitude error before", kf_before, "deg", fmt=".4f"))
    print(summary("keyframe attitude error after ", kf_after, "deg", fmt=".4f"))
    print(summary("all views attitude error before", all_before, "deg", fmt=".4f"))
    print(summary("all views attitude error after ", all_after, "deg", fmt=".4f"))
    print(summary("all views position error after", pos_after, "mm", 1000.0, ".1f"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
