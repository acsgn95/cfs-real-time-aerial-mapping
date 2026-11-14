#!/usr/bin/env python3
"""Compares the ImagePose messages of ALIGN_APP with the poses the injector
was given (ADR-014 section 4).

The injector's trajectory passes through the pose of every image at its
capture time, so the difference is the error of the interpolation between
the telemetry samples, plus anything lost on the way (frames, mounting,
time). Input: a capture of tools/ground/tlm_capture.py and the pose file of
the injector run. Images are matched by their sequence number, which the
injector takes from the file name.

    tools/ground/compare_image_poses.py run.jsonl data/processed/dorfberg_f01/poses_optimized.txt

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

RESULTS = ["VALID", "NO_SAMPLE_BEFORE", "GAP_TOO_LARGE", "WAIT_EXPIRED", "NO_CALIBRATION", "QUEUE_FULL", "CLEARED"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("capture", help="JSON lines of tools/ground/tlm_capture.py")
    ap.add_argument("poses", help="pose file of the injector run")
    ap.add_argument("--session", type=int, default=None, help="session identifier (default: the last one)")
    args = ap.parse_args(argv)

    _, poses = dataset.read_pose_file(args.poses)
    by_seq = {dataset.image_number(name, -1): (name, p, q) for name, (p, q) in poses.items()}

    records = [json.loads(line) for line in open(args.capture, encoding="utf-8")]
    records = [r for r in records if r["packet"] == "ALIGN_APP_IMAGE_POSE"]
    if not records:
        print("no ALIGN_APP_IMAGE_POSE packets", file=sys.stderr)
        return 1
    session = args.session if args.session is not None else records[-1]["SESSION_ID"]
    records = [r for r in records if r["SESSION_ID"] == session]

    results = {}
    att, pos, waited, latency = [], [], [], []
    for r in records:
        res = RESULTS[r["RESULT"]] if r["RESULT"] < len(RESULTS) else str(r["RESULT"])
        results[res] = results.get(res, 0) + 1
        waited.append(r["WAITED_MS"])
        latency.append((r["TRACE_EXIT_MONO"] - r["TRACE_ENTRY_MONO"]) / 1e6)
        if res != "VALID" or r["IMAGE_SEQ"] not in by_seq:
            continue
        _, p_ref, q_ref = by_seq[r["IMAGE_SEQ"]]
        p = (r["POSE_POSITION_ECEF_X"], r["POSE_POSITION_ECEF_Y"], r["POSE_POSITION_ECEF_Z"])
        q = (r["POSE_ATTITUDE_ECEF_FROM_CAMERA_W"], r["POSE_ATTITUDE_ECEF_FROM_CAMERA_X"],
             r["POSE_ATTITUDE_ECEF_FROM_CAMERA_Y"], r["POSE_ATTITUDE_ECEF_FROM_CAMERA_Z"])
        pos.append(math.dist(p, p_ref))
        att.append(geodesy.q_angle_deg(q, q_ref))

    seqs = [r["IMAGE_SEQ"] for r in records]
    print(f"session {session}: {len(records)} images, " + ", ".join(f"{k} {v}" for k, v in results.items()))
    print(f"in order: {seqs == sorted(seqs)}; waited median {statistics.median(waited):.0f} ms, "
          f"max {max(waited)} ms; ALIGN_APP time median {statistics.median(latency):.3f} ms, "
          f"max {max(latency):.3f} ms (ImageArrived received to ImagePose published)")
    if att:
        print(f"attitude error: median {statistics.median(att):.4f} deg, max {max(att):.4f} deg")
        print(f"position error: median {statistics.median(pos) * 1000:.2f} mm, max {max(pos) * 1000:.2f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
