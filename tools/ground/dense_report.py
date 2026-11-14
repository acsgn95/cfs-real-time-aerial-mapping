#!/usr/bin/env python3
"""Summarises DENSE_APP in a capture of tools/ground/tlm_capture.py
(ADR-005, ADR-015 section 3).

Input: a capture with DENSE_APP_DENSE_STATUS, DENSE_APP_DEPTH_READY and
OST_BATCH_READY. Reported:

* the outcome of every batch (done, skipped, failed) with its reason;
* per published depth set: valid and gated share, times of the job steps,
  waiting in the queue, the dense stage (BATCH_READY to DEPTH_READY) and
  the latency from the keyframe capture;
* with --offline, the valid share and the estimation time of the same
  keyframe images in a tools/offline_pipeline run (timings.csv).

    tools/ground/dense_report.py run.jsonl [--offline DIR/timings.csv] [--csv out.csv]

Standard library only.
"""

import argparse
import csv
import json
import re
import statistics
import sys

OUTCOMES = {1: "DONE", 2: "SKIPPED", 3: "FAILED"}
REASONS = {0: "NONE", 1: "NEWER_BATCH", 2: "QUOTA", 3: "NOT_READY", 4: "INVALID_BATCH",
           5: "ESTIMATION", 6: "GPU", 7: "NOT_SENT", 8: "STOPPED", 9: "NO_SOURCES"}


def summary(label, values, unit, fmt=".1f"):
    if not values:
        return f"{label}: none"
    return (f"{label}: median {statistics.median(values):{fmt}} {unit}, mean {statistics.mean(values):{fmt}} {unit}, "
            f"max {max(values):{fmt}} {unit} (n={len(values)})")


def image_number(name):
    m = re.search(r"(\d+)\.\w+$", name)
    return int(m.group(1)) if m else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("capture")
    ap.add_argument("--offline", help="timings.csv of a tools/offline_pipeline run")
    ap.add_argument("--csv", help="write one row per published depth set")
    args = ap.parse_args(argv)

    recs = [json.loads(line) for line in open(args.capture, encoding="utf-8")]
    status = [r for r in recs if r["packet"] == "DENSE_APP_DENSE_STATUS"]
    ready = [r for r in recs if r["packet"] == "DENSE_APP_DEPTH_READY"]
    batch_pub = {r["KEYFRAME_ID"]: r["TRACE_PUBLISH_UTC"] for r in recs if r["packet"] == "OST_BATCH_READY"}
    if not status:
        print("no DENSE_APP_DENSE_STATUS packets", file=sys.stderr)
        return 1

    counts = {}
    for r in status:
        key = (OUTCOMES.get(r["OUTCOME"], r["OUTCOME"]), REASONS.get(r["REASON"], r["REASON"]))
        counts[key] = counts.get(key, 0) + 1
    print(f"{len(batch_pub)} batches on BATCH_READY, {len(status)} outcomes: "
          + ", ".join(f"{o}{'' if r == 'NONE' else ' ' + r} {n}" for (o, r), n in sorted(counts.items())))
    skipped = [r["IMAGE_SEQ"] for r in status if r["OUTCOME"] != 1]
    if skipped:
        print(f"keyframe images not done: {skipped}")

    rows = []
    for r in ready:
        pixels = r["INTRINSICS_WIDTH"] * r["INTRINSICS_HEIGHT"]
        row = {
            "keyframe": r["KEYFRAME_ID"], "image": r["KEYFRAME_IMAGE_SEQ"], "partner": r["PARTNER_IMAGE_SEQ"],
            "pose_source": r["POSE_SOURCE"], "valid": r["VALID_PIXELS"] / pixels, "gated": r["GATED_PIXELS"] / pixels,
            "depth_min": r["DEPTH_MIN_M"], "depth_max": r["DEPTH_MAX_M"], "wait_ms": r["WAIT_MS"],
            "read_ms": r["READ_MS"], "setup_ms": r["SETUP_MS"], "estimate_ms": r["ESTIMATE_MS"], "gate_ms": r["GATE_MS"],
            "in_app_ms": (r["TRACE_EXIT_MONO"] - r["TRACE_ENTRY_MONO"]) / 1e6,
            "stage_ms": (r["TRACE_PUBLISH_UTC"] - batch_pub[r["KEYFRAME_ID"]]) / 1e6
            if r["KEYFRAME_ID"] in batch_pub else None,
            "latency_s": (r["TRACE_PUBLISH_UTC"] - r["KEYFRAME_CAPTURE_UTC"]) / 1e9,
        }
        rows.append(row)
    if not rows:
        print("no DENSE_APP_DEPTH_READY packets")
        return 0
    col = lambda k: [x[k] for x in rows if x[k] is not None]  # noqa: E731
    print(summary("valid share", [100 * v for v in col("valid")], "%"))
    print(summary("gated share", [100 * v for v in col("gated")], "%"))
    print(summary("estimate", col("estimate_ms"), "ms", ".0f"))
    print(summary("read (working images to grey)", col("read_ms"), "ms", ".0f"))
    print(summary("setup (frame, terrain grid, range)", col("setup_ms"), "ms", ".0f"))
    print(summary("terrain gate", col("gate_ms"), "ms", ".0f"))
    print(summary("waiting in the queue", col("wait_ms"), "ms", ".0f"))
    print(summary("in DENSE_APP (entry to publication)", col("in_app_ms"), "ms", ".0f"))
    print(summary("dense stage (BATCH_READY to DEPTH_READY)", col("stage_ms"), "ms", ".0f"))
    print(summary("latency (capture to DEPTH_READY)", col("latency_s"), "s"))

    if args.offline:
        offline = {}
        for o in csv.DictReader(open(args.offline, encoding="utf-8")):
            offline[image_number(o["image"])] = o
        pairs = [(x, offline[x["image"]]) for x in rows if x["image"] in offline]
        print(f"offline run: {len(offline)} keyframes, {len(pairs)} in common")
        if pairs:
            dv = [100 * (x["valid"] - float(o["valid_fraction"])) for x, o in pairs]
            ratio = [x["estimate_ms"] / (1e3 * float(o["dense_s"])) for x, o in pairs if float(o["dense_s"]) > 0]
            print(summary("valid share, mission - offline", dv, "points"))
            print(summary("estimate time, mission / offline", ratio, "", ".2f"))
            for x, o in pairs:
                x["offline_valid"] = float(o["valid_fraction"])
                x["offline_dense_ms"] = 1e3 * float(o["dense_s"])

    if args.csv:
        keys = list(dict.fromkeys(k for x in rows for k in x))
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
