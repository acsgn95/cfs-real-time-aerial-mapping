#!/usr/bin/env python3
"""Summarises GEOREF_APP in a capture of tools/ground/tlm_capture.py
(ADR-007, ADR-015 section 3).

Input: a capture with GEOREF_APP_POINTS_READY, GEOREF_APP_HK and
DENSE_APP_DEPTH_READY (CFE_EVS_LONG_EVENT_MSG for the dropped keyframes).
Reported:

* depth sets received, point sets published, keyframes dropped with their
  reasons (events of GEOREF_APP);
* per point set: points (and whether they equal the valid pixels of the
  depth set), tiles, times of the job steps, the georeferencing stage
  (DEPTH_READY to POINTS_READY) and the latency from the keyframe capture;
* the point cloud times and counters of the last housekeeping packet;
* with --laz DIR (the points/ directory of the output), the LAS headers
  of the files: files and points per keyframe against POINTS_READY, and
  the bounds. LAZ headers are not compressed, so no LAZ reader is needed.

    tools/ground/georef_report.py run.jsonl [--laz OUT/points] [--csv out.csv]

Standard library only.
"""

import argparse
import csv
import json
import re
import statistics
import struct
import sys
from pathlib import Path

LAZ_NAME = re.compile(r"points_(-?\d+)_(-?\d+)_kf(\d+)\.laz$")


def summary(label, values, unit, fmt=".1f"):
    if not values:
        return f"{label}: none"
    return (f"{label}: median {statistics.median(values):{fmt}} {unit}, mean {statistics.mean(values):{fmt}} {unit}, "
            f"max {max(values):{fmt}} {unit} (n={len(values)})")


def las_header(path):
    """Point count and bounds of a LAS 1.2 header."""
    with open(path, "rb") as f:
        h = f.read(227)
    if h[:4] != b"LASF":
        raise ValueError(f"{path}: not a LAS file")
    count = struct.unpack_from("<I", h, 107)[0]
    max_x, min_x, max_y, min_y, max_z, min_z = struct.unpack_from("<6d", h, 179)
    return count, (min_x, max_x, min_y, max_y, min_z, max_z)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("capture")
    ap.add_argument("--laz", help="points/ directory of the point cloud files")
    ap.add_argument("--csv", help="write one row per published point set")
    args = ap.parse_args(argv)

    recs = [json.loads(line) for line in open(args.capture, encoding="utf-8")]
    depth = {r["KEYFRAME_ID"]: r for r in recs if r["packet"] == "DENSE_APP_DEPTH_READY"}
    ready = [r for r in recs if r["packet"] == "GEOREF_APP_POINTS_READY"]
    hk = [r for r in recs if r["packet"] == "GEOREF_APP_HK"]
    events = [r for r in recs if r["packet"] == "CFE_EVS_LONG_EVENT_MSG"
              and r.get("PACKET_ID_APP_NAME", "").startswith("GEOREF_APP")]
    if not ready and not hk:
        print("no GEOREF_APP packets", file=sys.stderr)
        return 1

    print(f"{len(depth)} depth sets on DEPTH_READY, {len(ready)} point sets on POINTS_READY")
    for e in events:
        if e.get("PACKET_ID_EVENT_ID") in (20, 21, 23, 26):
            print(f"  event {e['PACKET_ID_EVENT_ID']}: {e.get('MESSAGE', '')}")

    rows = []
    for r in ready:
        d = depth.get(r["KEYFRAME_ID"])
        row = {
            "keyframe": r["KEYFRAME_ID"], "image": r["KEYFRAME_IMAGE_SEQ"], "points": r["POINT_COUNT"],
            "valid_pixels": d["VALID_PIXELS"] if d else None, "tiles": r["TILE_COUNT"],
            "height_min": r["HEIGHT_MIN_M"], "height_max": r["HEIGHT_MAX_M"], "wait_ms": r["WAIT_MS"],
            "allocate_ms": r["ALLOCATE_MS"], "convert_ms": r["CONVERT_MS"],
            "in_app_ms": (r["TRACE_EXIT_MONO"] - r["TRACE_ENTRY_MONO"]) / 1e6,
            "stage_ms": (r["TRACE_PUBLISH_UTC"] - d["TRACE_PUBLISH_UTC"]) / 1e6 if d else None,
            "latency_s": (r["TRACE_PUBLISH_UTC"] - r["KEYFRAME_CAPTURE_UTC"]) / 1e9,
            "point_set": r["POINT_SET_OBJECT_ID"],
        }
        rows.append(row)
    if rows:
        col = lambda k: [x[k] for x in rows if x[k] is not None]  # noqa: E731
        mismatch = [x["keyframe"] for x in rows if x["valid_pixels"] is not None and x["points"] != x["valid_pixels"]]
        print(f"points equal to the valid depth pixels: {len(rows) - len(mismatch)}/{len(rows)}"
              + (f" (differ: {mismatch})" if mismatch else ""))
        print(summary("points", [x / 1e6 for x in col("points")], "M", ".2f"))
        print(summary("tiles", col("tiles"), "", ".0f"))
        empty = [x["keyframe"] for x in rows if x["points"] == 0]
        if empty:
            print(f"published without points (no valid depth): keyframes {empty}")
        full = [x for x in rows if x["points"] > 0]
        if full:
            print(f"heights: {min(x['height_min'] for x in full):.1f} to {max(x['height_max'] for x in full):.1f} m "
                  "(ellipsoidal)")
        print(summary("waiting in the queue", col("wait_ms"), "ms", ".0f"))
        print(summary("count and allocate (with quota waits)", col("allocate_ms"), "ms", ".0f"))
        print(summary("convert (depth to P, colours, tiles)", col("convert_ms"), "ms", ".0f"))
        print(summary("in GEOREF_APP (entry to publication)", col("in_app_ms"), "ms", ".0f"))
        print(summary("georef stage (DEPTH_READY to POINTS_READY)", col("stage_ms"), "ms", ".0f"))
        print(summary("latency (capture to POINTS_READY)", col("latency_s"), "s"))
    if hk:
        h = hk[-1]
        print(f"last HK: received {h['DEPTH_SETS_RECEIVED']}, published {h['POINT_SETS_PUBLISHED']}, "
              f"failed {h['KEYFRAMES_FAILED']}, quota waits {h['QUOTA_WAITS']}, rejected {h['MESSAGES_REJECTED']}, "
              f"queue {h['QUEUE_DEPTH']}, LAZ files {h['LAZ_FILES']}, LAZ failures {h['LAZ_FAILURES']}, "
              f"max tiles {h['MAX_TILE_COUNT']}")
        print(f"point cloud: last {h['LAST_LAZ_MS']:.0f} ms, max {h['MAX_LAZ_MS']:.0f} ms; "
              f"convert max {h['MAX_CONVERT_MS']:.0f} ms; stage max {h['MAX_STAGE_MS']:.0f} ms")

    if args.laz:
        per_kf, bounds, files = {}, None, 0
        for p in sorted(Path(args.laz).glob("*.laz")):
            m = LAZ_NAME.search(p.name)
            if not m:
                continue
            count, b = las_header(p)
            files += 1
            kf = int(m.group(3))
            per_kf[kf] = per_kf.get(kf, 0) + count
            bounds = b if bounds is None else (min(bounds[0], b[0]), max(bounds[1], b[1]), min(bounds[2], b[2]),
                                               max(bounds[3], b[3]), min(bounds[4], b[4]), max(bounds[5], b[5]))
        print(f"LAZ: {files} files, {len(per_kf)} keyframes, {sum(per_kf.values()) / 1e6:.1f} M points")
        if bounds:
            print(f"LAZ bounds: E {bounds[0]:.1f}-{bounds[1]:.1f}, N {bounds[2]:.1f}-{bounds[3]:.1f}, "
                  f"heights {bounds[4]:.1f}-{bounds[5]:.1f} m (vertical datum of the files)")
        published = {x["keyframe"]: x["points"] for x in rows}
        differ = [k for k in published if published[k] > 0 and per_kf.get(k) != published[k]]
        print(f"keyframes whose files hold all points of POINTS_READY: {len(published) - len(differ)}/{len(published)}"
              + (f" (differ: {differ})" if differ else ""))
        for x in rows:
            x["laz_points"] = per_kf.get(x["keyframe"], 0)

    if args.csv and rows:
        keys = list(dict.fromkeys(k for x in rows for k in x))
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
