#!/usr/bin/env python3
"""Summarises MAPPROD_APP in a capture of tools/ground/tlm_capture.py
(ADR-007, ADR-001 R2).

Input: a capture with MAPPROD_APP_TILE_UPDATED, MAPPROD_APP_TILE_PUBLISHED,
MAPPROD_APP_HK and GEOREF_APP_POINTS_READY. Reported:

* point sets received, keyframes in the map, keyframes dropped;
* per keyframe: points fused and rejected, orthomosaic pixels, tiles, the
  times of the job steps, the map product stage (POINTS_READY to
  TILE_UPDATED) and the map update latency from the keyframe capture
  (ADR-001 R2);
* publications: tiles written, write time and publication delay;
* the last housekeeping packet: tiles, memory, failures.

    tools/ground/mapprod_report.py run.jsonl [--csv out.csv]

Standard library only.
"""

import argparse
import csv
import json
import statistics
import sys

REASONS = {1: "PERIODIC", 2: "FINALISE_ALL", 3: "STOPPED"}


def summary(label, values, unit, fmt=".1f"):
    if not values:
        return f"{label}: none"
    return (f"{label}: median {statistics.median(values):{fmt}} {unit}, mean {statistics.mean(values):{fmt}} {unit}, "
            f"max {max(values):{fmt}} {unit} (n={len(values)})")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("capture")
    ap.add_argument("--csv", help="write one row per keyframe in the map")
    args = ap.parse_args(argv)

    recs = [json.loads(line) for line in open(args.capture, encoding="utf-8")]
    points = {r["KEYFRAME_ID"]: r for r in recs if r["packet"] == "GEOREF_APP_POINTS_READY"}
    updated = [r for r in recs if r["packet"] == "MAPPROD_APP_TILE_UPDATED"]
    published = [r for r in recs if r["packet"] == "MAPPROD_APP_TILE_PUBLISHED"]
    hk = [r for r in recs if r["packet"] == "MAPPROD_APP_HK"]
    events = [r for r in recs if r["packet"] == "CFE_EVS_LONG_EVENT_MSG"
              and r.get("PACKET_ID_APP_NAME", "").startswith("MAPPROD_APP")]
    if not updated and not hk:
        print("no MAPPROD_APP packets", file=sys.stderr)
        return 1

    print(f"{len(points)} point sets on POINTS_READY, {len(updated)} keyframes on TILE_UPDATED")
    for e in events:
        if e.get("PACKET_ID_EVENT_ID") in (11, 20, 22, 25, 26, 29, 30):
            print(f"  event {e['PACKET_ID_EVENT_ID']}: {e.get('MESSAGE', '')}")

    rows = []
    for u in updated:
        g = points.get(u["KEYFRAME_ID"])
        rows.append({
            "keyframe": u["KEYFRAME_ID"], "image": u["IMAGE_SEQ"], "fused": u["POINTS_FUSED"],
            "rejected": u["POINTS_REJECTED"], "ortho": u["ORTHO"], "ortho_pixels": u["ORTHO_PIXELS"],
            "tiles": u["TILE_COUNT"], "active_tiles": u["ACTIVE_TILES"], "finalised": u["TILES_FINALISED"],
            "memory_mb": u["MEMORY_MB"], "wait_ms": u["WAIT_MS"], "fuse_ms": u["FUSE_MS"],
            "decode_ms": u["DECODE_MS"], "render_ms": u["RENDER_MS"], "lifecycle_ms": u["LIFECYCLE_MS"],
            "in_app_ms": (u["TRACE_EXIT_MONO"] - u["TRACE_ENTRY_MONO"]) / 1e6,
            "stage_ms": (u["TRACE_PUBLISH_UTC"] - g["TRACE_PUBLISH_UTC"]) / 1e6 if g else None,
            "latency_s": u["LATENCY_S"],
        })
    if rows:
        col = lambda k: [x[k] for x in rows if x[k] is not None]  # noqa: E731
        with_points = [x for x in rows if x["fused"] > 0]
        print(f"keyframes with points: {len(with_points)}, in the orthomosaic: {sum(x['ortho'] for x in rows)}")
        print(summary("points fused", [x["fused"] / 1e6 for x in with_points], "M", ".2f"))
        print(summary("rejected (k sigma)", [100 * x["rejected"] / max(1, x["fused"] + x["rejected"])
                                             for x in with_points], "%", ".2f"))
        print(summary("orthomosaic pixels", [x["ortho_pixels"] / 1e6 for x in with_points], "M", ".2f"))
        print(summary("tiles changed", col("tiles"), "", ".0f"))
        print(summary("waiting in the queue", col("wait_ms"), "ms", ".0f"))
        print(summary("fusion", [x["fuse_ms"] for x in with_points], "ms", ".0f"))
        print(summary("JPEG decoding", [x["decode_ms"] for x in rows if x["ortho"]], "ms", ".0f"))
        print(summary("rendering", [x["render_ms"] for x in rows if x["ortho"]], "ms", ".0f"))
        print(summary("lifecycle (finalising)", col("lifecycle_ms"), "ms", ".0f"))
        print(summary("in MAPPROD_APP (entry to TILE_UPDATED)", col("in_app_ms"), "ms", ".0f"))
        print(summary("map stage (POINTS_READY to TILE_UPDATED)", col("stage_ms"), "ms", ".0f"))
        print(summary("map update latency (capture to TILE_UPDATED, R2)", col("latency_s"), "s"))
        print(f"active tiles: max {max(col('active_tiles'))}, memory max {max(col('memory_mb')):.0f} MB, "
              f"tiles finalised by distance or budget: {sum(col('finalised'))}")
    if published:
        by_reason = {}
        for p in published:
            by_reason.setdefault(REASONS.get(p["REASON"], p["REASON"]), []).append(p)
        for reason, ps in by_reason.items():
            print(f"publications {reason}: {len(ps)}, tiles {sum(p['TILES_WRITTEN'] for p in ps)}")
            print("  " + summary("write time", [p["WRITE_MS"] for p in ps], "ms", ".0f"))
            print("  " + summary("tiles per publication", [p["TILES_WRITTEN"] for p in ps], "", ".1f"))
            if reason == "PERIODIC":
                print("  " + summary("publication delay (oldest update)", [p["PENDING_S"] for p in ps], "s"))
    if hk:
        h = hk[-1]
        print(f"last HK: received {h['POINT_SETS_RECEIVED']}, updated {h['KEYFRAMES_UPDATED']}, "
              f"failed {h['KEYFRAMES_FAILED']}, ortho skipped {h['ORTHO_SKIPPED']}, rejected messages "
              f"{h['MESSAGES_REJECTED']}, active tiles {h['ACTIVE_TILES']}, finalised {h['TILES_FINALISED']}, "
              f"publications {h['PUBLICATIONS']}, write failures {h['TILE_WRITE_FAILURES']}, "
              f"memory {h['MEMORY_MB']:.0f} MB")

    if args.csv and rows:
        keys = list(dict.fromkeys(k for x in rows for k in x))
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
