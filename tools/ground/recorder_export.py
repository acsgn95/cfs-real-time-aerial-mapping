#!/usr/bin/env python3
"""Turns a RECORDER_APP run into the files of ADR-017 section 4.

Input: a run directory <OutputDir>/<run id>/ of RECORDER_APP with
recording.ostrec and manifest.json. Output (default: the run directory):

* csv/<PACKET>.csv: one file per message, rx_utc_ns (receive time at the
  recorder) and the decoded fields;
* events.log: the EVS event messages, one line each;
* recording.jsonl: every message as one JSON object, the format of
  tools/ground/tlm_capture.py, so that the report tools (dense_report.py,
  georef_report.py, mapprod_report.py) read a recording as they read a
  capture;
* a summary on standard output: messages per type, the recorder's counts
  from the manifest, and whether the file ends cleanly.

Messages are decoded with the definitions of the generated COSMOS plugin
(tools/ground/generate_cosmos_plugin.py), as tlm_capture.py decodes them;
the recording holds the bytes TO_LAB would send.

    tools/ground/recorder_export.py RUN_DIR [--out DIR] [--defs DIR]

Standard library only.
"""

import argparse
import csv
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tlm_capture import DEFAULT_DEFS, load_definitions  # noqa: E402

MAGIC = b"OSTREC\0\0"
EVENT_TYPES = {1: "DEBUG", 2: "INFORMATION", 3: "ERROR", 4: "CRITICAL"}


def read_recording(path):
    """Header fields and (rx_utc_ns, message bytes) records; a truncated
    last record (the process was killed) is reported, not raised."""
    data = Path(path).read_bytes()
    if len(data) < 64 or data[:8] != MAGIC:
        raise ValueError(f"{path}: not a RECORDER_APP recording")
    header_size, fmt = struct.unpack_from("<II", data, 8)
    if fmt != 1:
        raise ValueError(f"{path}: recording format {fmt} not supported")
    start_utc = struct.unpack_from("<q", data, 16)[0]
    run_id = data[24:56].split(b"\0", 1)[0].decode("ascii", "replace")
    records, offset, truncated = [], header_size, False
    while offset < len(data):
        if offset + 12 > len(data):
            truncated = True
            break
        size, rx = struct.unpack_from("<Iq", data, offset)
        if offset + 12 + size > len(data):
            truncated = True
            break
        records.append((rx, data[offset + 12:offset + 12 + size]))
        offset += 12 + size
    return {"run_id": run_id, "start_utc_ns": start_utc}, records, truncated


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--out", help="output directory (default: the run directory)")
    ap.add_argument("--defs", default=str(DEFAULT_DEFS), help="generated COSMOS cmd_tlm directory")
    args = ap.parse_args(argv)

    run = Path(args.run_dir)
    out = Path(args.out) if args.out else run
    defs = load_definitions(args.defs)
    if not defs:
        print(f"no packet definitions in {args.defs}: run tools/ground/generate_cosmos_plugin.py", file=sys.stderr)
        return 1
    header, records, truncated = read_recording(run / "recording.ostrec")
    manifest = json.loads((run / "manifest.json").read_text()) if (run / "manifest.json").exists() else {}

    (out / "csv").mkdir(parents=True, exist_ok=True)
    writers, files, counts, unknown = {}, [], {}, {}
    with open(out / "recording.jsonl", "w", encoding="utf-8") as jsonl, \
            open(out / "events.log", "w", encoding="utf-8") as events:
        for rx, msg in records:
            mid = struct.unpack_from(">H", msg, 0)[0] if len(msg) >= 2 else None
            packet = defs.get(mid)
            if packet is None:
                unknown[mid] = unknown.get(mid, 0) + 1
                continue
            record = {"packet": packet.name, "rx_utc_ns": rx}
            record.update(packet.decode(msg))
            jsonl.write(json.dumps(record) + "\n")
            counts[packet.name] = counts.get(packet.name, 0) + 1

            row = {k: (json.dumps(v) if isinstance(v, list) else v) for k, v in record.items() if k != "packet"}
            w = writers.get(packet.name)
            if w is None:
                f = open(out / "csv" / f"{packet.name}.csv", "w", newline="", encoding="utf-8")
                files.append(f)
                w = csv.DictWriter(f, fieldnames=list(row), extrasaction="ignore")
                w.writeheader()
                writers[packet.name] = w
            w.writerow(row)

            if packet.name == "CFE_EVS_LONG_EVENT_MSG":
                events.write(f"{rx} {record.get('PACKET_ID_APP_NAME', '?')} {record.get('PACKET_ID_EVENT_ID', '?')} "
                             f"{EVENT_TYPES.get(record.get('PACKET_ID_EVENT_TYPE'), '?')}: "
                             f"{record.get('MESSAGE', '')}\n")
    for f in files:
        f.close()

    span = (records[-1][0] - records[0][0]) / 1e9 if len(records) > 1 else 0.0
    print(f"run {header['run_id']}: {len(records)} records over {span:.1f} s, manifest state "
          f"{manifest.get('state', '?')}" + (", last record truncated" if truncated else ""))
    if manifest:
        print(f"recorder: {manifest.get('records')} records, {manifest.get('dropped', 0)} dropped, "
              f"{manifest.get('encode_errors', 0)} without EDS, {manifest.get('write_errors', 0)} write errors, "
              f"longest write {manifest.get('max_write_ms', 0)} ms")
        if manifest.get("state") == "stopped" and manifest.get("records") != len(records):
            print(f"warning: the manifest counts {manifest.get('records')} records, the file holds {len(records)}")
    for name, n in sorted(counts.items()):
        print(f"  {name}: {n}")
    for mid, n in sorted(unknown.items(), key=lambda kv: kv[0] or 0):
        print(f"  unknown message 0x{mid:04X}: {n}" if mid is not None else f"  short record: {n}")
    print(f"written: {out / 'csv'}/, {out / 'events.log'}, {out / 'recording.jsonl'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
