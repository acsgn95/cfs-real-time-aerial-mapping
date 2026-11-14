#!/usr/bin/env python3
"""Records and decodes the TO_LAB telemetry of the mission, without COSMOS.

For tests and measurements that need every packet of a run (COSMOS keeps
them too, but reading them back is slower than this). TO_LAB sends to one
address only: this tool redirects it to a second loopback address
(default 127.0.0.2, port 2244), so it does not compete with COSMOS for
127.0.0.1:2244, records, and on exit points TO_LAB back to 127.0.0.1.
COSMOS receives nothing while the capture runs.

Packets are decoded with the definitions of the generated COSMOS plugin
(tools/ground/generate_cosmos_plugin.py), so the tool follows the EDS
without its own copy of the layouts. Output: one JSON object per packet,
{"packet": name, "rx_utc_ns": ..., fields...}.

    tools/ground/tlm_capture.py --out run.jsonl --packets ALIGN_APP_IMAGE_POSE INGEST_APP_HK
    (stop with Ctrl-C, or --duration)

Standard library only.
"""

import argparse
import json
import re
import signal
import socket
import struct
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DEFAULT_DEFS = REPO / "build/cosmos_plugin/openc3-cosmos-orthostream/targets/ORTHOSTREAM/cmd_tlm"

TO_LAB_CMD_MID = 0x1880
TO_LAB_ENABLE_OUTPUT_FC = 6

ITEM_RE = re.compile(r'^\s*APPEND_(ID_)?ITEM\s+(\S+)\s+(\d+)\s+(\S+)')
ARRAY_RE = re.compile(r'^\s*APPEND_ARRAY_ITEM\s+(\S+)\s+(\d+)\s+(\S+)\s+(\d+)')
PACKET_RE = re.compile(r'^\s*TELEMETRY\s+\S+\s+(\S+)\s+(BIG|LITTLE)_ENDIAN')
STREAMID_RE = re.compile(r'^\s*APPEND_ID_ITEM\s+CCSDS_STREAMID\s+16\s+UINT\s+(0x[0-9A-Fa-f]+|\d+)')


class Packet:
    def __init__(self, name):
        self.name = name
        self.mid = None
        self.items = []  # (name, bits, type, count or None)

    def decode(self, data):
        out, offset = {}, 0
        for name, bits, kind, count in self.items:
            n = (count or 1)
            values = []
            for _ in range(n):
                nbytes = bits // 8
                raw = data[offset:offset + nbytes]
                offset += nbytes
                if len(raw) < nbytes:
                    return out
                if kind == "UINT":
                    values.append(int.from_bytes(raw, "big", signed=False))
                elif kind == "INT":
                    values.append(int.from_bytes(raw, "big", signed=True))
                elif kind == "FLOAT":
                    values.append(struct.unpack(">f" if bits == 32 else ">d", raw)[0])
                elif kind == "STRING":
                    values.append(raw.split(b"\0", 1)[0].decode("utf-8", "replace"))
                else:
                    values.append(raw.hex())
            out[name] = values if count else values[0]
        return out


def load_definitions(directory):
    packets = {}
    for path in sorted(Path(directory).glob("*_def.txt")):
        current = None
        for line in path.read_text(encoding="utf-8").splitlines():
            m = PACKET_RE.match(line)
            if m:
                current = Packet(m.group(1))
                packets[current.name] = current
                continue
            if line.lstrip().startswith("COMMAND"):
                current = None
                continue
            if current is None:
                continue
            m = STREAMID_RE.match(line)
            if m:
                current.mid = int(m.group(1), 0)
            m = ARRAY_RE.match(line)
            if m:
                current.items.append((m.group(1), int(m.group(2)), m.group(3), int(m.group(4)) // int(m.group(2))))
                continue
            m = ITEM_RE.match(line)
            if m:
                current.items.append((m.group(2), int(m.group(3)), m.group(4), None))
    return {p.mid: p for p in packets.values() if p.mid is not None}


def enable_output(ci_host, ci_port, dest_ip):
    payload = dest_ip.encode("ascii").ljust(16, b"\0")
    length = 2 + len(payload) - 1  # secondary header + payload - 1
    msg = bytearray(struct.pack(">HHHBB", TO_LAB_CMD_MID, 0xC000, length, TO_LAB_ENABLE_OUTPUT_FC, 0) + payload)
    # cFE command checksum: the XOR of all bytes is 0xFF.
    x = 0
    for b in msg:
        x ^= b
    msg[7] = x ^ 0xFF
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(msg, (ci_host, ci_port))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True, help="JSON lines file")
    ap.add_argument("--packets", nargs="*", default=None, help="packet names to keep (default: all)")
    ap.add_argument("--listen", default="127.0.0.2", help="address TO_LAB is pointed to")
    ap.add_argument("--port", type=int, default=2244, help="TO_LAB telemetry port")
    ap.add_argument("--ci", default="127.0.0.1:1244", help="CI_LAB command address")
    ap.add_argument("--restore", default="127.0.0.1", help="TO_LAB destination set on exit; empty: leave")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0: until Ctrl-C or SIGTERM")
    ap.add_argument("--defs", default=str(DEFAULT_DEFS), help="generated COSMOS cmd_tlm directory")
    args = ap.parse_args(argv)

    defs = load_definitions(args.defs)
    if not defs:
        print(f"no packet definitions in {args.defs}: run scripts/cosmos_plugin.sh or "
              "tools/ground/generate_cosmos_plugin.py", file=sys.stderr)
        return 1
    keep = set(args.packets) if args.packets else None
    ci_host, ci_port = args.ci.rsplit(":", 1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen, args.port))
    sock.settimeout(0.2)
    enable_output(ci_host, int(ci_port), args.listen)

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    counts = {}
    end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        with open(args.out, "w", encoding="utf-8") as out:
            while not stop["flag"] and (end is None or time.monotonic() < end):
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    continue
                if len(data) < 2:
                    continue
                mid = int.from_bytes(data[0:2], "big")
                packet = defs.get(mid)
                name = packet.name if packet else f"MID_0x{mid:04X}"
                counts[name] = counts.get(name, 0) + 1
                if keep is not None and name not in keep:
                    continue
                record = {"packet": name, "rx_utc_ns": time.time_ns()}
                if packet:
                    record.update(packet.decode(data))
                out.write(json.dumps(record) + "\n")
                out.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if args.restore:
            enable_output(ci_host, int(ci_port), args.restore)
    print(", ".join(f"{k} {v}" for k, v in sorted(counts.items())) or "no packets")
    return 0


if __name__ == "__main__":
    sys.exit(main())
