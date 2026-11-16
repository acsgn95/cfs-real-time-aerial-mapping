#!/usr/bin/env python3
"""Sends a command to the mission through CI_LAB, without COSMOS.

For scripted runs, for example the start and the end of a test flight:

    tools/ground/send_command.py RECORDER_APP_CMD_START_RECORDING --param RUN_ID=flight_01
    tools/ground/send_command.py MAPPROD_APP_CMD_FINALISE_ALL

The layout (stream identifier, function code, parameters) is read from the
generated COSMOS plugin definitions (tools/ground/generate_cosmos_plugin.py),
so the tool follows the EDS. Parameters that are not given keep their
default; integer, float and string parameters are supported.

Standard library only.
"""

import argparse
import re
import socket
import struct
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DEFAULT_DEFS = REPO / "build/cosmos_plugin/openc3-cosmos-orthostream/targets/ORTHOSTREAM/cmd_tlm"

COMMAND_RE = re.compile(r"^COMMAND\s+\S+\s+(\S+)")
PARAM_RE = re.compile(r"^\s*APPEND_(ID_)?PARAMETER\s+(\S+)\s+(\d+)\s+(\S+)\s*(.*)$")


def find_command(directory, name):
    """Parameters of a command in order: (name, bits, type, default)."""
    for path in sorted(Path(directory).glob("*_cmd_def.txt")):
        current, params = None, []
        for line in path.read_text().splitlines():
            m = COMMAND_RE.match(line)
            if m:
                if current == name:
                    break
                current, params = m.group(1), []
                continue
            m = PARAM_RE.match(line)
            if m and current == name:
                kind, rest = m.group(4), m.group(5).split()
                default = None
                if kind in ("UINT", "INT") and len(rest) >= 3:
                    default = int(rest[2], 0)
                elif kind == "FLOAT" and len(rest) >= 3:
                    default = float(rest[2])
                elif kind == "STRING":
                    default = ""
                params.append((m.group(2), int(m.group(3)), kind, default))
        if current == name:
            return params
    return None


def pack(params, values):
    out = bytearray()
    for name, bits, kind, default in params:
        value = values.pop(name, default)
        if kind == "UINT":
            out += int(value).to_bytes(bits // 8, "big", signed=False)
        elif kind == "INT":
            out += int(value).to_bytes(bits // 8, "big", signed=True)
        elif kind == "FLOAT":
            out += struct.pack(">f" if bits == 32 else ">d", float(value))
        elif kind == "STRING":
            raw = str(value).encode("ascii")
            if len(raw) > bits // 8:
                raise ValueError(f"{name}: at most {bits // 8} characters")
            out += raw.ljust(bits // 8, b"\0")
        else:
            raise ValueError(f"{name}: type {kind} not supported")
    if values:
        raise ValueError(f"unknown parameters: {', '.join(values)}")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("command", help="for example MAPPROD_APP_CMD_FINALISE_ALL")
    ap.add_argument("--param", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--ci", default="127.0.0.1:1244", help="CI_LAB command address")
    ap.add_argument("--defs", default=str(DEFAULT_DEFS))
    args = ap.parse_args(argv)

    params = find_command(args.defs, args.command)
    if params is None:
        print(f"command {args.command} not found in {args.defs}", file=sys.stderr)
        return 1
    values = dict(p.split("=", 1) for p in args.param)
    for fixed in ("CCSDS_STREAMID", "CCSDS_SEQUENCE", "CCSDS_LENGTH", "CCSDS_FC", "CCSDS_CHECKSUM"):
        if fixed in values:
            print(f"{fixed} is set by the tool", file=sys.stderr)
            return 1
    try:
        msg = pack(params, values)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1
    struct.pack_into(">H", msg, 4, len(msg) - 7)  # CCSDS length: bytes after the primary header - 1
    msg[7] = 0
    x = 0
    for b in msg:  # cFE command checksum: the XOR of all bytes is 0xFF
        x ^= b
    msg[7] = x ^ 0xFF
    host, port = args.ci.rsplit(":", 1)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(bytes(msg), (host, int(port)))
    fields = {name: default for name, _, _, default in params}
    print(f"sent {args.command} (stream 0x{fields['CCSDS_STREAMID']:04X}, function code {fields['CCSDS_FC']}, "
          f"{len(msg)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
