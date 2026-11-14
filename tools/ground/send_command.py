#!/usr/bin/env python3
"""Sends a command without parameters to the mission through CI_LAB.

For scripted runs without COSMOS, for example the end of a test flight:

    tools/ground/send_command.py MAPPROD_APP_CMD_FINALISE_ALL

The stream identifier and function code are read from the generated COSMOS
plugin definitions (tools/ground/generate_cosmos_plugin.py), so the tool
follows the EDS. Commands with parameters are not supported; use COSMOS.

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
PARAM_RE = re.compile(r"^\s*APPEND_(ID_)?PARAMETER\s+(\S+)\s+(\d+)\s+(.*)$")


def find_command(directory, name):
    """Stream identifier, function code and parameter bits of a command."""
    for path in sorted(Path(directory).glob("*_cmd_def.txt")):
        current = None
        fields = {}
        bits = 0
        for line in path.read_text().splitlines():
            m = COMMAND_RE.match(line)
            if m:
                if current == name:
                    break
                current, fields, bits = m.group(1), {}, 0
                continue
            m = PARAM_RE.match(line)
            if m and current == name:
                rest = m.group(4).split()
                try:  # integer parameters: type, minimum, maximum, default
                    fields[m.group(2)] = int(rest[3], 0)
                except (IndexError, ValueError):
                    fields[m.group(2)] = None
                bits += int(m.group(3))
        if current == name:
            return fields, bits
    return None, 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("command", help="for example MAPPROD_APP_CMD_FINALISE_ALL")
    ap.add_argument("--ci", default="127.0.0.1:1244", help="CI_LAB command address")
    ap.add_argument("--defs", default=str(DEFAULT_DEFS))
    args = ap.parse_args(argv)

    fields, bits = find_command(args.defs, args.command)
    if fields is None:
        print(f"command {args.command} not found in {args.defs}", file=sys.stderr)
        return 1
    if bits != 64:  # primary header (48 bits), function code and checksum
        print(f"{args.command} has parameters; use COSMOS", file=sys.stderr)
        return 1
    msg = bytearray(struct.pack(">HHHBB", fields["CCSDS_STREAMID"], 0xC000, 1, fields["CCSDS_FC"], 0))
    # cFE command checksum: the XOR of all bytes is 0xFF.
    x = 0
    for b in msg:
        x ^= b
    msg[7] = x ^ 0xFF
    host, port = args.ci.rsplit(":", 1)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(bytes(msg), (host, int(port)))
    print(f"sent {args.command} (stream 0x{fields['CCSDS_STREAMID']:04X}, function code {fields['CCSDS_FC']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
