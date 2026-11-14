#!/usr/bin/env python3
"""Generates the OrthoStream OpenC3 COSMOS plugin (ADR-021 section 1).

Inputs:
* ground/cosmos/plugin/          template: plugin.txt, gemspec, Rakefile, target.txt
* build/cosmos/*_def.txt          packet definitions written by the EDS build
* mission/orthostream_defs/eds/cfe-topicids.xml   topic identifiers
* apps/*/eds/*.xml, the cFE EDS files            enumerations
* vendor/cfs/tools/cfs-cosmos-plugin             CCSDS protocols and the
                                                  time conversion (Apache-2.0)

Output: build/cosmos_plugin/openc3-cosmos-orthostream, ready for
`openc3.sh cli rake build`.

What the generator adds to the EDS output:
* The packet headers. The EDS output calls the helpers of the cFS plugin,
  which look message identifiers up in a fixed table of the cFE and lab
  applications. Here the identifiers are computed from the topic
  identifiers instead, as cFE does: command 0x1800 + offset, telemetry
  0x0800 + offset, offset = topic - base (checked against the built SCH_LAB
  and TO_LAB tables, 2026-11-14). Packets are big-endian, as TO_LAB encodes
  them through EDS; the telemetry header has 4 bytes of alignment after the
  time.
* STATES for items whose EDS type is an enumeration (open question of
  ADR-020): the cFS COSMOS writer emits them as plain integers.
* One housekeeping screen per application and a fault event screen.

Standard library only.
"""

import argparse
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SEDS = "{http://www.ccsds.org/schema/sois/seds}"

# Prefix of the generated file names -> EDS package and topic prefix.
PACKAGES = {
    "cfe_es": ("CFE_ES", "ES"),
    "cfe_evs": ("CFE_EVS", "EVS"),
    "cfe_sb": ("CFE_SB", "SB"),
    "cfe_tbl": ("CFE_TBL", "TBL"),
    "cfe_time": ("CFE_TIME", "TIME"),
    "to_lab": ("TO_LAB", "TO_LAB"),
    "ci_lab": ("CI_LAB", "CI_LAB"),
    "ost": ("OST", "OST"),
}
# OrthoStream applications: every apps/<app>/eds/<app>.xml.

CMD_BASE_MID = 0x1800
TLM_BASE_MID = 0x0800


def snake(name):
    s = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name)
    s = re.sub(r"(?<=[A-Za-z])(?=[0-9])", "_", s)
    return s.upper()


def topic_offsets(path):
    """Topic name -> offset within the command or telemetry block."""
    offsets = {}
    for d in ET.parse(path).getroot().iter():
        if not d.tag.endswith("Define"):
            continue
        m = re.fullmatch(r"\$\{CFE_MISSION/(TELECOMMAND|TELEMETRY)_BASE_TOPICID\}\s*\+\s*(\d+)",
                         d.get("value", "").strip())
        if m:
            offsets[d.get("name")] = int(m.group(2))
    return offsets


class EdsTypes:
    """Containers and enumerations of all EDS packages, for STATES."""

    def __init__(self, files):
        self.containers = {}  # "PKG/Name" -> [(entry name, type ref)]
        self.enums = {}  # "PKG/Name" -> [(label, value)]
        self.interfaces = {}  # "PKG/INTF" -> data type ref of the message
        for f in files:
            root = ET.parse(f).getroot()
            for pkg in root.iter(SEDS + "Package"):
                p = pkg.get("name")
                for c in pkg.iter(SEDS + "ContainerDataType"):
                    entries = [(e.get("name"), e.get("type")) for e in c.iter(SEDS + "Entry")]
                    self.containers[f"{p}/{c.get('name')}"] = (entries, c.get("baseType"))
                for e in pkg.iter(SEDS + "EnumeratedDataType"):
                    self.enums[f"{p}/{e.get('name')}"] = [(x.get("label"), int(x.get("value")))
                                                         for x in e.iter(SEDS + "Enumeration")]
                for i in pkg.iter(SEDS + "Interface"):
                    for g in i.iter(SEDS + "GenericTypeMap"):
                        self.interfaces[f"{p}/{i.get('name')}"] = self.ref(p, g.get("type"))

    @staticmethod
    def ref(pkg, t):
        return t if "/" in t else f"{pkg}/{t}"

    def flatten(self, type_ref, prefix=""):
        """Yields (ITEM_NAME, enum type ref) for enumerations inside a container."""
        if type_ref not in self.containers:
            return
        entries, base = self.containers[type_ref]
        pkg = type_ref.split("/")[0]
        if base and not base.startswith("CFE_HDR/"):
            yield from self.flatten(self.ref(pkg, base), prefix)
        for name, t in entries:
            if t is None:
                continue
            r = self.ref(pkg, t)
            item = f"{prefix}{snake(name)}"
            if r in self.enums:
                yield item, r
            elif r in self.containers:
                # The payload container itself does not add a prefix.
                yield from self.flatten(r, "" if name == "Payload" else item + "_")


def header_cmd(name, mid, fc, desc):
    return (f'COMMAND ORTHOSTREAM {name} BIG_ENDIAN "{desc}"\n'
            f'   APPEND_ID_PARAMETER CCSDS_STREAMID 16 UINT MIN_UINT16 MAX_UINT16 0x{mid:04X} "CCSDS Packet Identification"\n'
            f'      FORMAT_STRING "0x%04X"\n'
            f'   APPEND_PARAMETER CCSDS_SEQUENCE 16 UINT MIN_UINT16 MAX_UINT16 0xC000 "CCSDS Packet Sequence Control"\n'
            f'      FORMAT_STRING "0x%04X"\n'
            f'   APPEND_PARAMETER CCSDS_LENGTH 16 UINT MIN_UINT16 MAX_UINT16 1 "CCSDS Packet Data Length"\n'
            f'   APPEND_ID_PARAMETER CCSDS_FC 8 UINT MIN_UINT8 MAX_UINT8 {fc} "Command Function Code"\n'
            f'   APPEND_PARAMETER CCSDS_CHECKSUM 8 UINT MIN_UINT8 MAX_UINT8 0 "Command Checksum"\n')


def header_tlm(name, mid, desc):
    return (f'TELEMETRY ORTHOSTREAM {name} BIG_ENDIAN "{desc}"\n'
            f'   APPEND_ID_ITEM CCSDS_STREAMID 16 UINT 0x{mid:04X} "CCSDS Packet Identification"\n'
            f'      FORMAT_STRING "0x%04X"\n'
            f'   APPEND_ITEM CCSDS_SEQUENCE 16 UINT "CCSDS Packet Sequence Control"\n'
            f'      FORMAT_STRING "0x%04X"\n'
            f'   APPEND_ITEM CCSDS_LENGTH 16 UINT "CCSDS Packet Data Length"\n'
            f'   APPEND_ITEM SECONDS 32 UINT "Secondary header, seconds"\n'
            f'   APPEND_ITEM SUBSECS 16 UINT "Secondary header, subseconds"\n'
            f'   APPEND_ITEM SPARE_FOR_64_ALIGN 32 UINT "Alignment of the payload"\n'
            f'   ITEM PACKET_TIME 0 0 DERIVED "Time from SECONDS and SUBSECS"\n'
            f'      READ_CONVERSION unix_time_conversion_epoch_offset.rb SECONDS SUBSECS\n')


CMD_RE = re.compile(r"<%=\s*cfs_cmd_hdr\(target_name,\s*'(\w+)',\s*(\d+),\s*\"([^\"]*)\"\)\s*%>")
TLM_RE = re.compile(r"<%=\s*cfs_tlm_hdr\(target_name,\s*'(\w+)',\s*\"([^\"]*)\"\)\s*%>")


def add_states(text, states):
    """Appends STATE lines after the items that are enumerations."""
    out = []
    for line in text.splitlines():
        out.append(line)
        m = re.match(r"\s*APPEND_(?:ITEM|PARAMETER)\s+(\w+)\s", line)
        if m and m.group(1) in states:
            out.extend(f"      STATE {label} {value}" for label, value in states[m.group(1)])
    return "\n".join(out) + "\n"


def convert(def_file, package, topic_prefix, offsets, types, errors):
    """Rewrites one generated definition file; returns (text, packets)."""
    stem = def_file.name[: -len("_def.txt")]
    intf = stem.split("_eds_", 1)[1].upper()  # HK_TLM, CMD, LONG_EVENT_MSG, ...
    topic = f"{topic_prefix}_{intf}_TOPICID"
    if topic not in offsets:
        errors.append(f"{def_file.name}: topic {topic} not in cfe-topicids.xml")
        return "", []
    is_cmd = intf.endswith("CMD")
    mid = (CMD_BASE_MID if is_cmd else TLM_BASE_MID) + offsets[topic]
    text = def_file.read_text()
    packets = []

    # STATES of the telemetry payload (commands of OrthoStream have none).
    states = {}
    msg_type = types.interfaces.get(f"{package}/{intf}")
    if msg_type and not is_cmd:
        states = {item: types.enums[e] for item, e in types.flatten(msg_type)}

    def cmd(m):
        packets.append(("CMD", m.group(1), mid))
        return header_cmd(m.group(1), mid, int(m.group(2)), m.group(3)).rstrip("\n")

    def tlm(m):
        packets.append(("TLM", m.group(1), mid))
        return header_tlm(m.group(1), mid, m.group(2)).rstrip("\n")

    text = CMD_RE.sub(cmd, text)
    text = TLM_RE.sub(tlm, text)
    if "<%" in text:
        errors.append(f"{def_file.name}: unconverted ERB")
    return add_states(text, states), packets


def hk_screen(packet, text):
    items = re.findall(r"^\s*APPEND_ITEM\s+(\w+)\s", text.split(f"TELEMETRY ORTHOSTREAM {packet} ", 1)[1], re.M)
    items = [i for i in items if not i.startswith(("CCSDS_", "SPARE")) and i not in ("SECONDS", "SUBSECS")]
    lines = ["SCREEN AUTO AUTO 1.0", f'TITLE "{packet}"', "VERTICAL",
             f"  LABELVALUE ORTHOSTREAM {packet} PACKET_TIMEFORMATTED"]
    lines += [f"  LABELVALUE ORTHOSTREAM {packet} {i}" for i in items]
    lines += ["END", ""]
    return "\n".join(lines)


def main():
    root = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cosmos-dir", type=Path, default=root / "build" / "cosmos")
    ap.add_argument("--output", type=Path, default=root / "build" / "cosmos_plugin" / "openc3-cosmos-orthostream")
    args = ap.parse_args()

    template = root / "ground" / "cosmos" / "plugin"
    vendor = root / "vendor" / "cfs" / "tools" / "cfs-cosmos-plugin"
    offsets = topic_offsets(root / "mission" / "orthostream_defs" / "eds" / "cfe-topicids.xml")

    packages = dict(PACKAGES)
    eds_files = list((root / "apps").glob("*/eds/*.xml"))
    for f in eds_files:
        name = f.stem
        if name != "ost":
            packages[name] = (name.upper(), name.upper())
    eds_files += list((root / "vendor" / "cfs" / "cfe" / "modules").glob("*/eds/*.xml"))
    eds_files += [root / "vendor/cfs/apps/to_lab/eds/to_lab.xml", root / "vendor/cfs/apps/ci_lab/eds/ci_lab.xml"]
    types = EdsTypes(eds_files)

    if args.output.exists():
        shutil.rmtree(args.output)
    target = args.output / "targets" / "ORTHOSTREAM"
    (target / "cmd_tlm").mkdir(parents=True)
    (target / "lib").mkdir()
    (target / "screens").mkdir()
    for f in ["plugin.txt", "openc3-cosmos-orthostream.gemspec", "Rakefile"]:
        shutil.copy(template / f, args.output / f)
    shutil.copy(template / "targets" / "ORTHOSTREAM" / "target.txt", target / "target.txt")
    shutil.copy(root / "LICENSE", args.output / "LICENSE.txt")
    for f in ["protocol_ccsds_checksum.rb", "protocol_ccsds_length.rb", "protocol_evs_log.rb",
              "unix_time_conversion_epoch_offset.rb"]:
        shutil.copy(vendor / "targets" / "CFS" / "lib" / f, target / "lib" / f)
    (args.output / "README.md").write_text(
        "# openc3-cosmos-orthostream\n\nGenerated by tools/ground/generate_cosmos_plugin.py; do not edit.\n"
        "The CCSDS protocols and the time conversion under targets/ORTHOSTREAM/lib come from the\n"
        "cFS COSMOS plugin (NASA, Apache-2.0).\n")

    errors, all_packets = [], []
    for def_file in sorted(args.cosmos_dir.glob("*_def.txt")):
        prefix = def_file.name.split("_eds_", 1)[0]
        if prefix not in packages:
            continue
        package, topic_prefix = packages[prefix]
        text, packets = convert(def_file, package, topic_prefix, offsets, types, errors)
        if not packets:
            continue
        (target / "cmd_tlm" / def_file.name).write_text(text)
        all_packets += packets
        for kind, name, _ in packets:
            if kind == "TLM" and name.endswith("_HK") and prefix in packages and prefix not in PACKAGES:
                (target / "screens" / f"{name.lower()}.txt").write_text(hk_screen(name, text))
        if prefix == "ost" and "FAULT_EVENT" in def_file.name:
            (target / "screens" / "fault_event.txt").write_text(hk_screen(packets[0][1], text))

    mids = {}
    for kind, name, mid in all_packets:
        if kind == "TLM" and mids.setdefault(mid, name) != name:
            errors.append(f"message identifier 0x{mid:04X} used by {mids[mid]} and {name}")
    for e in errors:
        print("error:", e, file=sys.stderr)
    cmds = sum(1 for k, _, _ in all_packets if k == "CMD")
    print(f"{cmds} commands, {len(all_packets) - cmds} telemetry packets -> {args.output}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
