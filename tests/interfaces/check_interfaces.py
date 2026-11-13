#!/usr/bin/env python3
"""Consistency checks of the OrthoStream interface definitions (ADR-020).

* Fault catalog: the EDS enumeration OST/FaultId, the C catalog
  (ost_fault_catalog.c) and docs/interfaces/faults.md list the same
  identifiers, names, severities and policies.
* Events: the identifiers of every apps/<app>/fsw/inc/<app>_eventids.h are
  unique, lie in the ranges of ADR-020 section 3, and are listed in
  docs/interfaces/events.md under the application's section.
* Topic identifiers: the OrthoStream topic identifiers of the reference
  mission are unique and stay inside their application's block.

Standard library only. Exit status 0 when all checks pass.
Usage: tests/interfaces/check_interfaces.py [repository root]
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SEDS_NS = "{http://www.ccsds.org/schema/sois/seds}"

EVENT_RANGES = [
    (1, 19, "life cycle and commands"),
    (20, 59, "data path"),
    (60, 79, "fault changes"),
    (80, 99, "library diagnostics"),
]

# Topic blocks of the reference mission (cfe-topicids.xml).
TOPIC_BLOCKS = {
    "INGEST_APP": 32,
    "ALIGN_APP": 48,
    "SELECT_APP": 64,
    "REFINE_APP": 80,
    "DENSE_APP": 96,
    "GEOREF_APP": 112,
    "MAPPROD_APP": 144,
    "RECORDER_APP": 160,
    "OST": 176,
}


class Checker:
    def __init__(self, root: Path):
        self.root = root
        self.errors = []

    def error(self, text):
        self.errors.append(text)

    # ------------------------------------------------------------ faults
    def eds_faults(self):
        tree = ET.parse(self.root / "apps/ost_common/eds/ost.xml")
        for enum in tree.iter(SEDS_NS + "EnumeratedDataType"):
            if enum.get("name") == "FaultId":
                return {
                    int(e.get("value")): e.get("label")
                    for e in enum.iter(SEDS_NS + "Enumeration")
                    if e.get("label") != "NONE"
                }
        self.error("ost.xml: enumeration FaultId not found")
        return {}

    def c_catalog(self):
        text = (self.root / "apps/ost_common/fsw/src/ost_fault_catalog.c").read_text()
        rows = re.findall(
            r"\{OST_FaultId_(\w+),\s*OST_FaultSeverity_(\w+),\s*OST_FAULT_POLICY_(\w+),\s*\"(\w+)\"\}", text)
        catalog = {}
        for ident, severity, policy, name in rows:
            if ident != name:
                self.error(f"ost_fault_catalog.c: name \"{name}\" does not match OST_FaultId_{ident}")
            if ident in catalog:
                self.error(f"ost_fault_catalog.c: {ident} listed twice")
            catalog[ident] = (severity, "latch" if policy == "LATCH" else "auto-clear")
        return catalog

    def doc_faults(self):
        text = (self.root / "docs/interfaces/faults.md").read_text()
        rows = re.findall(r"^\| (\d+) \| `(\w+)` \| `(\w+)` \| ([\w-]+) \|", text, re.M)
        return {int(i): (name, sev, pol) for i, name, sev, pol in rows}

    def check_faults(self):
        eds = self.eds_faults()
        cat = self.c_catalog()
        doc = self.doc_faults()
        for value, label in eds.items():
            if label not in cat:
                self.error(f"fault {label} ({value}) missing in ost_fault_catalog.c")
            if value not in doc:
                self.error(f"fault {label} ({value}) missing in docs/interfaces/faults.md")
                continue
            name, sev, pol = doc[value]
            if name != label:
                self.error(f"faults.md: id {value} is {name}, EDS says {label}")
            if label in cat and cat[label] != (sev, pol):
                self.error(f"faults.md: {label} is {sev}/{pol}, catalog says {cat[label][0]}/{cat[label][1]}")
        for label in cat:
            if label not in eds.values():
                self.error(f"ost_fault_catalog.c: {label} is not in the EDS enumeration")
        for value in doc:
            if value not in eds:
                self.error(f"faults.md: id {value} is not in the EDS enumeration")
        return len(eds)

    # ------------------------------------------------------------ events
    def check_events(self):
        doc = (self.root / "docs/interfaces/events.md").read_text()
        sections = {}
        for match in re.finditer(r"^### `?(\w+)`?\s*$(.*?)(?=^##|\Z)", doc, re.M | re.S):
            sections[match.group(1)] = {
                int(i): name for i, name in re.findall(r"^\| (\d+) \| `(\w+)` \|", match.group(2), re.M)
            }
        count = 0
        for header in sorted(self.root.glob("apps/*/fsw/inc/*_eventids.h")):
            app = header.name[: -len("_eventids.h")].upper()
            ids = {}
            for name, value in re.findall(r"#define\s+(\w+_EID)\s+(\d+)", header.read_text()):
                value = int(value)
                count += 1
                if value in ids:
                    self.error(f"{header.name}: {name} and {ids[value]} share id {value}")
                ids[value] = name
                if not any(lo <= value <= hi for lo, hi, _ in EVENT_RANGES):
                    self.error(f"{header.name}: {name} = {value} is outside the ranges of ADR-020")
                elif 60 <= value <= 99:
                    self.error(f"{header.name}: {name} = {value} lies in a range owned by ost_common")
            listed = sections.get(app)
            if listed is None:
                self.error(f"events.md: no section for {app}")
                continue
            for value, name in ids.items():
                if listed.get(value) != name:
                    self.error(f"events.md: {app} id {value} should be {name}, found {listed.get(value)}")
            for value in listed:
                if value < 60 and value not in ids:
                    self.error(f"events.md: {app} id {value} is not defined in {header.name}")
        return count

    # ------------------------------------------------------------ topics
    def check_topics(self):
        text = (self.root / "mission/orthostream_defs/eds/cfe-topicids.xml").read_text()
        seen = {}
        count = 0
        for name, kind, offset in re.findall(
                r'<Define name="(\w+)_TOPICID"\s+value="\$\{CFE_MISSION/(TELECOMMAND|TELEMETRY)_BASE_TOPICID\} \+ (\d+)"/>',
                text):
            owner = next((app for app in TOPIC_BLOCKS if name.startswith(app + "_")), None)
            if owner is None:
                continue
            count += 1
            offset = int(offset)
            key = (kind, offset)
            if key in seen:
                self.error(f"topic {name} shares {kind} offset {offset} with {seen[key]}")
            seen[key] = name
            base = TOPIC_BLOCKS[owner]
            if not base <= offset < base + 16:
                self.error(f"topic {name} = {offset} is outside the block {base}-{base + 15} of {owner}")
        return count

    def run(self):
        faults = self.check_faults()
        events = self.check_events()
        topics = self.check_topics()
        return faults, events, topics


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
    checker = Checker(root)
    faults, events, topics = checker.run()
    for text in checker.errors:
        print("error:", text)
    print(f"checked {faults} faults, {events} event identifiers, {topics} topic identifiers: "
          f"{len(checker.errors)} error(s)")
    return 1 if checker.errors else 0


if __name__ == "__main__":
    sys.exit(main())
