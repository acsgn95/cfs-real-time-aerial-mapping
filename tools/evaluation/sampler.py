#!/usr/bin/env python3
"""System resource sampler of a measurement run (ADR-017 section 3).

Writes one CSV row per period (default 1 s) until it is stopped (SIGINT or
SIGTERM) or --duration ends:

* utc_ns: sample time, joined with the recorder output by UTC time;
* cpu_pct: whole machine, 100 % = all logical CPUs busy; cpu_mhz: mean clock;
  cpu_temp_c: package temperature (x86_pkg_temp);
* mem_used_mb, mem_available_mb, swap_used_mb;
* per watched process (--proc NAME=PATTERN): <NAME>_cpu_pct (100 % = one
  CPU, all threads together), <NAME>_rss_mb, <NAME>_threads; the process is
  found by its command name or command line, again in every period, so it
  may start later (empty fields while it does not run);
* gpu_util_pct, gpu_mem_used_mb, gpu_temp_c, gpu_sm_mhz, gpu_power_w
  (nvidia-smi; empty without it).

No monitoring dependency enters the flight software; the sampler covers the
injector as well.

    tools/evaluation/sampler.py --out sampler.csv --proc cfs=core-cpu1 --proc injector=orthostream_injector

Standard library only (nvidia-smi as an external command).
"""

import argparse
import csv
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

CLK_TCK = os.sysconf("SC_CLK_TCK")
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024


def cpu_times():
    """(busy, total) jiffies of all CPUs."""
    with open("/proc/stat") as f:
        values = [int(v) for v in f.readline().split()[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values) - idle, sum(values)


def meminfo():
    out = {}
    with open("/proc/meminfo") as f:
        for line in f:
            key, value = line.split(":", 1)
            out[key] = int(value.split()[0])  # kB
    return out


def cpu_mhz():
    values = []
    with open("/proc/cpuinfo") as f:
        for line in f:
            if line.startswith("cpu MHz"):
                values.append(float(line.split(":", 1)[1]))
    return sum(values) / len(values) if values else None


def package_temp():
    for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            if (zone / "type").read_text().strip() == "x86_pkg_temp":
                return int((zone / "temp").read_text()) / 1000.0
        except OSError:
            continue
    return None


def find_pids(pattern):
    """Processes whose command name or command line contains the pattern."""
    pids = []
    me = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == me:
            continue
        try:
            comm = (entry / "comm").read_text().strip()
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if pattern == comm or (pattern in cmdline and "sampler.py" not in cmdline):
            pids.append(int(entry.name))
    return pids


def process_stats(pid):
    """(cpu jiffies, rss kB, threads) of a process, or None."""
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except OSError:
        return None
    fields = stat[stat.rindex(")") + 2:].split()
    utime, stime, threads, rss_pages = int(fields[11]), int(fields[12]), int(fields[17]), int(fields[21])
    return utime + stime, rss_pages * PAGE_KB, threads


GPU_FIELDS = "utilization.gpu,memory.used,temperature.gpu,clocks.sm,power.draw"


def gpu_sample(smi):
    if smi is None:
        return [None] * 5
    try:
        out = subprocess.run([smi, f"--query-gpu={GPU_FIELDS}", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=2).stdout.strip().splitlines()
    except (OSError, subprocess.SubprocessError):
        return [None] * 5
    if not out:
        return [None] * 5
    values = []
    for v in out[0].split(","):
        try:
            values.append(float(v))
        except ValueError:
            values.append(None)
    return values


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--proc", action="append", default=[], metavar="NAME=PATTERN")
    ap.add_argument("--period", type=float, default=1.0, help="seconds")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0: until stopped")
    args = ap.parse_args(argv)

    procs = [p.split("=", 1) for p in args.proc]
    smi = shutil.which("nvidia-smi")
    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    header = ["utc_ns", "cpu_pct", "cpu_mhz", "cpu_temp_c", "mem_used_mb", "mem_available_mb", "swap_used_mb"]
    for name, _ in procs:
        header += [f"{name}_cpu_pct", f"{name}_rss_mb", f"{name}_threads"]
    header += ["gpu_util_pct", "gpu_mem_used_mb", "gpu_temp_c", "gpu_sm_mhz", "gpu_power_w"]

    start = time.monotonic()
    last_cpu = cpu_times()
    last_proc = {}  # pid -> jiffies
    last_t = time.monotonic()
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        while not stop:
            time.sleep(max(0.0, args.period - ((time.monotonic() - start) % args.period)))
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            busy, total = cpu_times()
            cpu_pct = 100.0 * (busy - last_cpu[0]) / max(1, total - last_cpu[1])
            last_cpu = (busy, total)
            m = meminfo()
            row = [time.time_ns(), round(cpu_pct, 1), round(cpu_mhz() or 0, 0), package_temp(),
                   (m["MemTotal"] - m["MemAvailable"]) // 1024, m["MemAvailable"] // 1024,
                   (m["SwapTotal"] - m["SwapFree"]) // 1024]
            for name, pattern in procs:
                jiffies = rss = threads = 0
                found = False
                for pid in find_pids(pattern):
                    s = process_stats(pid)
                    if s is None:
                        continue
                    found = True
                    previous = last_proc.get(pid, s[0])
                    last_proc[pid] = s[0]
                    jiffies += s[0] - previous
                    rss += s[1]
                    threads += s[2]
                if found:
                    row += [round(100.0 * jiffies / CLK_TCK / dt, 1), rss // 1024, threads]
                else:
                    row += [None, None, None]
            row += gpu_sample(smi)
            w.writerow(row)
            f.flush()
            if args.duration and now - start >= args.duration:
                break
    return 0


if __name__ == "__main__":
    sys.exit(main())
