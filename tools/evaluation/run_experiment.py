#!/usr/bin/env python3
"""Runs a measurement of the mission from an experiment file (ADR-017,
ADR-018).

One run, end to end:

 1. checks: no cFS running, release build, POSIX queue limit, machine load;
 2. starts cFS (build/exe/cpu1) with empty products and recordings, and the
    resource sampler (sampler.py);
 3. starts the recording of RECORDER_APP under the run identifier;
 4. replays the flight with the injector (parameters of the experiment);
 5. waits until the data path is empty: every selected batch has a dense
    outcome, every depth set its map update, and no data path message for
    `settle_s` (or `max_wait_s` after the injector);
 6. FINALISE_ALL, stop of the recording, stop of cFS;
 7. into <output>/<run id>/: the recording with its export (CSV, events,
    JSON lines), the products (DSM, orthomosaic, optionally the point
    cloud), cfs.log, injector.log, sent.csv, sampler.csv, the reports of
    tools/ground, the DSM evaluation against the reference, and
    manifest.json: the recorder's manifest with what the ground knows (git
    commit, build, queue limit, machine, load, experiment file, injector
    command, table checksums).

    tools/evaluation/run_experiment.py tools/evaluation/experiments/e1_window.toml [--repetitions N]

Experiment files are TOML (tomllib, standard library); see
tools/evaluation/experiments/. Standard library only.
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import time
import tomllib
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CPU1 = REPO / "build/exe/cpu1"
GROUND = REPO / "tools/ground"
sys.path.insert(0, str(GROUND))
from recorder_export import read_recording  # noqa: E402
from tlm_capture import DEFAULT_DEFS, load_definitions  # noqa: E402

DATA_PATH = {"OST_BATCH_SELECTED", "OST_BATCH_READY", "DENSE_APP_DEPTH_READY", "DENSE_APP_DENSE_STATUS",
             "GEOREF_APP_POINTS_READY", "MAPPROD_APP_TILE_UPDATED"}


def log(text):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {text}", flush=True)


def sh(*cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def send(command, *params):
    args = [sys.executable, str(GROUND / "send_command.py"), command]
    for p in params:
        args += ["--param", p]
    r = sh(*args)
    if r.returncode != 0:
        raise RuntimeError(f"{command}: {r.stderr.strip()}")


def wait_for(predicate, timeout, period=1.0, what=""):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period)
    raise TimeoutError(f"timed out after {timeout:.0f} s waiting for {what}")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# --------------------------------------------------------------- checks

def process_jiffies():
    out = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
        except OSError:
            continue
        fields = stat[stat.rindex(")") + 2:].split()
        out[int(entry.name)] = (stat[stat.index("(") + 1:stat.rindex(")")], int(fields[11]) + int(fields[12]))
    return out


def machine_state():
    """Load and the processes that used at least 20 % of a CPU over one
    second (command names only), and the GPU processes."""
    load = os.getloadavg()
    a = process_jiffies()
    time.sleep(1.0)
    b = process_jiffies()
    tck = os.sysconf("SC_CLK_TCK")
    me = os.getpid()
    busy = sorted(((b[p][1] - a[p][1]) * 100.0 / tck, b[p][0]) for p in b if p in a and p != me)
    busy = [f"{name} {pct:.0f} %" for pct, name in reversed(busy) if pct >= 20.0]
    gpu_apps = []
    if shutil.which("nvidia-smi"):
        out = sh("nvidia-smi", "--query-compute-apps=process_name,used_memory", "--format=csv,noheader").stdout
        gpu_apps = [Path(l.split(",")[0].strip()).name + "," + l.split(",")[1] for l in out.splitlines() if l.strip()]
    return {"loadavg": [round(x, 2) for x in load], "busy_processes": busy, "gpu_processes": gpu_apps}


def checks(force):
    problems, notes = [], []
    if sh("pgrep", "-x", "core-cpu1").stdout.strip():
        problems.append("cFS (core-cpu1) is already running")
    cache = (REPO / "build/CMakeCache.txt").read_text() if (REPO / "build/CMakeCache.txt").exists() else ""
    build_type = next((l.split("=", 1)[1] for l in cache.splitlines() if l.startswith("CMAKE_BUILD_TYPE:")), "?")
    if build_type.lower() != "release":
        problems.append(f"mission build type is {build_type}, measurements need release")
    msg_max = int(Path("/proc/sys/fs/mqueue/msg_max").read_text())
    if msg_max < 64:
        notes.append(f"fs.mqueue.msg_max is {msg_max} (< 64): Software Bus pipes are truncated (README)")
    state = machine_state()
    if state["busy_processes"] or state["gpu_processes"]:
        notes.append(f"machine not idle: {state['busy_processes']} {state['gpu_processes']}")
    for n in notes:
        log("warning: " + n)
    if problems and not force:
        raise SystemExit("cannot run: " + "; ".join(problems))
    return {"build_type": build_type, "msg_max": msg_max, "machine": state, "warnings": notes}


def environment():
    def first(cmd):
        try:
            return sh(*cmd).stdout.strip().splitlines()[0]
        except (OSError, IndexError):
            return None
    cpu = next((l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")), None)
    mem_kb = next(int(l.split()[1]) for l in open("/proc/meminfo") if l.startswith("MemTotal"))
    vcpkg = json.loads((REPO / "vcpkg.json").read_text()) if (REPO / "vcpkg.json").exists() else {}
    return {
        "git_commit": first(["git", "-C", str(REPO), "rev-parse", "HEAD"]),
        "git_dirty": bool(sh("git", "-C", str(REPO), "status", "--porcelain", "--untracked-files=no").stdout.strip()),
        "git_branch": first(["git", "-C", str(REPO), "rev-parse", "--abbrev-ref", "HEAD"]),
        "kernel": platform.release(),
        "cpu": cpu, "logical_cpus": os.cpu_count(), "memory_gb": round(mem_kb / 1048576, 1),
        "gpu": first(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"]),
        "python": platform.python_version(),
        "vcpkg_baseline": vcpkg.get("builtin-baseline"),
        "cfs_commit": first(["git", "-C", str(REPO / "vendor/cfs"), "rev-parse", "HEAD"]),
    }


def scheduling(pid):
    """Real-time priorities take effect only with CAP_SYS_NICE (ADR-015)."""
    try:
        cap = next(l.split()[1] for l in open(f"/proc/{pid}/status") if l.startswith("CapEff"))
        return {"cap_sys_nice": bool(int(cap, 16) & (1 << 23)), "cap_eff": cap}
    except (OSError, StopIteration):
        return {"cap_sys_nice": None}


# ------------------------------------------------------------ the run

class DataPath:
    """Counts of the data path in the recording of the run, read again on
    every call (the file is small)."""

    def __init__(self, recording):
        self.recording = recording
        self.defs = load_definitions(DEFAULT_DEFS)
        self.counts, self.last_rx, self.finalised = {}, 0, False

    def update(self):
        try:
            _, records, _ = read_recording(self.recording)
        except (OSError, ValueError):
            return
        counts, last, finalised = {}, 0, False
        for rx, msg in records:
            packet = self.defs.get(int.from_bytes(msg[:2], "big")) if len(msg) >= 2 else None
            if packet is None:
                continue
            if packet.name in DATA_PATH:
                counts[packet.name] = counts.get(packet.name, 0) + 1
                last = max(last, rx)
            elif packet.name == "MAPPROD_APP_TILE_PUBLISHED" and packet.decode(msg).get("REASON") == 2:
                finalised = True
        self.counts, self.last_rx, self.finalised = counts, last, finalised

    def balanced(self):
        c = self.counts
        selected = c.get("OST_BATCH_SELECTED", 0)
        return (selected > 0 and c.get("DENSE_APP_DENSE_STATUS", 0) >= selected
                and c.get("MAPPROD_APP_TILE_UPDATED", 0) >= c.get("DENSE_APP_DEPTH_READY", 0))


def injector_command(cfg, run_dir):
    inj = cfg["injector"]
    cmd = [sys.executable, "-m", "orthostream_injector",
           "--poses", str(REPO / inj["poses"]), "--captures", str(REPO / inj["captures"]),
           "--images", inj["images"], "--log", str(run_dir / "sent.csv")]
    for key in ("first", "count", "decimate", "time_factor", "telemetry_rate", "image_time_offset_ms",
                "session_id"):
        if key in inj:
            cmd += ["--" + key.replace("_", "-"), str(inj[key])]
    if inj.get("capture_samples"):
        cmd.append("--capture-samples")
    return cmd


def run_once(cfg, cfg_text, cfg_path, run_id, out_root, force):
    run_dir = out_root / run_id
    if run_dir.exists():
        raise SystemExit(f"{run_dir} exists: run identifiers name measurements")
    run_dir.mkdir(parents=True)
    pre = checks(force)
    rc = cfg.get("run", {})
    started = time.time_ns()
    log(f"run {run_id} -> {run_dir}")

    # cFS with empty products and recordings
    for d in ("products", "runs"):
        shutil.rmtree(CPU1 / "cf" / d, ignore_errors=True)
    cfs_log = open(run_dir / "cfs.log", "w")
    cfs = subprocess.Popen(["./core-cpu1"], cwd=CPU1, stdout=cfs_log, stderr=subprocess.STDOUT,
                           start_new_session=True)
    sampler = None
    try:
        text = lambda: (run_dir / "cfs.log").read_text(errors="replace")  # noqa: E731
        wait_for(lambda: "RECORDER_APP 11:" in text() and "MAPPROD_APP 27:" in text(), 90, what="cFS start-up")
        sched = scheduling(cfs.pid)
        sampler = subprocess.Popen([sys.executable, str(REPO / "tools/evaluation/sampler.py"),
                                    "--out", str(run_dir / "sampler.csv"), "--proc", "cfs=core-cpu1",
                                    "--proc", "injector=orthostream_injector"])
        send("RECORDER_APP_CMD_START_RECORDING", f"RUN_ID={run_id}")
        wait_for(lambda: f"recording run {run_id}" in text(), 10, what="recording start")
        log("cFS up, recording")

        # flight
        inj_cmd = injector_command(cfg, run_dir)
        inj_start = time.time_ns()
        with open(run_dir / "injector.log", "w") as inj_log:
            r = subprocess.run(inj_cmd, cwd=REPO / "tools/injector", stdout=inj_log, stderr=subprocess.STDOUT)
        inj_end = time.time_ns()
        if r.returncode != 0:
            raise RuntimeError(f"injector failed ({r.returncode}), see injector.log")
        log(f"injector done in {(inj_end - inj_start) / 1e9:.0f} s; waiting for the data path")

        # data path empty
        path = DataPath(CPU1 / "cf/runs" / run_id / "recording.ostrec")
        settle = float(rc.get("settle_s", 20))
        deadline = time.monotonic() + float(rc.get("max_wait_s", 400))
        while time.monotonic() < deadline:
            time.sleep(5)
            path.update()
            quiet = (time.time_ns() - path.last_rx) / 1e9 if path.last_rx else 0
            if path.balanced() and quiet >= settle:
                break
        else:
            log(f"warning: data path not balanced after max_wait_s: {path.counts}")
        drained = time.time_ns()
        log(f"data path: {path.counts}")

        send("MAPPROD_APP_CMD_FINALISE_ALL")
        try:
            wait_for(lambda: (path.update(), path.finalised)[1], 120, period=2, what="FINALISE_ALL")
        except TimeoutError as e:
            log(f"warning: {e}")
        time.sleep(2)
        send("RECORDER_APP_CMD_STOP_RECORDING")
        flight_manifest = CPU1 / "cf/runs" / run_id / "manifest.json"
        wait_for(lambda: flight_manifest.exists() and '"stopped"' in flight_manifest.read_text(), 15,
                 what="recording stop")
    finally:
        if sampler is not None:
            sampler.send_signal(signal.SIGTERM)
            sampler.wait(10)
        if cfs.poll() is None:
            os.killpg(cfs.pid, signal.SIGINT)
            try:
                cfs.wait(30)
            except subprocess.TimeoutExpired:
                os.killpg(cfs.pid, signal.SIGKILL)
        cfs_log.close()
    log("cFS stopped; collecting")

    # collect
    shutil.copytree(CPU1 / "cf/runs" / run_id, run_dir / "recording")
    keep = set(rc.get("keep_products", ["dsm", "ortho"]))
    products = CPU1 / "cf/products"
    (run_dir / "products").mkdir()
    for name in ("dsm", "ortho", "points"):
        if name in keep and (products / name).exists():
            shutil.copytree(products / name, run_dir / "products" / name)
    for vrt in ("dsm.vrt", "ortho.vrt"):
        if vrt.split(".")[0] in keep and (products / vrt).exists():
            shutil.copy2(products / vrt, run_dir / "products" / vrt)
    shutil.copy2(cfg_path, run_dir / "experiment.toml")

    rec = run_dir / "recording"
    sh(sys.executable, str(GROUND / "recorder_export.py"), str(rec))
    with open(run_dir / "reports.txt", "w") as f:
        for tool, extra in (("dense_report.py", []), ("georef_report.py", []), ("mapprod_report.py", [])):
            r = sh(sys.executable, str(GROUND / tool), str(rec / "recording.jsonl"), *extra)
            f.write(f"== {tool}\n{r.stdout}{r.stderr}\n")

    evaluation = None
    ref = rc.get("reference_dsm")
    if ref and (run_dir / "products/dsm").exists():
        r = sh(str(REPO / "build-libs/release/tools/offline_pipeline/offline_pipeline"), "--evaluate-only",
               "--out", str(run_dir / "products"), "--reference-dsm", ref,
               "--proj-data", str(CPU1 / "cf/geodata/proj"), "--grid-dir", str(REPO / "build-deps/proj-grids"))
        evaluation = next((l for l in r.stdout.splitlines() if l.startswith("DSM against")), r.stderr.strip())

    manifest = {
        "run_id": run_id,
        "experiment": cfg.get("run", {}).get("name"),
        "experiment_file": str(cfg_path),
        "experiment_config": cfg,
        "started_utc_ns": started,
        "injector_start_utc_ns": inj_start,
        "injector_end_utc_ns": inj_end,
        "data_path_drained_utc_ns": drained,
        "data_path_counts": path.counts,
        "injector_command": inj_cmd,
        "checks": pre,
        "scheduling": sched,
        "environment": environment(),
        "tables": {p.name: sha256(p) for p in sorted((CPU1 / "cf").glob("*.tbl"))},
        "recorder": json.loads((rec / "manifest.json").read_text()),
        "dsm_evaluation": evaluation,
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    shutil.rmtree(CPU1 / "cf/runs", ignore_errors=True)
    shutil.rmtree(CPU1 / "cf/products", ignore_errors=True)
    log(f"done: {run_dir}")
    if evaluation:
        log(evaluation)
    return run_dir


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("experiment", help="TOML experiment file")
    ap.add_argument("--repetitions", type=int, help="overrides run.repetitions")
    ap.add_argument("--output", help="overrides run.output")
    ap.add_argument("--force", action="store_true", help="run despite failed checks (not for measurements)")
    args = ap.parse_args(argv)

    cfg_path = Path(args.experiment).resolve()
    cfg_text = cfg_path.read_text()
    cfg = tomllib.loads(cfg_text)
    rc = cfg.get("run", {})
    if "tables" in cfg:
        raise SystemExit("[tables]: table images from experiment files are not supported yet; "
                         "the run uses the tables of the build")
    name = rc.get("name") or cfg_path.stem
    prefix = rc.get("id_prefix", name[:6])
    out_root = Path(args.output or rc.get("output", "/home/ahmet/MasterProgramm/test_outputs/runs"))
    repetitions = args.repetitions or int(rc.get("repetitions", 1))
    stamp = datetime.now(timezone.utc).strftime("%m%d%H%M%S")
    for k in range(1, repetitions + 1):
        # A run directory is one OSAL path component: at most 19 characters.
        run_id = f"{prefix}_{stamp}_{k}"
        if len(run_id) > 19:
            raise SystemExit(f"run identifier {run_id} longer than 19 characters: shorten run.id_prefix")
        run_once(cfg, cfg_text, cfg_path, run_id, out_root, args.force)
    return 0


if __name__ == "__main__":
    sys.exit(main())
