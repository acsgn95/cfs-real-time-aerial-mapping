# Evaluation tools

Measurement runs of the mission (ADR-017, ADR-018). Standard library only.

| Tool | Use |
| --- | --- |
| `run_experiment.py` | one experiment file, N runs, each end to end: cFS, recording, injector, drained data path, final tiles, collection, reports, DSM evaluation, manifest |
| `sampler.py` | CPU, memory, per-process CPU and memory (cFS, injector), GPU, temperatures and clocks at 1 Hz (ADR-017 section 3); started by `run_experiment.py` |
| `experiments/*.toml` | experiment files |

## A run

```bash
tools/evaluation/run_experiment.py tools/evaluation/experiments/e1_window.toml
tools/evaluation/run_experiment.py tools/evaluation/experiments/e1_nominal.toml --repetitions 3
```

Preconditions, checked before each run: no cFS running, a release build
(`ORTHOSTREAM_BUILD_TYPE=release mission/build.sh`), and the COSMOS plugin
definitions (`tools/ground/generate_cosmos_plugin.py`, used to decode the
recording). A queue limit below 64 (`fs.mqueue.msg_max`, README) and other
processes that use the CPU or the GPU are reported as warnings and stored in
the manifest: results of a busy machine are not measurements.

Steps:

1. cFS starts in `build/exe/cpu1` with empty `cf/products` and `cf/runs`;
   the sampler starts.
2. `RECORDER_APP` records under the run identifier
   `<id_prefix>_<MMDDhhmmss>_<repetition>` (at most 19 characters: the run
   directory is one OSAL path component).
3. The injector replays the flight with the parameters of `[injector]`.
4. The run waits until every selected batch has a dense outcome, every
   depth set has its map update, and no data path message arrived for
   `settle_s` (at most `max_wait_s` after the injector).
5. `FINALISE_ALL`, stop of the recording, stop of cFS.

`<output>/<run id>/` then holds:

| Path | Content |
| --- | --- |
| `manifest.json` | the recorder's manifest and what the ground knows: git commit and state, build type, queue limit, machine, competing processes, real-time privileges of cFS, experiment configuration, injector command, table checksums, data path counts, times, DSM evaluation |
| `recording/` | `recording.ostrec`, its `manifest.json`, `csv/`, `events.log`, `recording.jsonl` (`tools/ground/recorder_export.py`) |
| `products/` | DSM and orthomosaic tiles with their VRT (`keep_products`), the DSM evaluation (`evaluation.txt`, `dsm_difference.tif`, `compare_overview.jpg`) |
| `reports.txt` | `dense_report.py`, `georef_report.py`, `mapprod_report.py` on the recording |
| `sampler.csv`, `cfs.log`, `injector.log`, `sent.csv`, `experiment.toml` | as named |

## Experiment files

TOML (`tomllib` of the standard library; ADR-016 named YAML, whose parser is
not in the standard library):

```toml
[run]
name = "e1_window"
id_prefix = "e1w"            # at most 6 characters
repetitions = 1
output = "/home/ahmet/MasterProgramm/test_outputs/runs"
settle_s = 20
max_wait_s = 400
keep_products = ["dsm", "ortho"]   # "points" for the point cloud
reference_dsm = "/path/to/DSM_Dorfberg_20201109_10cm.tif"

[injector]                   # options of tools/injector (ADR-003)
poses = "data/processed/dorfberg_f01/poses_optimized.txt"
captures = "data/processed/dorfberg_f01/captures.txt"
images = "/path/to/Dorfberg/images"
first = 40                   # optional: first, count, decimate, time_factor,
count = 80                   # telemetry_rate, image_time_offset_ms, session_id,
time_factor = 1.0            # capture_samples
```

Not yet supported: table values per experiment (`[tables]`; the run uses the
tables of the build and stores their checksums), pose errors added by the
injector (E4 attitude and position noise) and link profiles (E7, `netem`).
