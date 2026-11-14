# Injector

Replays a recorded survey flight in real time over the emulated radio link
(ADR-003): autopilot telemetry on UDP, images on TCP, in the format of
[`docs/interfaces/link.md`](../../docs/interfaces/link.md). Python 3, standard
library only.

## Inputs

| Input | Produced by |
| --- | --- |
| pose file: camera calibration and one camera pose per image; selects the pose set | `tools/dataset/extract_metashape_poses.py` (`optimized`), `tools/dataset/extract_geotag_poses.py` (`non-optimized`) |
| capture time file: capture time and position accuracy per image | `tools/dataset/extract_capture_times.py` |
| image directory | the recorded JPEG files |

For Dorfberg Flight 01 the capture times are the PPK event marks of the
WingtraHub project file. The autopilot's trigger log is 70–190 ms later and
is not used.

## Use

```bash
cd tools/injector
python3 -m orthostream_injector \
    --poses ../../data/processed/dorfberg_f01/poses_optimized.txt \
    --captures ../../data/processed/dorfberg_f01/captures.txt \
    --images <image directory> --first 60 --count 40 --log sent.csv
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--first`, `--count`, `--decimate` | all images | images to send, after sorting by capture time; decimation sends every n-th image. The telemetry still follows all images. |
| `--time-factor` | 1 | scales the timeline (ADR-014 section 5) |
| `--telemetry-rate` | 10 Hz | in emulated time |
| `--capture-samples` | off | also sends a telemetry sample at each capture time, as an autopilot with camera trigger feedback does |
| `--image-time-offset-ms` | 0 | shifts the image timestamps only (timestamp offset experiment) |
| `--attitude-std-deg` | 3 3 3 | reported attitude accuracy, roll pitch yaw |
| `--fix` | 6 (PPK) | reported position fix |
| `--host`, `--udp-port`, `--tcp-port` | 127.0.0.1, 50100, 50101 | address of `INGEST_APP` |
| `--log` | none | CSV of the sent images with capture and send times |
| `--dry-run` | off | sends nothing; prints the schedule and the interpolation check below |

## Telemetry

The injector turns the per-image camera poses into body poses with the
nominal camera mounting, and interpolates them into a continuous trajectory.
It uses a cubic Hermite curve for the position and the attitude. The curve
passes through every image pose, with continuous velocity and angular rate.
The trajectory is sampled at the telemetry rate.

`--dry-run` reports how far a receiver that interpolates linearly between
the samples (ADR-014 section 4) is from the pose at each capture time.
Dorfberg Flight 01, all images:

| Telemetry | Curve | Attitude median | Attitude max | Position median | Position max |
| --- | --- | --- | --- | --- | --- |
| 10 Hz | piecewise linear (first version) | 0.042° | 0.45° | 5.3 mm | 0.54 m |
| 10 Hz | cubic Hermite | 0.0038° | 0.048° | 0.4 mm | 39 mm |
| 20 Hz | cubic Hermite | 0.0011° | 0.023° | 0.1 mm | 11 mm |
| 10 Hz + `--capture-samples` | either | 0 | 0 | 0 | 0 |

The largest values occur in the turns between flight lines.

## Not yet implemented

Controlled position and attitude errors (noise and slowly varying bias,
ADR-003 section 6 and ADR-019 section 7) and the link profiles with `netem`
(ADR-003 section 5).

## Tests

```bash
cd tools/injector
PYTHONPATH=.:tests python3 -m unittest discover -s tests
```

The link test vectors are rewritten with
`python3 -m orthostream_injector.link_vectors ../../interfaces/link/test_vectors.txt`.
