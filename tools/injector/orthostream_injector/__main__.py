"""Command line of the injector (ADR-003).

    python3 -m orthostream_injector --poses <pose file> --captures <capture file>
        --images <image dir> [options]

Example, Dorfberg Flight 01 in real time, Metashape poses, 40 images:

    cd tools/injector
    python3 -m orthostream_injector \\
        --poses ../../data/processed/dorfberg_f01/poses_optimized.txt \\
        --captures ../../data/processed/dorfberg_f01/captures.txt \\
        --images <Dorfberg image directory> --first 60 --count 40

--dry-run sends nothing. It prints the schedule and how far a receiver that
interpolates linearly between the telemetry samples (ADR-014 section 4) is
from the recorded pose at each capture time.
"""

import argparse
import csv
import math
import statistics
import sys
import time

from . import dataset, geodesy, injector
from .trajectory import Trajectory


def parse_args(argv):
    o = injector.Options()
    ap = argparse.ArgumentParser(prog="orthostream_injector", description="Replays a recorded flight (ADR-003).")
    ap.add_argument("--poses", required=True, help="pose file; selects the pose set")
    ap.add_argument("--captures", required=True, help="capture time file")
    ap.add_argument("--images", required=True, help="image directory")
    ap.add_argument("--first", type=int, default=0, help="index of the first image (after sorting by time)")
    ap.add_argument("--count", type=int, help="number of images to send")
    ap.add_argument("--decimate", type=int, default=1, help="send every n-th image")
    ap.add_argument("--time-factor", type=float, default=o.time_factor, help="1 = real time")
    ap.add_argument("--telemetry-rate", type=float, default=o.telemetry_rate_hz, help="Hz, in emulated time")
    ap.add_argument("--capture-samples", action="store_true",
                    help="also send a telemetry sample at each capture time")
    ap.add_argument("--image-time-offset-ms", type=float, default=0.0,
                    help="added to the image timestamps only (ADR-014 section 5)")
    ap.add_argument("--attitude-std-deg", type=float, nargs=3, default=o.attitude_std_deg,
                    metavar=("ROLL", "PITCH", "YAW"), help="reported attitude accuracy")
    ap.add_argument("--fix", type=int, default=o.position_fix, help="reported position fix (OST PositionFix)")
    ap.add_argument("--camera-name", default=o.camera_name)
    ap.add_argument("--session-id", type=int, help="default: from the clock")
    ap.add_argument("--host", default=o.host)
    ap.add_argument("--udp-port", type=int, default=o.udp_port)
    ap.add_argument("--tcp-port", type=int, default=o.tcp_port)
    ap.add_argument("--log", help="CSV file of the sent images (sequence, name, capture and send times)")
    ap.add_argument("--dry-run", action="store_true", help="send nothing; print the schedule and checks")
    return ap.parse_args(argv)


def alignment_check(traj, images, times):
    """Attitude and position error of linear interpolation between the
    telemetry samples around each capture time."""
    att, pos = [], []
    for f in images:
        t = f.recorded_ns
        i = next((k for k in range(len(times) - 1) if times[k] <= t <= times[k + 1]), None)
        if i is None:
            continue
        u = (t - times[i]) / (times[i + 1] - times[i]) if times[i + 1] > times[i] else 0.0
        a = traj.sample(times[i])
        b = traj.sample(times[i + 1])
        pa = geodesy.geodetic_to_ecef(math.radians(a[0]), math.radians(a[1]), a[2])
        pb = geodesy.geodetic_to_ecef(math.radians(b[0]), math.radians(b[1]), b[2])
        p = [x + u * (y - x) for x, y in zip(pa, pb)]
        q = geodesy.q_slerp(a[3], b[3], u)
        lat, lon, h, q_true, _ = traj.sample(t)
        p_true = geodesy.geodetic_to_ecef(math.radians(lat), math.radians(lon), h)
        att.append(geodesy.q_angle_deg(q, q_true))
        pos.append(math.dist(p, p_true))
    return att, pos


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    flight = dataset.load_flight(args.poses, args.captures, args.images)
    images = dataset.select_images(flight.frames, args.first, args.count, args.decimate)
    opts = injector.Options(host=args.host, udp_port=args.udp_port, tcp_port=args.tcp_port,
                            time_factor=args.time_factor, telemetry_rate_hz=args.telemetry_rate,
                            capture_samples=args.capture_samples,
                            image_time_offset_ns=int(round(args.image_time_offset_ms * 1e6)),
                            attitude_std_deg=tuple(args.attitude_std_deg), position_fix=args.fix,
                            camera_name=args.camera_name,
                            session_id=(args.session_id if args.session_id is not None
                                        else int(time.time()) & 0xFFFFFFFF))

    duration = (images[-1].recorded_ns - images[0].recorded_ns) / 1e9 / opts.time_factor
    total = sum(f.path.stat().st_size for f in images)
    skipped = ", ".join(f"{n} {r}" for r, n in flight.skipped.items() if n)
    print(f"{len(images)} images of {len(flight.frames)} ({skipped or 'none skipped'}), "
          f"{duration:.1f} s, {total / 1e6:.0f} MB, {total / 1e6 / max(duration, 1e-9):.1f} MB/s")

    if args.dry_run:
        traj = Trajectory(flight.frames)
        period = int(round(1e9 * opts.time_factor / opts.telemetry_rate_hz))
        extra = [f.recorded_ns for f in images] if opts.capture_samples else ()
        times = traj.sample_times(period, images[0].recorded_ns, images[-1].recorded_ns + period, extra)
        att, pos = alignment_check(traj, images, times)
        print(f"telemetry {opts.telemetry_rate_hz:g} Hz; interpolation at the capture times: attitude median "
              f"{statistics.median(att):.4f} deg, max {max(att):.4f} deg; position median "
              f"{statistics.median(pos) * 1000:.1f} mm, max {max(pos) * 1000:.1f} mm")
        return 0

    def progress(r):
        print(f"\r{r.images_sent}/{len(images)} images, {r.samples_sent} samples", end="", flush=True)

    report = injector.run(opts, flight, images, progress)
    print()
    print(f"sent {report.images_sent} images ({report.image_bytes / 1e6:.0f} MB), {report.samples_sent} samples, "
          f"{report.heartbeats_sent} heartbeats; latest start: image {report.max_image_late_ns / 1e6:.1f} ms, "
          f"telemetry {report.max_telemetry_late_ns / 1e6:.1f} ms")
    if args.log:
        with open(args.log, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["image_seq", "name", "capture_utc_ns", "send_start_utc_ns", "send_end_utc_ns"])
            w.writerows(report.image_log)
    if report.error:
        print(f"error: {report.error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
