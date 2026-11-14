"""Replays a recorded flight over the two link channels (ADR-003, ADR-014).

Time mapping (ADR-014 section 1): a recorded time t_rec is sent at

    t_emulated = t_start + (t_rec - t_rec0) / time_factor

on the UTC system clock, and carried as data time; t_rec is carried as the
recorded time. t_rec0 is the capture time of the first image sent.

Two threads:
* telemetry: UDP datagrams at the telemetry rate (in emulated time), and a
  heartbeat every second;
* imagery: one TCP connection; SESSION_START, one IMAGE frame per image at
  its emulated capture time, SESSION_END.

The image thread starts sending a frame at the capture time; a frame that
cannot be sent in time delays the following ones (lateness is reported).
"""

import socket
import threading
import time
from dataclasses import dataclass, field

from . import link
from .trajectory import Trajectory

NS = 1_000_000_000


@dataclass
class Options:
    host: str = "127.0.0.1"
    udp_port: int = 50100
    tcp_port: int = 50101
    time_factor: float = 1.0
    telemetry_rate_hz: float = 10.0
    capture_samples: bool = False
    image_time_offset_ns: int = 0
    attitude_std_deg: tuple = (3.0, 3.0, 3.0)
    position_fix: int = 6  # PPK
    camera_name: str = "camera"
    session_id: int = 0
    start_delay_s: float = 1.0
    margin_s: float = 1.0
    connect_timeout_s: float = 30.0


@dataclass
class Report:
    samples_sent: int = 0
    heartbeats_sent: int = 0
    images_sent: int = 0
    image_bytes: int = 0
    max_telemetry_late_ns: int = 0
    max_image_late_ns: int = 0
    image_log: list = field(default_factory=list)  # (seq, name, capture_ns, send_start_ns, send_end_ns)
    error: str = ""


class Clock:
    def __init__(self, start_ns, recorded0_ns, factor):
        self.start_ns = start_ns
        self.recorded0_ns = recorded0_ns
        self.factor = factor

    def emulated(self, recorded_ns):
        return self.start_ns + int(round((recorded_ns - self.recorded0_ns) / self.factor))

    @staticmethod
    def wait_until(t_ns, stop):
        while not stop.is_set():
            dt = t_ns - time.time_ns()
            if dt <= 0:
                return True
            stop.wait(min(dt / NS, 0.2))
        return False


def _telemetry_thread(opts, traj, times, clock, report, stop, images_sent):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    next_heartbeat = clock.start_ns
    try:
        for t_rec in times:
            t_emu = clock.emulated(t_rec)
            while next_heartbeat <= t_emu:
                if not Clock.wait_until(next_heartbeat, stop):
                    return
                sock.sendto(link.heartbeat(time.time_ns(), opts.session_id, report.samples_sent, images_sent[0], seq),
                            (opts.host, opts.udp_port))
                seq += 1
                report.heartbeats_sent += 1
                next_heartbeat += NS
            if not Clock.wait_until(t_emu, stop):
                return
            lat, lon, h, q, (h_acc, v_acc) = traj.sample(t_rec)
            sample = link.Telemetry(t_emu, t_rec, lat, lon, h, q, (h_acc, h_acc, v_acc), opts.attitude_std_deg,
                                    opts.position_fix, opts.session_id)
            report.max_telemetry_late_ns = max(report.max_telemetry_late_ns, time.time_ns() - t_emu)
            sock.sendto(link.telemetry(sample, seq), (opts.host, opts.udp_port))
            seq += 1
            report.samples_sent += 1
    finally:
        sock.close()


def _connect(opts, stop):
    deadline = time.monotonic() + opts.connect_timeout_s
    while not stop.is_set():
        try:
            return socket.create_connection((opts.host, opts.tcp_port), timeout=5.0)
        except OSError:
            if time.monotonic() > deadline:
                raise
            stop.wait(0.5)
    return None


def _image_thread(opts, flight, q_b_c, images, clock, report, stop, images_sent, sock):
    seq = 0
    try:
        cam = flight.camera
        sock.sendall(link.session_start(link.SessionStart(
            opts.session_id, clock.start_ns, clock.recorded0_ns, opts.time_factor, cam.width, cam.height, cam.k,
            cam.distortion, q_b_c, (0.0, 0.0, 0.0), opts.camera_name), seq))
        seq += 1
        for frame in images:
            t_capture = clock.emulated(frame.recorded_ns)
            data = frame.path.read_bytes()  # before the capture time, so disk time does not delay sending
            if not Clock.wait_until(t_capture, stop):
                return
            img = link.Image(opts.session_id, frame.image_seq, t_capture + opts.image_time_offset_ns,
                             frame.recorded_ns, data)
            t0 = time.time_ns()
            sock.sendall(link.image_head(img, seq))
            sock.sendall(data)
            t1 = time.time_ns()
            seq += 1
            report.max_image_late_ns = max(report.max_image_late_ns, t0 - t_capture)
            report.images_sent += 1
            report.image_bytes += len(data)
            images_sent[0] = report.images_sent
            report.image_log.append((frame.image_seq, frame.name, img.capture_ns, t0, t1))
        sock.sendall(link.session_end(opts.session_id, report.images_sent, time.time_ns(), seq))
    except OSError as e:
        report.error = f"image channel: {e}"
        stop.set()


def run(opts, flight, images, progress=None):
    """Sends the selected images of the flight and the telemetry around them.
    Returns a Report. progress(report) is called about once per second."""
    traj = Trajectory(flight.frames)
    margin = int(opts.margin_s * NS * opts.time_factor)
    t_first = images[0].recorded_ns
    t_last = images[-1].recorded_ns
    period = int(round(NS * opts.time_factor / opts.telemetry_rate_hz))
    extra = [f.recorded_ns for f in images] if opts.capture_samples else ()
    times = traj.sample_times(period, t_first - margin, t_last + margin, extra)

    report = Report()
    stop = threading.Event()
    images_sent = [0]
    sock = _connect(opts, stop)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # Telemetry starts at the first sample, before the first image by the margin.
    clock = Clock(time.time_ns() + int(opts.start_delay_s * NS) + int(opts.margin_s * NS), t_first,
                  opts.time_factor)

    threads = [
        threading.Thread(target=_telemetry_thread, args=(opts, traj, times, clock, report, stop, images_sent),
                         daemon=True),
        threading.Thread(target=_image_thread, args=(opts, flight, traj.q_b_c, images, clock, report, stop, images_sent, sock),
                         daemon=True),
    ]
    for t in threads:
        t.start()
    try:
        while any(t.is_alive() for t in threads):
            for t in threads:
                t.join(timeout=1.0)
            if progress is not None:
                progress(report)
    except KeyboardInterrupt:
        report.error = "interrupted"
        stop.set()
        for t in threads:
            t.join()
    finally:
        sock.close()
    return report
