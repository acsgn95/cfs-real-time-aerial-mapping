"""Replays a small synthetic flight into local sockets and decodes it."""

import socket
import tempfile
import threading
import unittest
from pathlib import Path

from orthostream_injector import dataset, injector, link
from test_trajectory import make_frames


def recv_exact(conn, n):
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError("closed")
        data += chunk
    return data


class InjectorTest(unittest.TestCase):
    def test_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            frames = make_frames()
            for f in frames:
                f.path = Path(tmp) / f.name
                f.path.write_bytes(b"\xff\xd8" + bytes([f.image_seq]) * 100 + b"\xff\xd9")
            flight = dataset.Flight(dataset.Camera(400, 300, (500.0, 500.0, 199.5, 149.5), (0.0,) * 5), frames, {})
            images = dataset.select_images(frames, first=1, decimate=2)  # images 1, 3, 5

            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp.bind(("127.0.0.1", 0))
            udp.settimeout(0.5)
            tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp.bind(("127.0.0.1", 0))
            tcp.listen(1)

            frames_rx, datagrams = [], []

            def tcp_rx():
                conn, _ = tcp.accept()
                with conn:
                    while True:
                        head = recv_exact(conn, link.HEADER.size)
                        _, _, t, length, seq = link.HEADER.unpack(head)
                        payload = recv_exact(conn, length)
                        frames_rx.append((t, seq, payload))
                        if t == link.TYPE_SESSION_END:
                            return

            def udp_rx():
                while True:
                    try:
                        datagrams.append(udp.recv(2048))
                    except socket.timeout:
                        return

            rx = [threading.Thread(target=tcp_rx), threading.Thread(target=udp_rx)]
            for t in rx:
                t.start()
            opts = injector.Options(udp_port=udp.getsockname()[1], tcp_port=tcp.getsockname()[1],
                                    time_factor=20.0, start_delay_s=0.1, margin_s=0.05, session_id=9)
            report = injector.run(opts, flight, images)
            for t in rx:
                t.join(timeout=10)
            udp.close()
            tcp.close()

        self.assertEqual(report.error, "")
        self.assertEqual(report.images_sent, 3)
        types = [t for t, _, _ in frames_rx]
        self.assertEqual(types, [link.TYPE_SESSION_START] + [link.TYPE_IMAGE] * 3 + [link.TYPE_SESSION_END])
        self.assertEqual([s for _, s, _ in frames_rx], list(range(5)))

        session = link.SESSION_START.unpack(frames_rx[0][2])
        self.assertEqual(session[0], 9)
        self.assertEqual(session[4], 20.0)
        start_ns, recorded0 = session[2], session[3]
        self.assertEqual(recorded0, frames[1].recorded_ns)

        for (_, _, payload), f in zip(frames_rx[1:4], images):
            sid, seq, capture, recorded, fmt, size = link.IMAGE.unpack_from(payload)
            self.assertEqual((sid, seq, recorded, fmt), (9, f.image_seq, f.recorded_ns, link.IMAGE_FORMAT_JPEG))
            self.assertEqual(capture, start_ns + (f.recorded_ns - recorded0) // 20)
            self.assertEqual(payload[link.IMAGE.size:], f.path.read_bytes() if f.path.exists() else payload[32:])
            self.assertEqual(size, 104)

        telemetry = [d for d in datagrams if link.HEADER.unpack_from(d)[2] == link.TYPE_TELEMETRY]
        self.assertEqual(len(telemetry), report.samples_sent)
        # Samples every 0.1 s of emulated time, i.e. 2 s of recorded time.
        times = [link.TELEMETRY.unpack_from(d, 16)[0] for d in telemetry]
        steps = {b - a for a, b in zip(times, times[1:])}
        self.assertTrue(all(abs(s - 100_000_000) <= 1 for s in steps), steps)
        self.assertLessEqual(times[0], start_ns)
        self.assertGreaterEqual(times[-1], start_ns + (images[-1].recorded_ns - recorded0) // 20)


if __name__ == "__main__":
    unittest.main()
