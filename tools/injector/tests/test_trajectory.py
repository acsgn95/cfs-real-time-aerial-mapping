import math
import unittest

from orthostream_injector import geodesy
from orthostream_injector.dataset import Frame
from orthostream_injector.trajectory import Q_B_C_NOMINAL, Trajectory

NS = 1_000_000_000


def make_frames():
    """Straight flight north at 15 m/s with a slowly changing yaw."""
    frames = []
    lat0, lon0, h = math.radians(46.82), math.radians(9.83), 2580.0
    for k in range(6):
        north = 15.0 * 1.5 * k
        lat = lat0 + north / 6.371e6
        p = geodesy.geodetic_to_ecef(lat, lon0, h)
        yaw = math.radians(10.0 + 2.0 * k)
        q_n_b = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        q_e_b = geodesy.q_mul(geodesy.q_conj(geodesy.q_n_e(lat, lon0)), q_n_b)
        q_e_c = geodesy.q_normalise(geodesy.q_mul(q_e_b, Q_B_C_NOMINAL))
        frames.append(Frame(f"img{k}.jpg", None, k, 1_600_000_000 * NS + int(1.5 * k * NS), p, q_e_c, 0.03, 0.05))
    return frames


class GeodesyTest(unittest.TestCase):
    def test_round_trip(self):
        for lat, lon, h in [(46.82, 9.83, 2580.0), (-33.9, 151.2, 50.0), (89.9, -120.0, 10000.0)]:
            p = geodesy.geodetic_to_ecef(math.radians(lat), math.radians(lon), h)
            la, lo, hh = geodesy.ecef_to_geodetic(*p)
            self.assertAlmostEqual(math.degrees(la), lat, places=10)
            self.assertAlmostEqual(math.degrees(lo), lon, places=10)
            self.assertAlmostEqual(hh, h, places=4)

    def test_log_exp(self):
        q = geodesy.q_normalise((0.9, 0.1, -0.2, 0.3))
        r = geodesy.q_exp(geodesy.q_log(q))
        self.assertLess(geodesy.q_angle_deg(q, r), 1e-9)


class TrajectoryTest(unittest.TestCase):
    def test_passes_through_the_image_poses(self):
        frames = make_frames()
        traj = Trajectory(frames)
        for f in frames:
            lat, lon, h, q_n_b, acc = traj.sample(f.recorded_ns)
            p = geodesy.geodetic_to_ecef(math.radians(lat), math.radians(lon), h)
            self.assertLess(math.dist(p, f.position_ecef), 1e-6)
            q_e_b = geodesy.q_mul(geodesy.q_conj(geodesy.q_n_e(math.radians(lat), math.radians(lon))), q_n_b)
            q_e_c = geodesy.q_mul(q_e_b, Q_B_C_NOMINAL)
            self.assertLess(geodesy.q_angle_deg(q_e_c, f.q_e_c), 1e-7)
            self.assertEqual(acc, (0.03, 0.05))

    def test_between_images(self):
        frames = make_frames()
        traj = Trajectory(frames)
        t = (frames[2].recorded_ns + frames[3].recorded_ns) // 2
        lat, lon, h, q_n_b, _ = traj.sample(t)
        self.assertAlmostEqual(h, 2580.0, places=2)
        yaw = math.degrees(2.0 * math.atan2(q_n_b[3], q_n_b[0]))
        self.assertAlmostEqual(yaw, 15.0, places=3)  # constant yaw rate: midway

    def test_sample_times(self):
        traj = Trajectory(make_frames())
        times = traj.sample_times(NS // 10, extra=[traj.start_ns + 5])
        self.assertEqual(times[0], traj.start_ns)
        self.assertIn(traj.start_ns + 5, times)
        self.assertLessEqual(times[-1], traj.end_ns)
        self.assertEqual(len(times), 77)  # 7.5 s at 10 Hz plus one extra


if __name__ == "__main__":
    unittest.main()
