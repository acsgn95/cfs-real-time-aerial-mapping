"""Continuous trajectory from the per-image poses (ADR-003 section 3).

The camera poses of the images are turned into body poses with the
camera-to-body transform, interpolated, and converted to what an autopilot
reports: geodetic position and q_N_B.

The interpolation is a cubic Hermite curve with Catmull-Rom tangents
(non-uniform times): for the position in ECEF, and for the attitude in the
rotation vector of q_E_B relative to the pose at the start of the segment.
It passes through every image pose and has a continuous velocity and
angular rate. A piecewise linear curve would change the angular rate
abruptly at every image; a receiver that interpolates linearly between
10 Hz samples would then be 0.042 degrees off at the capture times (median
on Dorfberg Flight 01, 0.45 degrees at most), an error of the emulation and
not of a real aircraft. With this curve: 0.0038 degrees median, 0.048 at
most, in the turns (`--dry-run`, 2026-11-14).

The recorded positions are camera positions; the body origin is placed at
the camera (t_B_C = 0), so the positions are sent unchanged.
"""

import bisect
import math

from . import geodesy

# Nominal camera mounting of the WingtraOne (tools/dataset/extract_geotag_poses.py):
# camera x along body y (right wing), camera y along -body x, camera z down.
R_B_C_NOMINAL = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
Q_B_C_NOMINAL = geodesy.q_from_matrix(R_B_C_NOMINAL)


class Trajectory:
    def __init__(self, frames, q_b_c=Q_B_C_NOMINAL):
        self.q_b_c = q_b_c
        self.t = [f.recorded_ns for f in frames]
        self.p = [f.position_ecef for f in frames]
        q_c_b = geodesy.q_conj(q_b_c)
        self.q_e_b = [geodesy.q_normalise(geodesy.q_mul(f.q_e_c, q_c_b)) for f in frames]
        self.accuracy = [(f.h_accuracy_m, f.v_accuracy_m) for f in frames]
        if any(b <= a for a, b in zip(self.t, self.t[1:])):
            raise ValueError("frame times are not strictly increasing")

    @property
    def start_ns(self):
        return self.t[0]

    @property
    def end_ns(self):
        return self.t[-1]

    def sample(self, t_ns):
        """Body pose at recorded time t_ns: (lat_deg, lon_deg, h, q_n_b,
        (h_acc, v_acc)). Times outside the frames are clamped."""
        i = bisect.bisect_right(self.t, t_ns) - 1
        i = max(0, min(i, len(self.t) - 2))
        u = (t_ns - self.t[i]) / (self.t[i + 1] - self.t[i])
        u = max(0.0, min(1.0, u))
        p, q_e_b = self._hermite(i, u)
        lat, lon, h = geodesy.ecef_to_geodetic(*p)
        q_n_b = geodesy.q_normalise(geodesy.q_mul(geodesy.q_n_e(lat, lon), q_e_b))
        accuracy = self.accuracy[i] if u < 1.0 else self.accuracy[i + 1]
        return math.degrees(lat), math.degrees(lon), h, q_n_b, accuracy

    def _tangents(self, i):
        """Catmull-Rom velocity (m/s per ns) and angular rate (rad per ns, in
        the frame of pose i) at frame i; one-sided at the ends."""
        a = max(0, i - 1)
        b = min(len(self.t) - 1, i + 1)
        dt = self.t[b] - self.t[a]
        v = tuple((pb - pa) / dt for pa, pb in zip(self.p[a], self.p[b]))
        r = geodesy.q_log(geodesy.q_mul(geodesy.q_conj(self.q_e_b[a]), self.q_e_b[b]))
        return v, tuple(c / dt for c in r)

    def _hermite(self, i, u):
        dt = self.t[i + 1] - self.t[i]
        h00 = 2 * u ** 3 - 3 * u ** 2 + 1
        h10 = u ** 3 - 2 * u ** 2 + u
        h01 = -2 * u ** 3 + 3 * u ** 2
        h11 = u ** 3 - u ** 2
        v0, w0 = self._tangents(i)
        v1, w1 = self._tangents(i + 1)
        p = tuple(h00 * a + h10 * dt * va + h01 * b + h11 * dt * vb
                  for a, b, va, vb in zip(self.p[i], self.p[i + 1], v0, v1))
        # Rotation vector from pose i: 0 at u = 0, r at u = 1. The angular
        # rates of both ends are taken in the frame of pose i, which is
        # exact to first order for the small rotations between images.
        r = geodesy.q_log(geodesy.q_mul(geodesy.q_conj(self.q_e_b[i]), self.q_e_b[i + 1]))
        rv = tuple(h10 * dt * a + h01 * c + h11 * dt * b for a, b, c in zip(w0, w1, r))
        return p, geodesy.q_normalise(geodesy.q_mul(self.q_e_b[i], geodesy.q_exp(rv)))

    def sample_times(self, period_ns, start_ns=None, end_ns=None, extra=()):
        """Sample times every period_ns from start_ns to end_ns (default: the
        first and last frame), merged with the given extra times inside that
        range. Outside the frames, sample() holds the first or last pose."""
        start = self.t[0] if start_ns is None else start_ns
        end = self.t[-1] if end_ns is None else end_ns
        times = list(range(start, end + 1, period_ns))
        times += [t for t in extra if start <= t <= end]
        return sorted(set(times))
