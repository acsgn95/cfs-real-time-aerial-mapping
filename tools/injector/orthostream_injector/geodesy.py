"""WGS 84 conversions and Hamilton quaternions (ADR-004), standard library only.

Quaternions are tuples (w, x, y, z). q_A_B rotates vectors from frame B to
frame A: v_A = q_A_B * v_B * conj(q_A_B).
"""

import math

A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563
E2 = F_WGS84 * (2.0 - F_WGS84)
B_WGS84 = A_WGS84 * (1.0 - F_WGS84)
EP2 = E2 / (1.0 - E2)


def geodetic_to_ecef(lat, lon, h):
    """Latitude and longitude in radians, ellipsoidal height in metres."""
    n = A_WGS84 / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon), (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - E2) + h) * math.sin(lat))


def ecef_to_geodetic(x, y, z):
    """Returns latitude and longitude in radians and the ellipsoidal height.

    Bowring's start value followed by Newton iterations on the latitude;
    sub-millimetre for aircraft heights after two iterations.
    """
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    theta = math.atan2(z * A_WGS84, p * B_WGS84)
    lat = math.atan2(z + EP2 * B_WGS84 * math.sin(theta) ** 3, p - E2 * A_WGS84 * math.cos(theta) ** 3)
    for _ in range(3):
        n = A_WGS84 / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - E2 * n / (n + h)))
    n = A_WGS84 / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return lat, lon, h


def r_e_n(lat, lon):
    """Rotation matrix from local north-east-down to ECEF."""
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return [[-sl * co, -so, -cl * co], [-sl * so, co, -cl * so], [cl, 0.0, -sl]]


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_normalise(q):
    n = math.sqrt(sum(v * v for v in q))
    q = tuple(v / n for v in q)
    return tuple(-v for v in q) if q[0] < 0 else q


def q_from_matrix(r):
    """Quaternion with w >= 0 of a rotation matrix."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        q = ((r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s)
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        q = ((r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s)
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        q = ((r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s)
    return q_normalise(q)


def q_n_e(lat, lon):
    """Rotation from ECEF to local NED at the given position."""
    return q_conj(q_from_matrix(r_e_n(lat, lon)))


def q_slerp(a, b, u):
    """Spherical linear interpolation from a (u = 0) to b (u = 1)."""
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-v for v in b)
        dot = -dot
    if dot > 0.9999999:
        return q_normalise(tuple(x + u * (y - x) for x, y in zip(a, b)))
    theta = math.acos(min(1.0, dot))
    s = math.sin(theta)
    wa = math.sin((1.0 - u) * theta) / s
    wb = math.sin(u * theta) / s
    return q_normalise(tuple(wa * x + wb * y for x, y in zip(a, b)))


def q_angle_deg(a, b):
    """Rotation angle between two attitudes in degrees."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def q_log(q):
    """Rotation vector (axis times angle, radians) of a unit quaternion."""
    w, x, y, z = q
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-12:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    angle = 2.0 * math.atan2(s, w)
    return (x / s * angle, y / s * angle, z / s * angle)


def q_exp(v):
    """Unit quaternion of a rotation vector."""
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-12:
        return q_normalise((1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]))
    s = math.sin(0.5 * angle) / angle
    return (math.cos(0.5 * angle), v[0] * s, v[1] * s, v[2] * s)
