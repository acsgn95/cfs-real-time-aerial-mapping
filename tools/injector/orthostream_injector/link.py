"""Encoder of the injector link (ADR-016 section 7).

The layout is interfaces/link/ost_link_format.h; docs/interfaces/link.md
describes it. tests/interfaces/check_interfaces.py compares the sizes and
offsets below with the C header, and interfaces/link/test_vectors.txt holds
byte vectors that both this module and libs/link are tested against.
"""

import struct
from dataclasses import dataclass, field

MAGIC = b"OSTL"
VERSION = 1

TYPE_TELEMETRY = 1
TYPE_HEARTBEAT = 2
TYPE_SESSION_START = 16
TYPE_IMAGE = 17
TYPE_SESSION_END = 18

IMAGE_FORMAT_JPEG = 1
CAMERA_NAME_LEN = 32

# Little-endian, packed ("<" disables alignment). Field order as in the C header.
HEADER = struct.Struct("<4sHHII")
TELEMETRY = struct.Struct("<qqddd4d3f3fBBHI")
HEARTBEAT = struct.Struct("<qIIII")
SESSION_START = struct.Struct("<IIqqdII4d5d4d3d32s")
IMAGE = struct.Struct("<IIqqB3xI")
SESSION_END = struct.Struct("<IIq")

# Payload sizes, checked against OST_LINK_*_SIZE.
SIZES = {
    "HEADER": HEADER.size,
    "TELEMETRY": TELEMETRY.size,
    "HEARTBEAT": HEARTBEAT.size,
    "SESSION_START": SESSION_START.size,
    "IMAGE": IMAGE.size,
    "SESSION_END": SESSION_END.size,
}


@dataclass
class Telemetry:
    time_ns: int
    recorded_ns: int
    latitude_deg: float
    longitude_deg: float
    height_m: float
    q_n_b: tuple  # w x y z
    std_position_m: tuple = (0.0, 0.0, 0.0)  # north east down
    std_attitude_deg: tuple = (0.0, 0.0, 0.0)  # roll pitch yaw
    fix: int = 0
    session_id: int = 0


@dataclass
class SessionStart:
    session_id: int
    start_ns: int
    recorded_start_ns: int
    time_factor: float
    width: int
    height: int
    k: tuple  # fx fy cx cy
    distortion: tuple  # k1 k2 k3 p1 p2
    q_b_c: tuple  # w x y z
    t_b_c: tuple = (0.0, 0.0, 0.0)
    camera_name: str = ""


@dataclass
class Image:
    session_id: int
    image_seq: int
    capture_ns: int
    recorded_ns: int
    data: bytes = field(repr=False)


def header(msg_type, length, sequence):
    return HEADER.pack(MAGIC, VERSION, msg_type, length, sequence & 0xFFFFFFFF)


def telemetry(t, sequence):
    payload = TELEMETRY.pack(t.time_ns, t.recorded_ns, t.latitude_deg, t.longitude_deg, t.height_m,
                             *t.q_n_b, *t.std_position_m, *t.std_attitude_deg, t.fix, 0, 0, t.session_id)
    return header(TYPE_TELEMETRY, len(payload), sequence) + payload


def heartbeat(time_ns, session_id, samples_sent, images_sent, sequence):
    payload = HEARTBEAT.pack(time_ns, session_id, samples_sent, images_sent, 0)
    return header(TYPE_HEARTBEAT, len(payload), sequence) + payload


def session_start(s, sequence):
    name = s.camera_name.encode("ascii")[:CAMERA_NAME_LEN]
    payload = SESSION_START.pack(s.session_id, 0, s.start_ns, s.recorded_start_ns, s.time_factor, s.width,
                                 s.height, *s.k, *s.distortion, *s.q_b_c, *s.t_b_c, name)
    return header(TYPE_SESSION_START, len(payload), sequence) + payload


def image_head(img, sequence):
    """Header and fixed part of an IMAGE frame; the image data follows."""
    fixed = IMAGE.pack(img.session_id, img.image_seq, img.capture_ns, img.recorded_ns, IMAGE_FORMAT_JPEG,
                       len(img.data))
    return header(TYPE_IMAGE, len(fixed) + len(img.data), sequence) + fixed


def session_end(session_id, images_sent, time_ns, sequence):
    payload = SESSION_END.pack(session_id, images_sent, time_ns)
    return header(TYPE_SESSION_END, len(payload), sequence) + payload
