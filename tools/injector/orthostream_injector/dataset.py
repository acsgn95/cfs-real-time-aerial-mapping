"""Recorded flight for the injector (ADR-003).

A flight is three inputs:

* a pose file (tools/dataset/extract_metashape_poses.py or
  extract_geotag_poses.py): the camera line and one camera pose per image,
  camera centre in ECEF and q_E_C; the file selects the pose set
  (`optimized` or `non-optimized`, ADR-003 section 4);
* a capture time file (tools/dataset/extract_capture_times.py): capture
  time and reported position accuracy per image;
* the image directory with the JPEG files.

Images present in all three are used, in the order of their capture times.
All of them define the trajectory of the telemetry; select_images() chooses
the images that are sent (ADR-003 section 6: first image, count, decimation),
so decimation removes images but not the flight.
"""

import re
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Camera:
    width: int
    height: int
    k: tuple  # fx fy cx cy
    distortion: tuple  # k1 k2 k3 p1 p2


@dataclass
class Frame:
    name: str
    path: Path
    image_seq: int
    recorded_ns: int
    position_ecef: tuple
    q_e_c: tuple
    h_accuracy_m: float
    v_accuracy_m: float


@dataclass
class Flight:
    camera: Camera
    frames: list
    skipped: dict  # reason -> number of images


def read_pose_file(path):
    camera, poses = None, {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if not parts or parts[0].startswith("#"):
                continue
            if parts[0] == "camera":
                v = [float(x) for x in parts[3:]]
                camera = Camera(int(parts[1]), int(parts[2]), tuple(v[0:4]), tuple(v[4:9]))
            elif parts[0] == "pose":
                v = [float(x) for x in parts[2:9]]
                poses[parts[1]] = (tuple(v[0:3]), tuple(v[3:7]))
    if camera is None:
        raise ValueError(f"{path}: no camera line")
    return camera, poses


def read_capture_file(path):
    captures = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if not parts or parts[0] != "capture":
                continue
            captures[parts[1]] = (int(parts[2]) * 1_000_000, float(parts[3]), float(parts[4]))
    return captures


def image_number(name, fallback):
    """The last group of digits in the file name, or the given number."""
    digits = re.findall(r"\d+", Path(name).stem)
    return int(digits[-1]) if digits else fallback


def load_flight(pose_file, capture_file, image_dir):
    camera, poses = read_pose_file(pose_file)
    captures = read_capture_file(capture_file)
    image_dir = Path(image_dir)

    skipped = {"no pose": 0, "no image file": 0}
    frames = []
    for name, (t_ns, h_acc, v_acc) in sorted(captures.items(), key=lambda kv: kv[1][0]):
        if name not in poses:
            skipped["no pose"] += 1
            continue
        path = image_dir / name
        if not path.is_file():
            skipped["no image file"] += 1
            continue
        position, q = poses[name]
        frames.append(Frame(name, path, image_number(name, len(frames)), t_ns, position, q, h_acc, v_acc))
    if len(frames) < 2:
        raise ValueError("fewer than two usable images")
    return Flight(camera, frames, skipped)


def select_images(frames, first=0, count=None, decimate=1):
    """Images first, first + decimate, ...; at most count of them."""
    if first < 0 or decimate < 1:
        raise ValueError("first must be >= 0 and decimate >= 1")
    selected = frames[first::decimate]
    if count is not None:
        selected = selected[:count]
    if not selected:
        raise ValueError("no image selected")
    return selected
