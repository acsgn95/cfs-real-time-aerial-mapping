#!/usr/bin/env python3
"""Downloads the Copernicus DEM GLO-30 tiles of a mission area (ADR-006).

The terrain prior of keyframe selection is a pre-mission asset. This tool
fetches the 1 x 1 degree tiles that cover an area from the public AWS
Open Data bucket (no account needed), checks each tile's TIFF header, and
writes:

* tiles/<tile>.tif     the Cloud Optimized GeoTIFF tiles, unchanged;
* <name>.vrt           a GDAL virtual mosaic of the tiles;
* <name>.json          a manifest: area, tile URLs, sizes, SHA-256 sums,
                       retrieval time, vertical datum and attribution.

Heights are orthometric heights above the EGM2008 geoid (EPSG:3855) of a
surface model (vegetation and buildings included). OrthoStream converts
them to ellipsoidal heights with the PROJ grid us_nga_egm08_25.tif
(scripts/fetch_proj_grids.sh); the CRS of the mosaic is EPSG:4326+3855.

Areas without a tile (oceans, and a few countries not released to the
public) have no data in the mosaic.

Usage:
    fetch_dem.py --bbox SOUTH WEST NORTH EAST [--out DIR] [--name NAME]
    fetch_dem.py --poses poses.txt [--margin-km KM] [--out DIR] [--name NAME]

--poses takes the camera file of tools/dataset/extract_metashape_poses.py
and covers the camera positions plus a margin. Only the Python standard
library is used.

Data: Copernicus DEM GLO-30, (c) DLR e.V. 2010-2014 and (c) Airbus Defence
and Space GmbH 2014-2018, provided under COPERNICUS by the European Union
and ESA. Products derived from it must carry an attribution (see the
manifest).
"""

import argparse
import datetime
import hashlib
import json
import math
import struct
import sys
import urllib.error
import urllib.request
from pathlib import Path
from xml.sax.saxutils import escape

BUCKET = "https://copernicus-dem-30m.s3.amazonaws.com"
ATTRIBUTION = (
    "Produced using Copernicus WorldDEM-30 (c) DLR e.V. 2010-2014 and "
    "(c) Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS "
    "by the European Union and ESA; all rights reserved"
)
NODATA = -32767.0


def tile_name(lat, lon):
    ns = "N" if lat >= 0 else "S"
    ew = "E" if lon >= 0 else "W"
    return f"Copernicus_DSM_COG_10_{ns}{abs(lat):02d}_00_{ew}{abs(lon):03d}_00_DEM"


def ecef_to_geodetic(x, y, z):
    """WGS 84 latitude and longitude in degrees (iterative)."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(10):
        n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - e2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon)


def bbox_from_poses(path, margin_km):
    lats, lons = [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            t = line.split()
            if t and t[0] == "pose":
                lat, lon = ecef_to_geodetic(float(t[2]), float(t[3]), float(t[4]))
                lats.append(lat)
                lons.append(lon)
    if not lats:
        sys.exit(f"no poses in {path}")
    dlat = margin_km / 111.32
    dlon = margin_km / (111.32 * math.cos(math.radians(sum(lats) / len(lats))))
    return min(lats) - dlat, min(lons) - dlon, max(lats) + dlat, max(lons) + dlon


def tiff_georeference(data):
    """Width, height and GDAL geotransform from a GeoTIFF header."""
    order = {b"II": "<", b"MM": ">"}.get(data[:2])
    if order is None or struct.unpack(order + "H", data[2:4])[0] != 42:
        raise ValueError("not a classic TIFF file")
    (ifd,) = struct.unpack(order + "I", data[4:8])
    (count,) = struct.unpack(order + "H", data[ifd:ifd + 2])
    sizes = {1: 1, 2: 1, 3: 2, 4: 4, 12: 8, 16: 8}
    codes = {1: "B", 2: "c", 3: "H", 4: "I", 12: "d", 16: "Q"}
    tags = {}
    for i in range(count):
        entry = data[ifd + 2 + 12 * i: ifd + 14 + 12 * i]
        tag, typ, n = struct.unpack(order + "HHI", entry[:8])
        if typ not in sizes:
            continue
        size = sizes[typ] * n
        if size <= 4:
            raw = entry[8:8 + size]
        else:
            (off,) = struct.unpack(order + "I", entry[8:12])
            raw = data[off:off + size]
        tags[tag] = struct.unpack(order + codes[typ] * n, raw)
    width, height = tags[256][0], tags[257][0]
    sx, sy, _ = tags[33550]
    _, _, _, x0, y0, _ = tags[33922][:6]
    keys = tags.get(34735, ())
    raster_type = 1
    for k in range(4, len(keys), 4):
        if keys[k] == 1025:  # GTRasterTypeGeoKey: 1 area, 2 point
            raster_type = keys[k + 3]
    if raster_type == 2:  # tie point at a pixel centre
        x0 -= sx / 2.0
        y0 += sy / 2.0
    return width, height, (x0, sx, 0.0, y0, 0.0, -sy)


def fetch(url):
    with urllib.request.urlopen(url, timeout=120) as r:
        return r.read()


def write_vrt(path, tiles):
    res_x = min(t["geotransform"][1] for t in tiles)
    res_y = min(-t["geotransform"][5] for t in tiles)
    west = min(t["geotransform"][0] for t in tiles)
    north = max(t["geotransform"][3] for t in tiles)
    east = max(t["geotransform"][0] + t["width"] * t["geotransform"][1] for t in tiles)
    south = min(t["geotransform"][3] + t["height"] * t["geotransform"][5] for t in tiles)
    width = round((east - west) / res_x)
    height = round((north - south) / res_y)
    lines = [
        f'<VRTDataset rasterXSize="{width}" rasterYSize="{height}">',
        '  <SRS dataAxisToSRSAxisMapping="2,1">EPSG:4326</SRS>',
        f"  <GeoTransform>{west!r}, {res_x!r}, 0.0, {north!r}, 0.0, {-res_y!r}</GeoTransform>",
        '  <Metadata><MDI key="VERTICAL_CRS">EPSG:3855 (EGM2008 height)</MDI>'
        f'<MDI key="ATTRIBUTION">{escape(ATTRIBUTION)}</MDI></Metadata>',
        '  <VRTRasterBand dataType="Float32" band="1">',
        f"    <NoDataValue>{NODATA}</NoDataValue>",
    ]
    for t in tiles:
        gt = t["geotransform"]
        dx = (gt[0] - west) / res_x
        dy = (north - gt[3]) / res_y
        dw = t["width"] * gt[1] / res_x
        dh = t["height"] * -gt[5] / res_y
        lines += [
            "    <SimpleSource>",
            f'      <SourceFilename relativeToVRT="1">tiles/{t["file"]}</SourceFilename>',
            "      <SourceBand>1</SourceBand>",
            f'      <SrcRect xOff="0" yOff="0" xSize="{t["width"]}" ySize="{t["height"]}"/>',
            f'      <DstRect xOff="{dx!r}" yOff="{dy!r}" xSize="{dw!r}" ySize="{dh!r}"/>',
            "    </SimpleSource>",
        ]
    lines += ["  </VRTRasterBand>", "</VRTDataset>", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    area = ap.add_mutually_exclusive_group(required=True)
    area.add_argument("--bbox", nargs=4, type=float, metavar=("SOUTH", "WEST", "NORTH", "EAST"))
    area.add_argument("--poses", type=Path, help="camera file; covers the camera positions")
    ap.add_argument("--margin-km", type=float, default=2.0, help="margin around the poses (default 2 km)")
    ap.add_argument("--out", type=Path, default=Path("data/raw/dem/cop30"))
    ap.add_argument("--name", default="dem")
    args = ap.parse_args()

    south, west, north, east = args.bbox if args.bbox else bbox_from_poses(args.poses, args.margin_km)
    if not (-90 <= south < north <= 90 and -180 <= west < east <= 180):
        sys.exit("invalid area")
    tile_dir = args.out / "tiles"
    tile_dir.mkdir(parents=True, exist_ok=True)

    tiles, missing = [], []
    for lat in range(math.floor(south), math.ceil(north)):
        for lon in range(math.floor(west), math.ceil(east)):
            name = tile_name(lat, lon)
            url = f"{BUCKET}/{name}/{name}.tif"
            target = tile_dir / f"{name}.tif"
            if target.exists():
                data = target.read_bytes()
                state = "cached"
            else:
                try:
                    data = fetch(url)
                except urllib.error.HTTPError as e:
                    if e.code in (403, 404):
                        missing.append(name)
                        print(f"no tile  {name}")
                        continue
                    raise
                target.with_suffix(".part").write_bytes(data)
                target.with_suffix(".part").rename(target)
                state = "fetched"
            width, height, gt = tiff_georeference(data)
            tiles.append({
                "file": target.name,
                "url": url,
                "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
                "width": width,
                "height": height,
                "geotransform": gt,
            })
            print(f"{state:8} {name}  {width} x {height}")
    if not tiles:
        sys.exit("no tiles cover the area")

    vrt = args.out / f"{args.name}.vrt"
    write_vrt(vrt, tiles)
    manifest = {
        "dataset": "Copernicus DEM GLO-30 Public (DGED), AWS Open Data",
        "source": BUCKET,
        "retrieved": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "area": {"south": south, "west": west, "north": north, "east": east},
        "crs": "EPSG:4326+3855",
        "vertical_datum": "EGM2008 (EPSG:3855); convert with PROJ grid us_nga_egm08_25.tif",
        "surface": "digital surface model (vegetation and buildings included)",
        "attribution": ATTRIBUTION,
        "mosaic": vrt.name,
        "tiles": tiles,
        "missing_tiles": missing,
    }
    (args.out / f"{args.name}.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"mosaic   {vrt}")


if __name__ == "__main__":
    main()
