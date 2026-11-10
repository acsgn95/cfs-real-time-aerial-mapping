# ADR-004: Coordinate Frames and Conventions

* Status: Accepted
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D04
* Evidence: thesis proposal (FB_YL-01) section 4.1; ADR-001 (R6, unlimited
  area); ADR-003 (telemetry channel); Dorfberg Flight 01 geotags and Metashape
  project (CH1903+ / LV95 with Swiss Geoid 2004); ground control point file
  (ellipsoidal height 1715.765 m against elevation 1665.612 m for point 1)

## Context

The proposal names WGS 84, ECEF and local ENU for georeferencing. ADR-001 rules
out a single local origin for the whole flight (R6) and requires unlimited
survey area. ADR-003 sends poses on a telemetry channel whose conventions are
still open.

The nominal dataset mixes references:

* geotags are WGS 84 latitude, longitude and ellipsoidal height;
* the Metashape project and the reference products use the Swiss projection
  CH1903+ / LV95 with heights from the Swiss geoid model;
* the geoid lies about 50 m below the ellipsoid at the site.

Mixing up height references, angle conventions or transform directions would
cause errors far larger than the accuracy being measured.

## Decision

### 1. Frames

| Symbol | Frame | Definition |
| --- | --- | --- |
| `C` | Camera | origin at the projection centre; x right, y down, z along the optical axis (OpenCV and COLMAP convention) |
| `B` | Body | FRD: x forward, y right, z down; origin at the point the telemetry position refers to |
| `N` | Navigation | local NED (north, east, down) at the vehicle position; used only for attitude on the telemetry channel |
| `E` | ECEF | Earth-centred, Earth-fixed, on the WGS 84 ellipsoid |
| `G` | Geodetic | latitude, longitude, ellipsoidal height on the WGS 84 ellipsoid |
| `L_k` | Local working frame | ENU tangent frame of keyframe batch `k`, with its origin at the reference camera of the batch |
| `P` | Product frame | projected CRS of the map products (EPSG code) plus a vertical datum |

### 2. Pose and rotation conventions

* `T_A_B` transforms points from frame `B` into frame `A`:
  `p_A = T_A_B * p_B`. A camera pose is stored as `T_W_C` (camera to world).
* Rotations are unit quaternions in the Hamilton convention, stored in the
  order (w, x, y, z), normalised to w ≥ 0.
* Lengths are in metres and angles in radians inside OrthoStream. Degrees
  appear only where an external format requires them (for example,
  latitude and longitude on the telemetry channel, or GeoTIFF metadata).
* Euler angles never appear on an OrthoStream interface. The injector
  converts dataset attitudes (yaw, pitch, roll) to quaternions.
* Coordinates that can be large (ECEF, projected, geodetic) are stored in
  double precision.

### 3. Telemetry channel

* Position: geodetic latitude and longitude in degrees, ellipsoidal height in
  metres (double precision).
* Attitude: quaternion `q_N_B` (body to local NED), as autopilot telemetry
  usually provides it.
* The camera-to-body transform `T_B_C` is sent once in the session start
  message (ADR-003).
* The injector provides body poses and a `T_B_C` that reproduce the
  dataset's camera poses exactly. For Dorfberg, this means the Metashape
  camera poses for `optimized` and the recorded geotags for `non-optimized`.

### 4. No global local frame; one working frame per keyframe batch

* There is no single ENU origin for the flight. Each keyframe batch (a
  reference keyframe and its source views) is processed in its own working
  frame `L_k`: ENU axes at the reference camera position.
* All poses of the batch are expressed in `L_k` before dense reconstruction.
  Inside `L_k`, single precision is allowed, because coordinates stay within
  a few kilometres of the origin.
* Results (depth maps converted to points) are transformed from `L_k` to `E`
  in double precision and from there to `P`.
* Any other component that needs a local Cartesian frame defines its own in
  the same way, for example per tile. Re-anchoring is therefore automatic, and
  ADR-001 R6 holds by construction.

### 5. Heights

* Internal geometry uses ellipsoidal heights.
* The vertical datum of the products is configurable: ellipsoidal by
  default, or orthometric through a geoid model given as a PROJ grid.
* For the Dorfberg evaluation, products are converted to the vertical
  reference of the reference DSM (Swiss geoid model) before comparison.

### 6. Product projection

* The product CRS is set per mission by an EPSG code. The default is the WGS 84
  UTM zone in which the mission starts.
* The same zone is kept for the whole mission, even when the flight crosses a
  zone boundary.
* The Dorfberg evaluation uses CH1903+ / LV95 (EPSG:2056), the CRS of the
  reference products.
* Map tiles (D07) are defined on a regular grid in `P`. The tile grid does not
  depend on any working frame.

### 7. Datum realisations and libraries

* All coordinates of one run are in the datum realisation of its data source.
  OrthoStream does not convert between realisations (for example ETRS89 and
  ITRF); this is consistent within a dataset.
* Geodetic, projection and vertical datum transformations use PROJ, which is
  already a GDAL dependency. Only the ECEF-to-ENU rotation is implemented
  directly, and it is tested against PROJ.

## Rationale

* The OpenCV and COLMAP camera convention avoids conversions at the dense
  reconstruction interface (D05). FRD and NED match autopilot telemetry.
* Per-batch working frames remove the flat-Earth error and the need for
  re-anchoring logic. They also keep single-precision GPU computation
  accurate, because coordinates stay small.
* Explicit transform naming and a single quaternion convention remove the most
  common sources of silent pose errors.
* Ellipsoidal heights inside, with configurable output datum, keep geometry
  independent of geoid models while still allowing comparison with national
  reference products.

## Alternatives Considered

* **One ENU origin, re-anchored when the vehicle moves too far.** Rejected:
  re-anchoring events add state and can cause seams. Per-batch frames give the
  same accuracy without them.
* **Computation directly in ECEF.** Rejected: single precision at ECEF
  magnitudes (about 6.4 × 10⁶ m) has a resolution of about 0.5 m, which is too
  coarse for GPU computation.
* **3D computation in projected coordinates (UTM).** Rejected: projected
  coordinates have a varying scale factor and mix map heights with Cartesian
  geometry.
* **Euler angles on interfaces.** Rejected: the order of rotations is
  ambiguous and they have singularities.
* **NED as the working frame.** Rejected: the proposal names ENU, and ENU
  matches the projected product frame (east, north, up).

## Affected Areas

* [x] simulator interface
* [x] coordinate frames and conventions
* [x] algorithm libraries
* [x] Software Bus messages
* [x] evaluation and experiments
* [x] documentation

## Impact

* **Injector (ADR-003):** converts dataset attitudes to `q_N_B` and provides a
  consistent `T_B_C`.
* **D05:** dense reconstruction runs in `L_k`. The world frame of the COLMAP
  modules is `L_k`.
* **D06:** footprint computation uses `T_W_C` and a terrain height estimate.
* **D07:** tiles are defined in `P`.
* **D15:** message fields follow sections 2 and 3.
* **D17:** evaluation converts products to EPSG:2056 with the Swiss geoid
  before comparison.

## Open Questions

* What is the exact vertical CRS code of the reference DSM? To be confirmed
  when the complete DSM file is read.
