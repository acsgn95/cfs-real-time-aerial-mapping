# ADR-008: Offline Reference Products and Camera Model Conventions

* Status: Accepted
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D08
* Revised: 2026-11-11 (verification criterion corrected; Metashape chunk frame is
  CH1903+ geocentric; both found with the GCP prototype)
* Evidence: thesis proposal (FB_YL-01) section 4.3; Dorfberg Flight 01
  Metashape processing report (2020-11-10) and project (`chunk.zip`); complete
  reference DSM checked against the five ground control points (ADR-004);
  file sizes of the Metashape dense cloud locally (80 MB) and on the data
  server (10.6 GB)

## Context

The proposal compares the real-time results with an offline COLMAP
reconstruction of the same images and with the simulator's reference surface.

The nominal dataset already contains an offline photogrammetric
reconstruction of the same flight. It was made with Agisoft Metashape, using
the same images, bundle adjustment with ground control points, a 10 cm DSM and
a 3 cm orthophoto. Running COLMAP offline as well would add a second
reconstruction pipeline that the thesis does not need.

The injector and the dense reconstruction use the Metashape camera
calibration. Metashape, OpenCV and COLMAP use different calibration and pixel
conventions, and ADR-004 does not define them.

## Decision

### 1. Metashape products are the offline reference

* The offline reference for the nominal dataset is the Metashape
  reconstruction of the same flight:

  | Product | Use |
  | --- | --- |
  | DSM, 10 cm, EPSG:2056 with LHN95 heights | reference for DSM accuracy and for point cloud accuracy |
  | Orthophoto, 3 cm | reference for orthomosaic accuracy and quality |
  | Five GNSS ground control points | the only fully independent check |

* No offline COLMAP reconstruction is run.
* The Metashape dense cloud is not used. The local copy is incomplete, and
  its format can only be read by Metashape. Point cloud accuracy is measured
  as the vertical distance from each point to the reference DSM.
* The orthophoto (10 GB) is downloaded when the orthomosaic evaluation
  starts.

This departs from the proposal in two ways, and both need the supervisor's
agreement:

* the offline reference is Metashape instead of COLMAP;
* the same Metashape products serve as both the "offline reconstruction" and
  the "reference surface". The thesis must state that they are not an
  independent ground truth. Only the ground control points are independent.

### 2. Camera model and pixel conventions

* **Internal convention: OpenCV.** Pixel coordinates have their origin at the
  centre of the top-left pixel, (0, 0). The camera model is the OpenCV
  pinhole model with radial (k1, k2, k3) and tangential (p1, p2) distortion.
* **From Metashape, in the injector:**
  * `fx = fy = f`;
  * `cx = W / 2 + cx_metashape - 0.5` and `cy = H / 2 + cy_metashape - 0.5`.
    Metashape gives the principal point as an offset from the image centre
    and places the centre of the top-left pixel at (0.5, 0.5).
  * `k1, k2, k3` are taken as they are;
  * the tangential terms are swapped: `p1 = P2_metashape` and
    `p2 = P1_metashape`.
* **To COLMAP, at the dense reconstruction boundary (ADR-005):** add 0.5 to
  `cx` and `cy`, because COLMAP places the centre of the top-left pixel at
  (0.5, 0.5). The OpenCV model maps to COLMAP's `OPENCV` (k3 = 0) or
  `FULL_OPENCV` camera model.
* **Verification** (25 marker observations: five ground control points,
  each marked in five images):
  1. Projecting with the converted OpenCV model gives the same pixel
     positions as the Metashape model (within 1e-6 px).
  2. Each marker, triangulated from its observations, reprojects with the
     per-marker error of the Metashape report (0.18–0.44 px) within 0.01 px.
  3. The difference between the triangulated and the surveyed position
     matches the per-point check point errors of the report within about
     1.5 cm.
  4. Projecting the surveyed positions directly gives about 2 px RMS. This
     is reported, not used as a threshold: the five points were check points
     (not used in the bundle adjustment) with 9.6 cm RMS error in the
     Metashape model, and 1 px is 3.3 cm on the ground.

  A prototype reproduced items 1–4 on 2026-11-11.

### 3. Metashape project frame

* The chunk transform of the Metashape project (`ECEF = s·R·X + t`) maps to
  the geocentric frame of the project CRS's datum. For Dorfberg this is
  CH1903+ (Bessel ellipsoid), not WGS 84. ETRS89 / WGS 84 ECEF follows by
  adding the `TOWGS84` translation of the project CRS, (674.374, 15.056,
  405.346) m. CH1903+ is defined from CHTRS95 by exactly this translation,
  so the step is exact. Without it, all poses are off by about 705 m.
* The injector applies this when it converts Metashape poses to the
  telemetry channel (ADR-004 section 3).
* In the ground control point file `20201109_Dorfberg.txt`, the latitude and
  longitude columns do not match the site (46.49°, 9.50° instead of about
  46.82°, 9.84°). The ECEF and LV95 columns are consistent and are used.

## Rationale

* The Metashape products already exist, match the flight and the ground
  control, and come from widely used photogrammetric software. Producing a
  second offline reconstruction would cost time without improving the
  evaluation.
* Measuring point cloud accuracy against the reference DSM avoids a
  dependency on a proprietary format.
* A single internal camera convention with explicit conversions at the
  boundaries prevents silent errors of half a pixel to several pixels. Such
  errors would bias depth and orthomosaic accuracy.

## Alternatives Considered

* **Offline COLMAP reconstruction, as in the proposal.** Rejected: duplicates
  the existing Metashape reconstruction and adds a second pipeline to set up.
* **Downloading the complete Metashape dense cloud (10.6 GB).** Rejected: its
  format requires Metashape; the reference DSM serves the purpose.
* **COLMAP pixel convention internally.** Rejected: OpenCV is used for image
  processing throughout, so a single conversion at the COLMAP boundary is
  simpler.

## Affected Areas

* [x] simulator interface
* [x] coordinate frames and conventions
* [x] algorithm libraries
* [x] evaluation and experiments
* [x] documentation

## Impact

* **ADR-004:** completed by section 2 (pixel and camera model conventions).
* **ADR-003 (injector):** converts the Metashape calibration as in section 2
  and sends it in the session start message.
* **ADR-005:** the dense reconstruction adapter converts to the COLMAP
  convention.
* **D17:** the evaluation compares with the Metashape DSM and orthophoto and
  with the ground control points. It reports that the Metashape products are
  not an independent ground truth.

## Open Questions

* Has the supervisor agreed to Metashape products in place of an offline
  COLMAP reconstruction?
