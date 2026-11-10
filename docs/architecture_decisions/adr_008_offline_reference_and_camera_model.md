# ADR-008: Offline Reference Products and Camera Model Conventions

* Status: Accepted
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D08
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
* **Verification:** the conversion is tested by projecting the five ground
  control points into the images where Metashape reports their marker
  positions. Reprojection residuals must stay at the level reported by
  Metashape (about 0.5 px).

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
