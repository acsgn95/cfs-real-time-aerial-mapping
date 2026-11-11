# ADR-001: Flight Envelope, Real-Time Definition and Bounded Memory

* Status: Accepted; amended by [ADR-003](adr_003_data_source_and_injector.md) (no 60 min continuous-operation test)
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D01
* Revised: 2026-11-11 (dense working resolution given as image width; 2048 × 1366 is 2.8 MP)
* Evidence: thesis proposal (FB_YL-01, 2026-10-05) sections 4.2 and 4.3;
  Dorfberg Flight 01 dataset (WingtraHub processing report, Agisoft Metashape
  processing report of 2020-11-10, geotags); footprint, GSD and image-rate
  calculations in this record

## Context

The thesis proposal requires the system to be evaluated over different flight
altitudes, speeds, overlap ratios, camera viewing angles, terrain types and
image acquisition rates, and to report map update latency, images processed per
second and map update frequency. It does not define what "real time" means for
this system or which ranges the system must support.

Every later decision depends on these numbers: the hardware (D02), the data
source (D03), the dense reconstruction method (D05), the large data transport
(D12) and the execution model (D14).

Two observations shape the decision:

1. **Image rates in aerial mapping are low.** The interval between images is
   `(1 - p) * F_along / v`, where `p` is forward overlap, `F_along` the
   along-track footprint and `v` the ground speed. Across the envelope the
   input rate ranges from about 0.04 Hz to about 4 Hz.
2. **Not every input image needs dense processing.** At high overlap,
   consecutive images are largely redundant. Frame selection (D06) picks
   keyframes at a target overlap, so the dense processing rate is lower than
   the input rate and is what determines real-time capability.

The survey area is not limited. A survey area of any size can be supported if
the local frame origin is re-anchored as the vehicle moves (D04) and map
products are kept in tiles that are finalised and written to disk once the
vehicle has left them (D07). What must be limited is the memory working set,
not the area.

The nominal point is taken from a real survey flight, the Dorfberg Flight 01
dataset, replayed into the system by a Python injector. Whether recorded-data
replay replaces or complements a rendering simulator is decided in D03. Using
recorded data instead of a simulation environment departs from the proposal
and needs the supervisor's agreement.

## Decision

### 1. Flight scenarios are given as a design envelope

The system is specified over ranges, not over a single scenario. Each run
selects its parameters from the envelope through configuration; no value is
fixed at compile time.

| Parameter | Design envelope | Nominal point (Dorfberg Flight 01) |
| --- | --- | --- |
| Altitude above ground | 50–300 m | about 250 m (247 m mean) |
| Ground speed | 5–20 m/s | about 18 m/s |
| Forward overlap of input images | 60–90 % | about 85 % |
| Side overlap | 30–80 % | to be measured in D03 |
| Camera tilt from nadir | 0–20° | nadir (attitude within a few degrees) |
| Input image resolution | 1–100 MP | 7952 × 5304 (42.2 MP) |
| Input image rate | derived; up to 4 Hz | about 0.69 Hz (one image per 1.45 s) |
| Input pixel rate (resolution × image rate) | up to 100 MP/s (about 300 MB/s raw RGB) | about 29 MP/s (about 87 MB/s raw RGB) |
| Survey area | unlimited | 1.99 km² |
| Flight duration | unlimited; tested for at least 60 min | 24 min |

The resolution and image-rate ranges are not independent. Combinations above
the pixel-rate limit are outside the envelope. For example, 100 MP at 1 Hz is
inside the envelope, but 100 MP at 4 Hz is not.

### 1a. Input pixel-rate limit

The input pixel rate is limited to 100 MP/s.

* It covers survey cameras at their usual capture rates. For example, a 45 MP
  camera at a 0.7 s interval delivers about 64 MP/s, and a 100 MP camera at
  1 Hz delivers 100 MP/s.
* It leaves a margin of about 3.4 × over the nominal point (29 MP/s).
* Ingest at this rate is not expected to be a bottleneck. Decoding JPEG runs
  on the order of 100 MP/s per CPU core with an optimised decoder, and raw
  memory traffic is about 300 MB/s per pass, provided images are not copied
  between applications (D12). D02 confirms these estimates by measurement.
* A higher limit, such as 100 MP at 4 Hz (400 MP/s), matches no realistic
  survey camera and would only make the data path harder to design.

### 1b. Nominal point: Dorfberg Flight 01

| Property | Value |
| --- | --- |
| Platform | WingtraOne VTOL fixed-wing |
| Camera | Sony DSC-RX1RM2, 35 mm lens, 4.53 µm pixels |
| Calibration (Metashape, adjusted) | f = 7466.67 px, cx = -15.32 px, cy = -13.67 px, K1–K3, P1–P2 |
| Field of view | 56.1° × 39.1° |
| Images | 753, flown 2020-11-09 10:27 UTC |
| Image GSD | 3.31 cm (Metashape mean) |
| Terrain | alpine slope, camera ellipsoidal height 1856–2585 m; meadow, forest, avalanche barriers, roads, buildings |
| Positions | PPK, 100 % fixed, about 2 cm horizontal and 3 cm vertical |
| Timestamps | per image, millisecond resolution |
| Reference data | 10 cm DSM of the same day (Metashape), 5 GNSS ground control points, Metashape project with adjusted camera poses |

The attitude recorded by the aircraft differs from the bundle-adjusted
attitude by about 1.5° on average (Metashape report). At 247 m this is about
6 m on the ground and about 195 px in the image. That is too large for
multi-view stereo with known poses. The poses fed to the system as "known"
are therefore the adjusted poses. The recorded aircraft attitude is a
realistic erroneous pose set for the pose-error experiments. D03 decides the
details.

### 1c. Envelope corners

The envelope corners are sized with a generic 4:3 camera (70° × 55° field of
view, long side across track). The nominal row uses the Dorfberg camera.

| Case | Image GSD | Input interval | Keyframe interval at 70 % overlap |
| --- | --- | --- | --- |
| 50 m, 20 m/s, 90 %, generic 2.8 MP | 3.6 cm | 0.26 s (3.8 Hz) | 0.78 s (1.3 Hz) |
| **Nominal: Dorfberg, 247 m, 18 m/s, 85 %** | **3.3 cm** | **1.45 s (0.69 Hz)** | **2.9 s (0.35 Hz)** |
| 300 m, 20 m/s, 60 %, generic 2.8 MP | 21.9 cm | 6.3 s (0.16 Hz) | 4.7 s (0.21 Hz) |
| 100 m, nominal geometry, generic 100 MP | 1.2 cm | — | — |

The keyframe overlap of 70 % is an assumption for sizing; D06 decides it.

### 1d. Input resolution and processing resolution are separate

* The **input resolution** is the resolution of the images delivered to the
  system (1–100 MP; 42.2 MP at the nominal point).
* The **dense working resolution** is the resolution at which depth maps are
  computed. It is configurable and independent of the input resolution. Its
  default is decided in D05. It is given as an image width; the height
  follows from the aspect ratio. About 2048 px is expected on the target GPU:
  2048 × 1366 (2.8 MP) at the nominal point, which gives about 13 cm GSD.
* The orthomosaic may sample the input images at full resolution. The DSM is
  built from depth maps at the dense working resolution.
* Note for D05: JPEG input can be decoded directly at 1/2, 1/4 or 1/8 scale
  (DCT scaling), which makes the reduction to the working resolution cheap.

Rationale: a raw RGB image is 127 MB at the nominal point and 300 MB at
100 MP. Full-resolution multi-view stereo with several source views does not
fit in the memory of the development GPU (6 GB), and its runtime would be far
above the keyframe interval.

### 2. Definition of real time

Soft real time on a general-purpose operating system. No hard deadlines are
claimed. A run is real time when all of the following hold:

* **R1 Sustained throughput.** Over every 60 s window, the dense processing
  backlog does not grow. The mean keyframe processing time is not greater than
  the keyframe interval.
* **R2 Bounded latency.** Map update latency is measured from the image
  timestamp to the moment every map tile that the image affects has been
  updated. Target: 95th percentile at most 5 s at the nominal point.
* **R3 No uncontrolled loss.** Input images inside the envelope are always
  ingested. When the dense stage is overloaded, keyframes are skipped only
  through the policy defined in D14. Every skipped keyframe is counted and
  reported.

The **real-time envelope** is the part of the design envelope where R1–R3 hold
on the target hardware. It is not fixed in advance; measuring it is a result
of the thesis evaluation (D17). **Acceptance criterion:** the nominal point
must be real time on the target hardware (D02).

### 3. Bounded memory and unlimited area

* **R4** RAM and GPU memory use are bounded by configured budgets (for
  example, the number of active map tiles and queued keyframes). They do not
  grow with the area covered or the flight duration.
* **R5** Disk use may grow linearly with the area covered. Finalised tiles are
  written to disk and released from memory.
* **R6** No component may assume a maximum survey area or a single local
  frame origin for the whole flight. Frame re-anchoring is defined in D04.

For D04: a flat local frame drops below the ellipsoid by about `d² / 2R`, which
is 8 cm at 1 km, 31 cm at 2 km, 71 cm at 3 km and 2 m at 5 km from the origin.

### 4. Map product resolution

Product resolutions are configurable and expressed relative to the GSD of the
data each product is built from:

| Product | Relative to | Default | Allowed range | At the nominal point |
| --- | --- | --- | --- | --- |
| Orthomosaic | input image GSD | 2 ×, never finer than 3 cm | 1–4 ×, never finer than 3 cm | about 6.6 cm |
| DSM | dense working resolution GSD | 2 × | 1–8 × | about 26 cm with a 2048 × 1366 working resolution |

The 3 cm floor limits tile count and disk use for high-resolution input at low
altitude. Without it, a 100 MP camera at 50 m would give a 1.2 cm
orthomosaic.

## Rationale

* The proposal varies the flight parameters. An envelope with a measured
  real-time region makes this variation a result instead of an uncontrolled
  condition.
* A real survey flight as the nominal point gives real imagery, a real
  camera calibration, real timing and independent ground control points.
* Defining real time on keyframes instead of input images avoids spending
  dense reconstruction time on redundant images. At the nominal point this
  leaves about 2.9 s per keyframe, which is consistent with the 5 s latency
  target.
* A memory bound instead of an area bound keeps the system usable for long
  flights and matches tiled, incremental map generation.
* Product resolutions tied to the source GSD stay meaningful across the
  altitude and resolution ranges.

## Alternatives Considered

* **A single fixed scenario.** Rejected: it cannot show how altitude, speed
  and overlap affect accuracy and latency, which the proposal requires.
* **A generic simulated camera (1920 × 1440) as the nominal point.** Rejected
  in favour of the Dorfberg flight, which has real imagery, calibration,
  timing and reference data.
* **A maximum survey area.** Rejected: re-anchoring the frame origin and
  tiling remove the need for it, and an area limit would hide memory growth.
* **Real time defined per input image.** Rejected: at 85–90 % overlap most
  input images add little new geometry, and the requirement would be set by
  redundant data.
* **Hard real time on an RTOS.** Rejected: GPU-based dense reconstruction runs
  on a general-purpose OS. Hard real-time guarantees are outside the thesis
  scope.
* **Fixed product resolutions in metres.** Rejected: one value cannot fit
  both 50 m and 300 m altitude, or 2 MP and 100 MP cameras.
* **No lower bound on orthomosaic resolution.** Rejected: very fine
  resolutions multiply tile count and disk use without benefit for the
  evaluation.
* **A pixel-rate limit of 400 MP/s (100 MP at 4 Hz).** Rejected: see 1a.

## Affected Areas

* [x] requirements and performance targets
* [x] simulator interface
* [x] coordinate frames and conventions
* [x] algorithm libraries
* [x] large data transport
* [x] time and execution model
* [x] observability and performance measurement
* [x] evaluation and experiments
* [x] documentation

## Impact

* **D02:** The target hardware must make the nominal point real time. RAM,
  disk bandwidth and disk capacity must be sized for 100 MP input as well.
* **D03:** The data source must deliver the nominal dataset in real time
  (images, adjusted poses, timestamps, calibration). Any source covering the
  rest of the envelope must support a configurable camera (resolution up to
  100 MP, field of view, tilt), image rates up to 4 Hz, and flights without
  area limits. D03 also decides how altitude and tilt are varied, since a
  single recorded flight cannot vary them, and how the 60 min
  continuous-operation test is fed, since the nominal flight lasts 24 min.
* **D04:** Re-anchoring of the local frame origin is required. The nominal
  dataset uses Swiss LV95 coordinates and WGS 84.
* **D05:** Dense reconstruction runs at a configurable working resolution, so
  input images are downsampled before matching.
* **D06:** Keyframes are selected by overlap, not by a fixed image interval.
* **D07:** Map products are tiled, and tiles are finalised and evicted.
* **D12:** A raw RGB image is 127 MB at the nominal point and 300 MB at
  100 MP. The input data rate reaches about 300 MB/s at the pixel-rate limit.
  At this size, copying images between applications is not acceptable.
* **Data recording and replay:** The nominal flight is 14 GB as JPEG and
  about 95 GB as raw RGB. A 60 min flight at 100 MP is about 520 GB as raw RGB.
  The Data Recorder must not store full-resolution raw images by default.
* **D14:** A keyframe skipping policy is required.
* **D16:** Instrumentation must measure R1–R3 directly: backlog length, latency
  per image and tile, and skip counts.

## Open Questions

None. The question of how the 60 min continuous-operation test is fed was
moved to D03.
