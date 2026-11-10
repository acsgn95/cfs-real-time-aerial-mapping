# ADR-002: Target Hardware and Runtime Environment

* Status: Accepted
* Date: 2026-11-10
* Decided by: Ahmet Coşgun
* Backlog item: D02
* Evidence: ADR-001; inventory of the development machine (`lscpu`,
  `nvidia-smi`, `lsblk`, toolchain versions) taken on 2026-11-10

## Context

ADR-001 requires the nominal point (Dorfberg Flight 01) to be real time on the
target hardware and bounds RAM and GPU memory by configured budgets. It does
not name the hardware.

The data source is a Python injector that replays recorded flights. It
emulates a UAV that sends its images and pose data to the ground over a radio
link such as Wi-Fi or a cellular network. It does not render images and does
not need a GPU.

## Decision

### 1. The development laptop is the target

| Component | Value |
| --- | --- |
| CPU | Intel Core i5-12600HX, 12 cores (4 performance, 8 efficiency), 16 threads, up to 4.6 GHz |
| GPU | NVIDIA GeForce RTX 3050 Laptop, 6 GB, compute capability 8.6 (Ampere) |
| RAM | 37 GB |
| Storage | 2 TB NVMe SSD |
| Display | driven by the integrated GPU; the NVIDIA GPU is free for computation |

An onboard or embedded target is outside the thesis scope and is left for
future work.

### 2. Everything runs on one machine

The injector, the cFS mission and all processing run on the same machine.
The injector sends data to OrthoStream over the local network interface, in
the same way a UAV would send it over its radio link. OrthoStream therefore
acts as ground-side processing of downlinked data.

D03 defines the emulated link: protocol, bandwidth, latency and loss. For
sizing: the nominal flight sends about 12.6 MB/s of JPEG images (about
100 Mbit/s).

### 3. Resource budgets

| Resource | OrthoStream budget | Reason |
| --- | --- | --- |
| GPU memory | at most 5 GB | about 1 GB is kept for the driver and other processes |
| RAM | at most 16 GB | the rest is kept for the injector, the desktop and development tools |
| CPU | not partitioned here | thread and core assignment is decided in D14 |
| Disk | no fixed limit | recordings follow the rules of ADR-001 |

The injector uses the CPU only.

### 4. Measurement conditions

Latency, throughput and resource measurements for the thesis are taken
under fixed conditions:

* the laptop runs on AC power;
* the power profile is set to `performance` (the default is `balanced`);
* no other heavy load runs during a measurement;
* every measured run starts with a warm-up run that is not counted;
* CPU and GPU clock rates and temperatures are recorded with each result
  (D16).

### 5. Software baseline

| Component | Version |
| --- | --- |
| Operating system | Ubuntu 26.04 LTS, Linux 7.0 |
| CUDA | 13.1 |
| Compiler | GCC 15.2 |
| Build system | CMake 4.2 |
| GDAL | 3.12 |
| Python (injector and tools) | 3.14 |

The versions in use are recorded with every experiment. Changing the baseline
during the thesis is a decision of its own.

## Rationale

* The proposal is about simulated data and does not require an onboard
  computer. Using the available machine avoids hardware cost and lets the
  evaluation start early.
* One machine keeps clocks shared and avoids a physical network in the
  latency measurements. The link is still a real network path, so its
  behaviour can be controlled in D03.
* Measurement conditions matter on a laptop: power profile and thermal
  throttling change processing times by large factors.

## Alternatives Considered

* **An embedded target (for example, Jetson Orin).** Deferred to future work:
  not required by the proposal, and it would add a second platform to the
  thesis.
* **Injector on a second machine.** Rejected: it adds clock synchronisation
  between machines and physical network effects that the thesis does not
  study. A link with controlled properties is emulated instead (D03).

## Affected Areas

* [x] requirements and performance targets
* [x] simulator interface
* [x] time and execution model
* [x] observability and performance measurement
* [x] evaluation and experiments
* [x] vendor/cFS baseline
* [x] documentation

## Impact

* **D03:** The injector is a local network sender and has to emulate the
  radio link. About 100 Mbit/s is needed at the nominal point.
* **D05:** The dense reconstruction method must fit in 5 GB of GPU memory at
  the nominal point.
* **D09:** Toolchain risks are checked first:
  * GCC 15 defaults to C23, where `bool`, `true` and `false` are keywords;
    cFS and OSAL may need `-std=gnu17`.
  * CMake 4 rejects projects with `cmake_minimum_required` below 3.5.
  * COLMAP has to be built with CUDA 13.1.
  * OpenCV development files are not installed yet.
* **D16:** Clock rates and temperatures are recorded with each measurement.

## Open Questions

None.
