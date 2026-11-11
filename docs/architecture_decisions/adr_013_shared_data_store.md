# ADR-013: Shared Data Store for Bulk Data

* Status: Accepted
* Date: 2026-11-11
* Decided by: Ahmet Coşgun
* Backlog item: D12
* Evidence: ADR-001 (data sizes, R4), ADR-002 (16 GB RAM budget), ADR-006
  (image pool), ADR-011 (caller-owned buffers), ADR-012 (four applications
  share bulk data); the cFS native build runs all applications as threads of
  one process (`core-cpu1`); a single-producer shared-memory frame ring with
  sequence locks in a cFS mission on the same machine (NavionCore
  `camera_pool`)

## Context

Bulk data cannot travel on the Software Bus. Its messages are limited to
32 KB, while a JPEG image is about 18 MB, a working-resolution image about
6 MB and a set of depth, normal and confidence maps about 40 MB at the
nominal point (ADR-001).

ADR-012 has four applications that produce or read bulk data: `INGEST_APP`
(JPEG), `SELECT_APP` (working-resolution images), `DENSE_APP` (depth sets)
and `GEOREF_APP` (point sets). Readers include `DENSE_APP`, `GEOREF_APP` and
`MAPPROD_APP`.

A fixed-size ring with one producer and lossy overwrite fits a camera stream,
but not this data:

* object sizes vary by type and camera;
* lifetimes are long and not first-in, first-out. An image of the previous
  flight line may be needed as a source one or two minutes later;
* loss is not acceptable. Geometric consistency needs the depth maps of the
  source keyframes;
* there are several producers.

On the target (ADR-002), all cFS applications run as threads of one process.

## Decision

### 1. A `datastore` library loaded as a cFS library

* The data store is a ninth algorithm-side library, `datastore`, with a C
  interface (ADR-011).
* It is loaded as a cFS library module (`CFE_LIB`) before the applications
  start. Applications use it through its C interface.
* The store lives in the process memory of the cFS mission. POSIX shared
  memory is not used.

### 2. Write-once, reference-counted objects

* **Producer:** allocates an object of a type and size, writes its data and
  metadata, seals it, and publishes a Software Bus message that carries a
  reference to it.
* **Reader:** acquires the object (reference count + 1), reads it, and
  releases it (reference count − 1).
* A sealed object never changes, so reading needs no lock. Allocation,
  acquisition and release are short operations under a mutex.
* **Reference:** `{object id, generation}`. The generation detects the use of
  a reference to an object that has already been freed. Such a use is an
  error and is counted.

### 3. Ownership and release

* An object is freed when its reference count reaches zero.
* `SELECT_APP` holds references to the images, and through its batches to
  the depth sets, that may still serve as sources. It releases them when the
  spatial eviction rule of ADR-006 removes them from the image pool.
* Every holder releases what it acquired. Ownership is explicit; nothing is
  overwritten while in use.

### 4. Preallocated memory with quotas per type

* One memory arena is allocated when the store starts. No allocation from
  the system heap happens during operation.
* Default size 8 GB, within the 16 GB RAM budget of ADR-002, with quotas per
  type:

  | Object type | Default quota | Holds |
  | --- | --- | --- |
  | JPEG image | 1 GB | images waiting to be decoded and keyframe images waiting for orthomosaic rendering |
  | Working-resolution image | 2 GB | the image pool, about 300 images |
  | Depth set (depth, normal, confidence) | 4 GB | keyframes of about two or three flight lines |
  | Point set | 1 GB | between `GEOREF_APP` and `MAPPROD_APP` |

* Sizes and quotas are configuration values (D15).
* An allocation that would exceed a quota fails with an explicit status. The
  producer reacts through the backpressure policy of D14. Memory never grows
  beyond the configured size (ADR-001 R4).

### 5. No copies on the data path

* `INGEST_APP` receives JPEG data from the TCP stream directly into the
  allocated object.
* Libraries receive pointers into sealed objects (ADR-011 caller-owned
  buffers).
* The only required copy is the transfer of images to the GPU. Registering
  the arena as pinned memory for faster transfers is a possible later
  optimisation.

### 6. Observability

The store reports, per type, the bytes and objects in use, failed
allocations and stale-reference errors. These values go into housekeeping
telemetry (D16).

### 7. Portability

The interface does not expose the process model. If the applications later
run in separate processes or partitions, only the implementation changes,
for example to POSIX shared memory. The applications stay the same.

### 8. Software Bus messages stay small

Every message stays well below the 32 KB limit. Bulk data travels only as
references.

## Rationale

* Write-once objects with reference counts give the long, non-sequential
  lifetimes the image pool needs, without loss and without locks on reads.
* Generations make use-after-free visible instead of silent.
* A preallocated arena with quotas makes the memory bound of ADR-001 R4
  explicit and enforceable, and avoids heap fragmentation during long runs.
* A cFS library module is the standard way to share a service between
  applications.
* Keeping the store inside the process avoids shared memory setup that the
  current target does not need, while the interface keeps that option open.

## Alternatives Considered

* **Fixed-size ring with sequence locks.** Rejected: lossy overwrite, fixed
  object size, first-in first-out lifetimes and a single producer do not fit
  this data.
* **POSIX shared memory.** Not needed now: all applications run in one
  process. Kept as a later implementation option (section 7).
* **Temporary files (for example on tmpfs).** Rejected: copies, system calls
  and latency on every access.
* **Bulk data on the Software Bus.** Not possible: message size limit.
* **Heap allocation per object.** Rejected: unbounded growth is possible and
  fragmentation over long runs.

## Affected Areas

* [x] large data transport
* [x] algorithm libraries
* [x] cFS application structure
* [x] Software Bus messages
* [x] configuration/tables
* [x] observability and performance measurement
* [x] documentation

## Impact

* **ADR-011:** the library list gains `datastore`.
* **Startup script (ADR-009):** loads the `datastore` library module before
  the applications.
* **D14:** defines backpressure when an allocation fails.
* **D15:** messages carry `{object id, generation}` references. Store size
  and quotas are table values.
* **D16:** store statistics go into housekeeping.

## Open Questions

None.
