# vcpkg overlay ports

Ports here replace the port of the same name in `vendor/vcpkg` (ADR-010).
Each one is a copy of the upstream port at the pinned vcpkg release with a
minimal, commented change. When `vendor/vcpkg` is updated, compare each
overlay with the new upstream port and remove it if the change is no longer
needed.

| Port | Upstream version | Change |
| --- | --- | --- |
| `gtsam` | 4.3.0 (vcpkg 2026.10.16) | `GTSAM_BUILD_WITH_WERROR=OFF`: GCC 15 reports a false `maybe-uninitialized` warning in Eigen's SIMD code in GTSAM's timing utilities, which `-Werror` turns into a build failure |
