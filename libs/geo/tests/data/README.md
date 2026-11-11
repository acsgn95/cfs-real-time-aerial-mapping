# geo test data

`dorfberg_gcp.txt` is a small extract of Dorfberg Flight 01 (2020-11-09) for
the ground control point check of ADR-008 section 2: the adjusted camera
calibration, the chunk transform, the five ground control points, their 25
marker observations, the 25 camera poses that observe them, and the
per-point results of the Metashape processing report.

It is generated from the dataset with
`tools/fixtures/extract_dorfberg_gcp.py <dataset dir> libs/geo/tests/data/dorfberg_gcp.txt`
and must not be edited by hand.

Source: Bühler, Y., Stoffel, A. and Salzmann, C. (2022). *Photogrammetric
Drone Data Dorfberg*. EnviDat. <https://doi.org/10.16904/envidat.376>.
Licensed under the Open Database License (ODbL) with the Database Contents
License (DbCL).
