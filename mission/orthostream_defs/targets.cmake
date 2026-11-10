######################################################################
#
# OrthoStream mission targets (ADR-009)
#
# One processor, cpu1: Linux, POSIX OSAL, pc-linux PSP, native build.
# The OrthoStream applications are added here as they are implemented
# (ADR-011). Until then the mission runs the cFS lab applications only.
#
######################################################################

SET(MISSION_NAME "OrthoStream")

# Spacecraft ID: 0x4F ("O"). Only used to fill message headers.
SET(SPACECRAFT_ID 0x4F)

# Installation directory of host-side functional tests
SET(FT_INSTALL_SUBDIR "host/functional-test")

SET(MISSION_CPUNAMES cpu1)

SET(cpu1_PROCESSORID 1)

# cFS lab applications for development: SCH_LAB (scheduler), CI_LAB (UDP
# command ingest) and TO_LAB (UDP telemetry output). They come from
# vendor/cfs/apps, which mission/build.sh adds to CFS_APP_PATH.
SET(cpu1_APPLIST sch_lab ci_lab to_lab)
