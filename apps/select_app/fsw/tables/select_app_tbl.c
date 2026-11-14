/*
 * Default configuration table of SELECT_APP (ADR-016 section 5): the
 * defaults of ADR-006. The terrain paths point to the files the mission
 * installs under /cf/geodata (mission/orthostream_defs/cpu1/install_custom.cmake).
 */
#include "cfe_tbl_filedef.h"
#include "select_app_eds_typedefs.h"

SELECT_APP_ConfigTable_t ConfigTable = {
    .RefinePose                     = 1,    /* ADR-019 */
    .WorkChannels                   = 3,    /* RGB: also the orthomosaic source */
    .DelayedSources                 = 1,    /* ADR-005 section 2: delayed mode */
    .Spare                          = 0,
    .WorkWidth                      = 2048, /* ADR-006 revision of 2026-11-11 */
    .FootprintIterations            = 3,
    .MaxSources                     = 5,
    .MaxPoolImages                  = 200,  /* 8.4 MB each: 1.7 GB of the 2 GB working image quota */
    .NominalHeightAglM              = 250.0f,
    .MaxTiltDeg                     = 25.0f,
    .TargetOverlap                  = 0.7f,
    .MinOverlap                     = 0.4f,
    .MinBaselineRatio               = 0.05f,
    .MaxBaselineRatio               = 0.6f,
    .MaxViewAngleDeg                = 30.0f,
    .PreferredTriangulationAngleDeg = 10.0f,
    .LineTurnDeg                    = 45.0f,
    .PoolDistanceM                  = 600.0f,
    .TerrainRadiusM                 = 20000.0f,
    .MaxDelayS                      = 10.0f, /* about three keyframe intervals */
    .DemPath                        = "/cf/geodata/dem/terrain.vrt",
    .DemCrs                         = "EPSG:4326+3855", /* Copernicus GLO-30, EGM2008 heights */
    .ProjDataDir                    = "/cf/geodata/proj",
    .GridDir                        = "/cf/geodata/grids",
};

CFE_TBL_FILEDEF(ConfigTable, SELECT_APP.ConfigTable, SELECT_APP configuration, select_app_tbl.tbl)
