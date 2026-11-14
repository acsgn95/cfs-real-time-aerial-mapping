/*
 * Default configuration table of DENSE_APP (ADR-016 section 5): the
 * defaults of ADR-005 (SGBM since 2026-11-13, section 5; terrain gate,
 * section 5a). The terrain paths point to the files the mission installs
 * under /cf/geodata (mission/orthostream_defs/cpu1/install_custom.cmake).
 */
#include "cfe_tbl_filedef.h"
#include "dense_app_eds_typedefs.h"

DENSE_APP_ConfigTable_t ConfigTable = {
    .Method                  = DENSE_APP_Method_SGBM,
    .StoreNormals            = 0, /* SGBM normals only point to the camera */
    .MaxWaitingBatches       = 16, /* bursts of REFINE_APP; 1 without pose refinement (ADR-015 section 3) */
    .Spare                   = 0,
    .Threads                 = 0, /* OpenCV's default */
    .QuotaWaitMs             = 2000,
    .SgbmTargetBaselineRatio = 0.2f,
    .SgbmBlockSize           = 5,
    .SgbmP1Factor            = 8,
    .SgbmP2Factor            = 32,
    .SgbmUniquenessRatio     = 10,
    .SgbmDisp12MaxDiff       = 1,
    .SgbmSpeckleWindow       = 100,
    .SgbmSpeckleRange        = 2,
    .SgbmMaxDisparities      = 512,
    .PmGpuIndex              = 0,
    .PmIterations            = 3,
    .PmWindowRadius          = 3,
    .PmWindowStep            = 1,
    .PmSamples               = 7,
    .PmFilterMinConsistent   = 2,
    .PmFilterMinNcc          = 0.3f,
    .PmFilterMinAngleDeg     = 3.0f,
    .PmMinFreeGpuMb          = 1024.0f, /* peak 0.6 GB at 2048 px (section 6a) */
    .DepthMarginM            = 50.0f,
    .TerrainGateM            = 30.0f,
    .GateSpacingM            = 5.0f,
    .GateMarginM             = 100.0f,
    .TerrainRadiusM          = 20000.0f,
    .DemPath                 = "/cf/geodata/dem/terrain.vrt",
    .DemCrs                  = "EPSG:4326+3855", /* Copernicus GLO-30, EGM2008 heights */
    .ProjDataDir             = "/cf/geodata/proj",
    .GridDir                 = "/cf/geodata/grids",
};

CFE_TBL_FILEDEF(ConfigTable, DENSE_APP.ConfigTable, DENSE_APP configuration, dense_app_tbl.tbl)
