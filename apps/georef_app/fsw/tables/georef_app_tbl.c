/*
 * Default configuration table of GEOREF_APP (ADR-016 section 5). P is LV95
 * with ellipsoidal heights inside the mission; the point cloud files carry
 * LHN95 heights (ADR-004 sections 5 and 6). The tile grid is that of
 * ADR-007 section 1 and must equal the one of MAPPROD_APP. The PROJ files
 * are those the mission installs under /cf/geodata
 * (mission/orthostream_defs/cpu1/install_custom.cmake).
 */
#include "cfe_tbl_filedef.h"
#include "georef_app_eds_typedefs.h"

GEOREF_APP_ConfigTable_t ConfigTable = {
    .ProjectedCrs  = "EPSG:2056",
    .VerticalCrs   = "EPSG:5729", /* LHN95, grid ch_swisstopo_chgeo2004_ETRS89_LHN95.tif */
    .ProjDataDir   = "/cf/geodata/proj",
    .GridDir       = "/cf/geodata/grids",
    .OutputDir     = "/cf/products",
    .CellSizeM     = 0.26f,
    .TileCells     = 512,
    .MinConfidence = 0.0f, /* SGBM confidence is 1 for every matched pixel */
    .QuotaWaitMs   = 5000,
    .WriteLaz      = 1,
    .Spare1        = 0,
    .Spare2        = 0,
};

CFE_TBL_FILEDEF(ConfigTable, GEOREF_APP.ConfigTable, GEOREF_APP configuration, georef_app_tbl.tbl)
