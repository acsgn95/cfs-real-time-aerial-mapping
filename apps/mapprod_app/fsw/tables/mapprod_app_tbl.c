/*
 * Default configuration table of MAPPROD_APP (ADR-016 section 5): the
 * defaults of ADR-007. P is LV95 with ellipsoidal heights inside the
 * mission; the DSM is written with LHN95 heights (ADR-004 sections 5 and
 * 6). The tile grid and the CRS must equal those of GEOREF_APP. The PROJ
 * files are those the mission installs under /cf/geodata.
 */
#include "cfe_tbl_filedef.h"
#include "mapprod_app_eds_typedefs.h"

MAPPROD_APP_ConfigTable_t ConfigTable = {
    .ProjectedCrs      = "EPSG:2056",
    .VerticalCrs       = "EPSG:5729",
    .ProjDataDir       = "/cf/geodata/proj",
    .GridDir           = "/cf/geodata/grids",
    .OutputDir         = "/cf/products",
    .CellSizeM         = 0.26f,
    .TileCells         = 512,
    .MinObservations   = 3,
    .RejectionSigmas   = 3.0f,
    .MinSigmaM         = 0.25f,
    .MinConfidence     = 0.0f,
    .OrthoPixelM       = 0.066f,
    .SurfaceSmoothingM = 1.0f,
    .SwitchMarginDeg   = 2.0f,
    .RenderMarginM     = 30.0f,
    .OrthoWidth        = 0, /* half the camera width: 1/2 DCT scale at Dorfberg */
    .MaxActiveTiles    = 64,
    .FinaliseDistanceM = 600.0f,
    .PublishPeriodS    = 5,
    .RenderOrtho       = 1,
    .FinaliseOnStop    = 1,
    .Spare             = 0,
};

CFE_TBL_FILEDEF(ConfigTable, MAPPROD_APP.ConfigTable, MAPPROD_APP configuration, mapprod_app_tbl.tbl)
