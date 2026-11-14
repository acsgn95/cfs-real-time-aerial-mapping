/*
 * Default configuration table of ALIGN_APP (ADR-016 section 5). The limits
 * are those of ADR-014 section 4.
 */
#include "cfe_tbl_filedef.h"
#include "align_app_eds_typedefs.h"

ALIGN_APP_ConfigTable_t ConfigTable = {
    .MaxGapMs          = 300,   /* three periods at 10 Hz */
    .MaxWaitMs         = 1000,
    .BufferMs          = 10000,
    .MaxPendingImages  = 64,
    .MaxPositionStdM   = 5.0f,  /* single-point GNSS and better */
    .MaxAttitudeStdDeg = 5.0f,  /* the injector reports 3 degrees by default */
};

CFE_TBL_FILEDEF(ConfigTable, ALIGN_APP.ConfigTable, ALIGN_APP configuration, align_app_tbl.tbl)
