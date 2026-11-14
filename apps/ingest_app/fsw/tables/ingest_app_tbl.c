/*
 * Default configuration table of INGEST_APP (ADR-016 section 5). The
 * reference mission may replace it under mission/orthostream_defs/tables/.
 */
#include "cfe_tbl_filedef.h"
#include "ingest_app_eds_typedefs.h"

INGEST_APP_ConfigTable_t ConfigTable = {
    .UdpPort          = 50100,
    .TcpPort          = 50101,
    .TelemetryStaleMs = 1000,  /* ADR-014 section 4: waiting limit 1 s */
    .ImageStaleMs     = 30000, /* the turns of Dorfberg Flight 01 last up to 25 s */
    .AutoStart        = 1,
    .Spare1           = 0,
    .Spare2           = 0,
};

CFE_TBL_FILEDEF(ConfigTable, INGEST_APP.ConfigTable, INGEST_APP configuration, ingest_app_tbl.tbl)
