/*
 * REFINE_APP configuration of the reference mission (replaces the default
 * of apps/refine_app/fsw/tables): the nominal dataset, Dorfberg Flight 01,
 * has PPK positions, so positions are held and only attitudes are refined
 * (ADR-019 section 1, precise_position = true; the setting of the
 * sliding-window benchmark, section 9).
 */
#include "cfe_tbl_filedef.h"
#include "refine_app_eds_typedefs.h"

REFINE_APP_ConfigTable_t ConfigTable = {
    .PrecisePosition         = 1, /* PPK */
    .EstimateBoresight       = 1,
    .Spare                   = 0,
    .MaxLatencyS             = 120.0f, /* refine_max_latency */
    .MaxWaitS                = 100.0f,
    .WindowS                 = 240.0f,
    .MaxWindowImages         = 16,
    .MaxPoints               = 2000,
    .MaxIterations           = 30,
    .MaxFeatures             = 4000,
    .PositionSigmaM          = 2.0f,
    .PrecisePositionSigmaM   = 0.02f,
    .AttitudeSigmaDeg        = 3.0f,
    .BoresightSigmaDeg       = 5.0f,
    .BoresightSettleBatches  = 4,
    .HeldBoresightSigmaDeg   = 0.005f,
    .RefinedAttitudeSigmaDeg = 0.02f,
    .MinImageObservations    = 50,
    .MaxReprojectionRmsPx    = 2.0f,
    .MaxCorrectionSigmas     = 5.0f,
};

CFE_TBL_FILEDEF(ConfigTable, REFINE_APP.ConfigTable, REFINE_APP configuration, refine_app_tbl.tbl)
