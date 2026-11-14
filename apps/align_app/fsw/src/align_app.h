/*
 * ALIGN_APP: time alignment of the images with the telemetry stream
 * (ADR-012, ADR-014 section 4).
 *
 * One task: commands, housekeeping and the input messages arrive on one
 * pipe. After every message, and at least every ALIGN_APP_PIPE_TIMEOUT_MS,
 * the images whose outcome is known are published as ImagePose (in arrival
 * order), so that the waiting limit is enforced without a wake-up message.
 *
 * Inputs (ADR-012 section 1a): TelemetrySample, ImageArrived and
 * SessionCalibration of the source adapter. A message of another session
 * starts a new session: the telemetry buffer and the mounting are dropped,
 * waiting images are published as CLEARED.
 */
#ifndef ALIGN_APP_H
#define ALIGN_APP_H

#include "cfe.h"
#include "align_app_eds_typedefs.h"
#include "align_app_platform_cfg.h"
#include "ingest_app_eds_typedefs.h"
#include "ost_common.h"
#include "time_align_c_api.h"

#include <stdbool.h>

/* Faults of ALIGN_APP, in event identifier order (60 + index). */
#define ALIGN_APP_FAULT_COUNT 4

/* An image between ImageArrived and ImagePose. */
typedef struct
{
    bool            InUse;
    uint64          Id; /* session << 32 | image sequence number */
    uint32          SessionId;
    uint32          ImageSeq;
    int64           CaptureUtc;
    int64           RecordedUtc;
    int64           EntryMono;
    OST_ObjectRef_t Jpeg;
    uint32          JpegSize;
} ALIGN_APP_Image_t;

typedef struct
{
    uint32 ImagesReceived;
    uint32 PosesValid;
    uint32 Rejected[OST_ALIGN_RESULT_CLEARED + 1]; /* by result; [0] unused */
    uint32 AccuracyLow;
    uint32 MessagesRejected;
    uint32 LastWaitedMs;
    uint32 MaxWaitedMs;
} ALIGN_APP_Counters_t;

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t         Pipe;
    CFE_TBL_Handle_t        TblHandle;
    bool                    ConfigValid;
    ALIGN_APP_ConfigTable_t Config;

    CFE_SB_MsgId_t TelemetrySampleMid;
    CFE_SB_MsgId_t ImageArrivedMid;
    CFE_SB_MsgId_t SessionCalibrationMid;

    OST_Align_t *Align; /* NULL without a valid table */
    uint32       SessionId;
    bool         Calibrated;

    ALIGN_APP_Image_t Images[ALIGN_APP_MAX_IMAGES];

    ALIGN_APP_HkTlm_t        HkTlm;
    ALIGN_APP_ImagePoseTlm_t PoseTlm;
    ALIGN_APP_Counters_t     Counters;

    OST_FaultSet_t Faults;
} ALIGN_APP_Data_t;

extern ALIGN_APP_Data_t ALIGN_APP_Data;

/* Command handlers (EDS dispatch). */
CFE_Status_t ALIGN_APP_NoopCmd(const ALIGN_APP_NoopCmd_t *Msg);
CFE_Status_t ALIGN_APP_ResetCountersCmd(const ALIGN_APP_ResetCountersCmd_t *Msg);
CFE_Status_t ALIGN_APP_ClearLatchedFaultsCmd(const ALIGN_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t ALIGN_APP_SendHkCmd(const ALIGN_APP_SendHkCmd_t *Msg);

#endif /* ALIGN_APP_H */
