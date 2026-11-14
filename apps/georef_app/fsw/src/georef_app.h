/*
 * GEOREF_APP: georeferencing of the depth sets (ADR-007, ADR-012).
 *
 * Tasks (ADR-015 section 2):
 *   main task    commands, housekeeping; depth sets of DEPTH_READY go into
 *                the worker queue
 *   GEOREF_WORK  one keyframe at a time: depth set to a point set in P
 *                (OST_Map_GeorefConvert, libs/map_products), publication on
 *                POINTS_READY, then the point cloud files (LAZ)
 *
 * GEOREF_APP does not drop data (ADR-015 section 3): when the point set
 * quota is full it waits up to QuotaWaitMs for MAPPROD_APP. A keyframe
 * dropped after that, or for any other error, is counted and reported
 * (POINTS_FAILED).
 *
 * Data store references (ADR-013 section 3): DEPTH_READY carries one count
 * of the depth set, the keyframe's working image and its JPEG. GEOREF_APP
 * releases the depth set and the working image after the conversion and
 * hands the JPEG count and the new point set on with POINTS_READY. For the
 * point cloud it holds its own count of the point set until the files are
 * written.
 */
#ifndef GEOREF_APP_H
#define GEOREF_APP_H

#include "cfe.h"
#include "dense_app_eds_typedefs.h"
#include "georef_app_eds_typedefs.h"
#include "georef_app_platform_cfg.h"
#include "ost_common.h"

#include <stdbool.h>

/* Faults of GEOREF_APP, in event identifier order (60 + index). */
#define GEOREF_APP_FAULT_COUNT 5

typedef struct
{
    int64                          EntryMono;
    DENSE_APP_DepthReady_Payload_t Depth;
} GEOREF_APP_Item_t;

typedef struct
{
    /* Written by the main task */
    uint32 DepthSetsReceived;
    uint32 MessagesRejected;

    /* Written under CountMutex (both tasks) */
    uint32 KeyframesFailed;

    /* Written by the worker */
    uint32 PointSetsPublished;
    uint32 QuotaWaits;
    uint32 LazFiles;
    uint32 LazFailures;
    uint32 LastKeyframeId;
    uint32 LastPointCount;
    uint32 LastTileCount;
    uint32 MaxTileCount;
    float  LastWaitMs;
    float  LastAllocateMs;
    float  LastConvertMs;
    float  MaxConvertMs;
    float  LastStageMs;
    float  MaxStageMs;
    float  LastLazMs;
    float  MaxLazMs;
    float  LastLatencyS;
    float  MaxLatencyS;
    volatile bool Ready;
} GEOREF_APP_Counters_t;

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t          Pipe;
    CFE_TBL_Handle_t         TblHandle;
    bool                     ConfigValid;
    GEOREF_APP_ConfigTable_t Config;

    CFE_SB_MsgId_t DepthReadyMid;

    /* Worker queue: a ring with a mutex and a counting semaphore. */
    GEOREF_APP_Item_t Ring[GEOREF_APP_QUEUE_DEPTH];
    osal_id_t         RingMutex;
    osal_id_t         ItemSem;
    uint32            QueuedItems;
    uint32            TakenItems;

    GEOREF_APP_HkTlm_t HkTlm;

    OST_FaultSet_t Faults;
    osal_id_t      FaultMutex;
    osal_id_t      CountMutex;

    GEOREF_APP_Counters_t Counters;
} GEOREF_APP_Data_t;

extern GEOREF_APP_Data_t GEOREF_APP_Data;

void GEOREF_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text);
void GEOREF_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text);
void GEOREF_APP_FaultResolve(OST_FaultId_Enum_t Id);

/* Releases a data store reference and reports a failure (any task). */
void GEOREF_APP_Release(const OST_ObjectRef_t *Ref);

/* Drops a keyframe: releases every count its DEPTH_READY carries, counts
 * it and reports it with the text (any task). */
void GEOREF_APP_Drop(const DENSE_APP_DepthReady_Payload_t *Depth, const char *Text);

uint32 GEOREF_APP_QueueDepth(void);
bool   GEOREF_APP_Dequeue(GEOREF_APP_Item_t *Item);

/* Worker child task (georef_app_worker.c). */
void GEOREF_APP_WorkerTask(void);

/* Command handlers (EDS dispatch, georef_app.c). */
CFE_Status_t GEOREF_APP_NoopCmd(const GEOREF_APP_NoopCmd_t *Msg);
CFE_Status_t GEOREF_APP_ResetCountersCmd(const GEOREF_APP_ResetCountersCmd_t *Msg);
CFE_Status_t GEOREF_APP_ClearLatchedFaultsCmd(const GEOREF_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t GEOREF_APP_SendHkCmd(const GEOREF_APP_SendHkCmd_t *Msg);

#endif /* GEOREF_APP_H */
