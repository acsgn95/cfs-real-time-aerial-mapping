/*
 * MAPPROD_APP: DSM and orthomosaic tiles (ADR-007, ADR-012).
 *
 * Tasks (ADR-015 section 2):
 *   main task     commands, housekeeping, the 1 Hz wake-up; point sets,
 *                 the session calibration and publication requests go into
 *                 the worker queue, in arrival order
 *   MAPPROD_WORK  owns the tiles (libs/map_products is not thread-safe):
 *                 per keyframe DSM fusion of the point set, orthomosaic
 *                 rendering of the JPEG, tile lifecycle and TILE_UPDATED;
 *                 every publication period a copy of the changed tiles
 *   MAPPROD_PUB   writes the copies (publication, ADR-007 section 6) and
 *                 sends TILE_PUBLISHED; writing an orthomosaic tile takes
 *                 most of a second, which would otherwise hold up the map
 *                 update
 *
 * MAPPROD_APP does not drop data (ADR-015 section 3). A point set that
 * finds the queue full, or that fails, is counted and reported
 * (MAP_UPDATE_FAILED).
 *
 * Data store references (ADR-013 section 3): POINTS_READY carries one count
 * of the point set and of the keyframe's JPEG. MAPPROD_APP releases both
 * after the keyframe; it is the last consumer of the data path.
 */
#ifndef MAPPROD_APP_H
#define MAPPROD_APP_H

#include "cfe.h"
#include "georef_app_eds_typedefs.h"
#include "ingest_app_eds_typedefs.h"
#include "mapprod_app_eds_typedefs.h"
#include "mapprod_app_platform_cfg.h"
#include "ost_common.h"

#include "map_products_c_api.h"

#include <stdbool.h>

/* Faults of MAPPROD_APP, in event identifier order (60 + index). */
#define MAPPROD_APP_FAULT_COUNT 5

typedef enum
{
    MAPPROD_APP_ITEM_POINTS       = 1,
    MAPPROD_APP_ITEM_CALIBRATION  = 2,
    MAPPROD_APP_ITEM_PUBLISH      = 3,
    MAPPROD_APP_ITEM_FINALISE_ALL = 4,
} MAPPROD_APP_ItemType_t;

typedef struct
{
    MAPPROD_APP_ItemType_t Type;
    int64                  EntryMono;
    union
    {
        GEOREF_APP_PointsReady_Payload_t         Points;
        INGEST_APP_SessionCalibration_Payload_t Calibration;
    } u;
} MAPPROD_APP_Item_t;

typedef struct
{
    /* Written by the main task */
    uint32 PointSetsReceived;
    uint32 MessagesRejected;

    /* Written under CountMutex (several tasks) */
    uint32 KeyframesFailed;
    uint32 TileWriteFailures;
    uint32 Publications;
    float  LastPublishMs;
    float  MaxPublishMs;

    /* Written by the worker */
    uint32 KeyframesUpdated;
    uint32 OrthoSkipped;
    uint32 ActiveTiles;
    uint32 ChangedTiles;
    uint32 TilesFinalised;
    uint32 LastKeyframeId;
    float  MemoryMb;
    float  LastFuseMs;
    float  LastDecodeMs;
    float  LastRenderMs;
    float  LastLifecycleMs;
    float  LastUpdateMs;
    float  MaxUpdateMs;
    float  LastLatencyS;
    float  MaxLatencyS;
    volatile bool Ready;
    volatile bool CalibrationKnown;
} MAPPROD_APP_Counters_t;

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t           Pipe;
    CFE_TBL_Handle_t          TblHandle;
    bool                      ConfigValid;
    MAPPROD_APP_ConfigTable_t Config;

    CFE_SB_MsgId_t PointsReadyMid;
    CFE_SB_MsgId_t CalibrationMid;

    /* Worker queue: a ring with a mutex and a counting semaphore. */
    MAPPROD_APP_Item_t Ring[MAPPROD_APP_QUEUE_DEPTH];
    osal_id_t          RingMutex;
    osal_id_t          ItemSem;
    uint32             QueuedItems;
    uint32             TakenItems;

    /* Publication period, counted by the main task on the wake-up. */
    uint32        SecondsSincePublish;
    volatile bool PublishQueued; /* cleared by the worker */

    osal_id_t     WorkerDone; /* given by the worker when it ends */
    bool          WorkerStarted;

    /* Publisher: one snapshot at a time, handed over by the worker. */
    osal_id_t           PublishSem;     /* given by the worker with a snapshot, or to stop */
    osal_id_t           PublisherDone;  /* given by the publisher when it ends */
    volatile bool       PublisherBusy;  /* set by the worker, cleared by the publisher */
    volatile bool       PublisherStop;
    OST_Map_Products_t *volatile Products; /* created and destroyed by the worker */
    OST_Map_Snapshot_t *Snapshot;
    uint32              SnapshotTiles;
    uint32              SnapshotKeyframeId;
    int64               SnapshotFirstUtc; /* first update since the previous publication */
    uint32              PublishedSequence; /* under CountMutex */

    MAPPROD_APP_HkTlm_t HkTlm;

    OST_FaultSet_t Faults;
    osal_id_t      FaultMutex;
    osal_id_t      CountMutex;

    MAPPROD_APP_Counters_t Counters;
} MAPPROD_APP_Data_t;

extern MAPPROD_APP_Data_t MAPPROD_APP_Data;

void MAPPROD_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text);
void MAPPROD_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text);
void MAPPROD_APP_FaultResolve(OST_FaultId_Enum_t Id);

/* Releases a data store reference and reports a failure (any task). */
void MAPPROD_APP_Release(const OST_ObjectRef_t *Ref);

/* Drops a keyframe: releases the counts of its POINTS_READY, counts it and
 * reports it with the text (any task). */
void MAPPROD_APP_Drop(const GEOREF_APP_PointsReady_Payload_t *Points, const char *Text);

uint32 MAPPROD_APP_QueueDepth(void);
bool   MAPPROD_APP_Dequeue(MAPPROD_APP_Item_t *Item);

/* Worker and publisher child tasks (mapprod_app_worker.c). */
void MAPPROD_APP_WorkerTask(void);
void MAPPROD_APP_PublisherTask(void);

/* Sends TILE_PUBLISHED and counts the publication (any task). */
void MAPPROD_APP_SendPublished(MAPPROD_APP_TilePublishedTlm_t *Msg, MAPPROD_APP_PublishReason_Enum_t Reason,
                               const OST_Map_WriteResult_t *R, uint32 LastKeyframeId, int64 FirstPendingUtc);

/* Command handlers (EDS dispatch, mapprod_app.c). */
CFE_Status_t MAPPROD_APP_NoopCmd(const MAPPROD_APP_NoopCmd_t *Msg);
CFE_Status_t MAPPROD_APP_ResetCountersCmd(const MAPPROD_APP_ResetCountersCmd_t *Msg);
CFE_Status_t MAPPROD_APP_ClearLatchedFaultsCmd(const MAPPROD_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t MAPPROD_APP_FinaliseAllCmd(const MAPPROD_APP_FinaliseAllCmd_t *Msg);
CFE_Status_t MAPPROD_APP_SendHkCmd(const MAPPROD_APP_SendHkCmd_t *Msg);
CFE_Status_t MAPPROD_APP_WakeupCmd(const MAPPROD_APP_WakeupCmd_t *Msg);

#endif /* MAPPROD_APP_H */
