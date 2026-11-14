/*
 * REFINE_APP: pose refinement of the keyframe batches (ADR-019).
 *
 * Tasks (ADR-019 section 6):
 *   main task      commands, housekeeping; batches of BATCH_SELECTED and the
 *                  session calibration go into the worker queue
 *   REFINE_WORK    features (libs/refine), the sliding window, publication
 *                  on BATCH_READY
 *
 * A batch waits in the worker until the next flight line covers its
 * keyframe or MaxWaitS expires (section 3, item 4). It is never dropped: a
 * batch whose refinement fails, or that cannot be queued, is published with
 * its telemetry poses and pose source REFINE_FAILED (section 5).
 *
 * Data store references: a batch carries one count of each working image
 * and the keyframe's JPEG count for its consumer. REFINE_APP reads the
 * working images under those counts and hands all of them on with the
 * published batch (ADR-013 section 3).
 */
#ifndef REFINE_APP_H
#define REFINE_APP_H

#include "cfe.h"
#include "ingest_app_eds_typedefs.h"
#include "ost_common.h"
#include "ost_eds_designparameters.h"
#include "refine_app_eds_typedefs.h"
#include "refine_app_platform_cfg.h"

#include <stdbool.h>

/* Faults of REFINE_APP, in event identifier order (60 + index). */
#define REFINE_APP_FAULT_COUNT 4

typedef enum
{
    REFINE_APP_ITEM_BATCH   = 1,
    REFINE_APP_ITEM_SESSION = 2, /* a session calibration: solve and publish everything */
    REFINE_APP_ITEM_FLUSH   = 3  /* flush command */
} REFINE_APP_ItemType_t;

typedef struct
{
    uint32                      Type;
    uint32                      SessionId;
    int64                       EntryMono;
    OST_KeyframeBatch_Payload_t Batch;
} REFINE_APP_Item_t;

typedef struct
{
    /* Written by the main task */
    uint32 BatchesReceived;
    uint32 QueueOverflows;
    uint32 MessagesRejected;

    /* Written by the worker */
    uint32 BatchesPublished;
    uint32 BatchesRefined;
    uint32 BatchesFailed;
    uint32 BatchesCovered;
    uint32 BatchesWaitLimit;
    uint32 BatchesHeld;
    uint32 LatencyExceeded;
    uint32 ImagesAdded;
    uint32 WindowImages;
    uint32 LastPoints;
    uint32 LastIterations;
    uint32 LastWindowImages;
    float  LastWaitedS;
    float  LastLatencyS;
    float  MaxLatencyObservedS;
    float  LastRmsPx;
    float  LastBoresightDeg;
    float  LastCorrectionDeg;
    uint32 LastSolveMs;
    uint32 MaxSolveMs;
    uint32 LastFeaturesMs;
    float  FeaturesTotalS;
    volatile bool BoresightHeld;
    volatile bool Ready;
} REFINE_APP_Counters_t;

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t          Pipe;
    CFE_TBL_Handle_t         TblHandle;
    bool                     ConfigValid;
    REFINE_APP_ConfigTable_t Config;

    CFE_SB_MsgId_t BatchSelectedMid;
    CFE_SB_MsgId_t SessionCalibrationMid;

    /* Worker queue: a ring with a mutex and a counting semaphore. */
    REFINE_APP_Item_t Ring[REFINE_APP_QUEUE_DEPTH];
    osal_id_t         RingMutex;
    osal_id_t         ItemSem;
    uint32            QueuedItems;
    uint32            TakenItems;

    REFINE_APP_HkTlm_t HkTlm;

    OST_FaultSet_t Faults;
    osal_id_t      FaultMutex;
    osal_id_t      PublishMutex; /* both tasks publish batches */

    REFINE_APP_Counters_t Counters;
} REFINE_APP_Data_t;

extern REFINE_APP_Data_t REFINE_APP_Data;

void REFINE_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text);
void REFINE_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text);

/* Releases a data store reference and reports a failure (any task). */
void REFINE_APP_Release(const OST_ObjectRef_t *Ref);

/* Publishes a batch on BATCH_READY with the given pose source (any task;
 * the batch's counts go to the consumer, or are released if the message
 * cannot be sent). Returns true if it was sent. */
bool REFINE_APP_PublishBatch(const OST_KeyframeBatch_Payload_t *Batch, int64 EntryMono);

bool REFINE_APP_Dequeue(REFINE_APP_Item_t *Item, uint32 TimeoutMs);

/* Worker child task (refine_app_worker.c). */
void REFINE_APP_WorkerTask(void);

/* Command handlers (EDS dispatch, refine_app.c). */
CFE_Status_t REFINE_APP_NoopCmd(const REFINE_APP_NoopCmd_t *Msg);
CFE_Status_t REFINE_APP_ResetCountersCmd(const REFINE_APP_ResetCountersCmd_t *Msg);
CFE_Status_t REFINE_APP_ClearLatchedFaultsCmd(const REFINE_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t REFINE_APP_FlushCmd(const REFINE_APP_FlushCmd_t *Msg);
CFE_Status_t REFINE_APP_SendHkCmd(const REFINE_APP_SendHkCmd_t *Msg);

#endif /* REFINE_APP_H */
