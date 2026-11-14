/*
 * DENSE_APP: depth maps of the keyframe batches (ADR-005, ADR-012).
 *
 * Tasks (ADR-015 section 2):
 *   main task    commands, housekeeping; batches of BATCH_READY go into the
 *                worker queue
 *   DENSE_WORK   one depth job at a time: working images to grey, depth
 *                set in the data store, OST_Dense_Estimate (libs/dense),
 *                publication on DEPTH_READY
 *
 * The input is the mission's one controlled drop point (ADR-015 section
 * 3): at most MaxWaitingBatches batches wait behind the running job; a
 * newer batch replaces the oldest waiting one ("newest wins"). Every
 * batch ends with one DENSE_STATUS message: done, skipped or failed, with
 * the reason.
 *
 * Data store references (ADR-013 section 3): a batch carries one count of
 * each working image and the keyframe's JPEG count. DENSE_APP releases the
 * counts of the sources after the job and hands the keyframe's counts and
 * the new depth set on with DEPTH_READY. A skipped or failed batch
 * releases all of its counts.
 */
#ifndef DENSE_APP_H
#define DENSE_APP_H

#include "cfe.h"
#include "dense_app_eds_typedefs.h"
#include "dense_app_platform_cfg.h"
#include "ost_common.h"
#include "ost_eds_designparameters.h"

#include <stdbool.h>

/* Faults of DENSE_APP, in event identifier order (60 + index). */
#define DENSE_APP_FAULT_COUNT 7

typedef struct
{
    int64                       EntryMono;
    OST_KeyframeBatch_Payload_t Batch;
} DENSE_APP_Item_t;

typedef struct
{
    /* Written by the main task */
    uint32 BatchesReceived;
    uint32 MessagesRejected;

    /* Written under StatusMutex (both tasks) */
    uint32 BatchesDone;
    uint32 BatchesSkipped;
    uint32 BatchesFailed;

    /* Written by the worker */
    uint32 QuotaWaits;
    uint32 TerrainLoads;
    uint32 LastKeyframeId;
    float  LastValidFraction;
    float  LastGatedFraction;
    float  LastDepthMinM;
    float  LastDepthMaxM;
    float  LastWaitMs;
    float  LastReadMs;
    float  LastSetupMs;
    float  LastEstimateMs;
    float  MaxEstimateMs;
    float  LastGateMs;
    float  LastJobMs;
    float  MaxJobMs;
    float  LastLatencyS;
    float  MaxLatencyS;
    volatile bool Ready;
    volatile bool LastTerrain;
} DENSE_APP_Counters_t;

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t         Pipe;
    CFE_TBL_Handle_t        TblHandle;
    bool                    ConfigValid;
    DENSE_APP_ConfigTable_t Config;

    CFE_SB_MsgId_t BatchReadyMid;

    /* Worker queue: a ring with a mutex and a counting semaphore. A batch
     * replaced by a newer one leaves the ring without a semaphore count. */
    DENSE_APP_Item_t Ring[DENSE_APP_QUEUE_DEPTH];
    osal_id_t        RingMutex;
    osal_id_t        ItemSem;
    uint32           QueuedItems;
    uint32           TakenItems;

    DENSE_APP_HkTlm_t HkTlm;

    OST_FaultSet_t Faults;
    osal_id_t      FaultMutex;
    osal_id_t      StatusMutex; /* both tasks publish DENSE_STATUS */

    DENSE_APP_Counters_t Counters;
} DENSE_APP_Data_t;

extern DENSE_APP_Data_t DENSE_APP_Data;

void DENSE_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text);
void DENSE_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text);
void DENSE_APP_FaultResolve(OST_FaultId_Enum_t Id);

/* Releases a data store reference and reports a failure (any task). */
void DENSE_APP_Release(const OST_ObjectRef_t *Ref);

/* Releases every count a batch carries (any task). */
void DENSE_APP_ReleaseBatch(const OST_KeyframeBatch_Payload_t *Batch);

/* Publishes the outcome of a batch on DENSE_STATUS and counts it (any
 * task). */
void DENSE_APP_PublishStatus(const OST_KeyframeBatch_Payload_t *Batch, int64 EntryMono,
                             DENSE_APP_Outcome_Enum_t Outcome, DENSE_APP_Reason_Enum_t Reason);

/* Number of batches waiting in the ring. */
uint32 DENSE_APP_QueueDepth(void);

bool DENSE_APP_Dequeue(DENSE_APP_Item_t *Item);

/* Worker child task (dense_app_worker.c). */
void DENSE_APP_WorkerTask(void);

/* Command handlers (EDS dispatch, dense_app.c). */
CFE_Status_t DENSE_APP_NoopCmd(const DENSE_APP_NoopCmd_t *Msg);
CFE_Status_t DENSE_APP_ResetCountersCmd(const DENSE_APP_ResetCountersCmd_t *Msg);
CFE_Status_t DENSE_APP_ClearLatchedFaultsCmd(const DENSE_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t DENSE_APP_SendHkCmd(const DENSE_APP_SendHkCmd_t *Msg);

#endif /* DENSE_APP_H */
