/*
 * SELECT_APP: footprints, keyframes and source views, the image pool and
 * the keyframe batches (ADR-006, ADR-012, ADR-019).
 *
 * Tasks (ADR-015 section 2):
 *   main task      commands, housekeeping; ImagePose and SessionCalibration
 *                  are copied into the worker queue in arrival order
 *   SELECT_WORK    selection (libs/selection with libs/terrain), decoding
 *                  to the working resolution (libs/imaging), the pool of
 *                  working images in the data store, batch publication
 *
 * The worker owns the selector, the converter and the pool. Counters are
 * written by one task each and read by the main task for housekeeping. The
 * fault set is shared and protected by FaultMutex.
 *
 * Data store references (ADR-013 section 3): the pool holds one count of
 * each working image. A batch carries one more count of each working image
 * it names and the keyframe's JPEG count from ImagePose, for its consumer.
 * JPEGs of images that do not become keyframes are released after decoding.
 */
#ifndef SELECT_APP_H
#define SELECT_APP_H

#include "cfe.h"
#include "align_app_eds_typedefs.h"
#include "ingest_app_eds_typedefs.h"
#include "select_app_eds_typedefs.h"
#include "select_app_platform_cfg.h"
#include "ost_common.h"
#include "ost_eds_designparameters.h"

#include <stdbool.h>

/* Faults of SELECT_APP, in event identifier order (60 + index). */
#define SELECT_APP_FAULT_COUNT 5

typedef enum
{
    SELECT_APP_ITEM_CALIBRATION = 1,
    SELECT_APP_ITEM_IMAGE       = 2
} SELECT_APP_ItemType_t;

/* One entry of the worker queue. */
typedef struct
{
    uint32 Type;
    int64  EntryMono; /* when the message was received */
    union
    {
        INGEST_APP_SessionCalibration_Payload_t Calibration;
        ALIGN_APP_ImagePose_Payload_t           Image;
    } u;
} SELECT_APP_Item_t;

typedef struct
{
    /* Written by the main task */
    uint32 ImagesReceived;
    uint32 ImagesInvalidPose;
    uint32 QueueOverflows;
    uint32 MessagesRejected;

    /* Written by the worker */
    uint32 ImagesNoCalibration;
    uint32 ImagesNoFootprint;
    uint32 ImagesNoTerrain;
    uint32 ImagesDecoded;
    uint32 DecodeErrors;
    uint32 QuotaFailures;
    uint32 Keyframes;
    uint32 BatchesPublished;
    uint32 BatchesByTimeout;
    uint32 SourcesPublished;
    uint32 BatchesWithOtherLine;
    uint32 PoolImages;
    uint32 PoolEvicted;
    uint32 QuotaEvictions;
    uint32 FlightLines;
    uint32 TerrainLoads;
    uint32 LastDecodeMs;
    uint32 MaxDecodeMs;
    uint32 LastSelectMs;
    uint32 MaxSelectMs;
    float  LastOverlap;
    volatile uint32 SessionId;
    volatile bool   Calibrated;
    volatile bool   TerrainLoaded;
    volatile bool   KeyframeWaiting;
} SELECT_APP_Counters_t;

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t          Pipe;
    CFE_TBL_Handle_t         TblHandle;
    bool                     ConfigValid;
    SELECT_APP_ConfigTable_t Config;

    CFE_SB_MsgId_t ImagePoseMid;
    CFE_SB_MsgId_t SessionCalibrationMid;

    /* Worker queue: a ring of items, RingMutex for the indices, ItemSem
     * counts the items (POSIX message queues allow only 10 entries without
     * privileges). */
    SELECT_APP_Item_t Ring[SELECT_APP_QUEUE_DEPTH];
    osal_id_t         RingMutex;
    osal_id_t         ItemSem;
    uint32            QueuedItems; /* put by the main task */
    uint32            TakenItems;  /* taken by the worker */

    SELECT_APP_HkTlm_t HkTlm;

    OST_FaultSet_t Faults;
    osal_id_t      FaultMutex;

    SELECT_APP_Counters_t Counters;
} SELECT_APP_Data_t;

extern SELECT_APP_Data_t SELECT_APP_Data;

/* Fault set access from any task. */
void SELECT_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text);
void SELECT_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text);
void SELECT_APP_FaultResolve(OST_FaultId_Enum_t Id);

/* Releases a data store reference and reports a failure (any task). */
void SELECT_APP_Release(const OST_ObjectRef_t *Ref);

/* Takes the next item of the worker queue; false after TimeoutMs. */
bool SELECT_APP_Dequeue(SELECT_APP_Item_t *Item, uint32 TimeoutMs);

/* Worker child task (select_app_worker.c). */
void SELECT_APP_WorkerTask(void);

/* Command handlers (EDS dispatch, select_app.c). */
CFE_Status_t SELECT_APP_NoopCmd(const SELECT_APP_NoopCmd_t *Msg);
CFE_Status_t SELECT_APP_ResetCountersCmd(const SELECT_APP_ResetCountersCmd_t *Msg);
CFE_Status_t SELECT_APP_ClearLatchedFaultsCmd(const SELECT_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t SELECT_APP_SendHkCmd(const SELECT_APP_SendHkCmd_t *Msg);

#endif /* SELECT_APP_H */
