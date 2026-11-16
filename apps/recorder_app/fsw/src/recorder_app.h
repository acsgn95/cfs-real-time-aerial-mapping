/*
 * RECORDER_APP: measurement records of a run (ADR-012, ADR-017).
 *
 * Tasks: the main task takes every message of the table's subscription
 * list off the pipe, encodes it as on the ground link (EDS packed, as
 * TO_LAB encodes it) and appends it with its receive time to a buffer in
 * memory; REC_WRITE writes the buffer to the recording of the current run.
 * A write may block for seconds under memory pressure; the pipe is still
 * emptied, and a message that finds the buffer full is counted (Dropped).
 * A run has a directory <OutputDir>/<RunId>/ with
 *
 *   recording.ostrec  header (64 bytes) and records, little-endian framing:
 *                     uint32 length, int64 receive UTC (ns), the encoded
 *                     message (CCSDS, big-endian as on the link)
 *   manifest.json     run identifier, start and stop times, counts,
 *                     subscriptions
 *
 * tools/ground/recorder_export.py turns a run into CSV files per message,
 * the event log and a JSON-lines file for the report tools.
 *
 * Recording stops below MinFreeMb of free space or at MaxRecordingMb
 * (DISK_SPACE_LOW); a write failure stops it as well (RECORDING_FAILED).
 */
#ifndef RECORDER_APP_H
#define RECORDER_APP_H

#include "cfe.h"
#include "ost_common.h"
#include "recorder_app_eds_typedefs.h"
#include "recorder_app_platform_cfg.h"

#include <stdbool.h>

/* Faults of RECORDER_APP, in event identifier order (60 + index). */
#define RECORDER_APP_FAULT_COUNT 3

#define RECORDER_APP_RUN_ID_MAX (sizeof(((RECORDER_APP_HkTlm_Payload_t *)0)->RunId))

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t           Pipes[RECORDER_APP_PIPES];
    CFE_TBL_Handle_t          TblHandle;
    bool                      ConfigValid;
    RECORDER_APP_ConfigTable_t Config;
    uint16                    Subscriptions;

    /* Current run */
    bool      Recording;
    osal_id_t File;
    char      RunId[RECORDER_APP_RUN_ID_MAX + 1];
    char      RunDir[OS_MAX_PATH_LEN];
    int64     StartUtc;
    uint32    Runs;
    uint32    Records;
    uint64    Bytes;
    uint32    EncodeErrors;
    uint32    Dropped;
    float     FreeMb;

    /* Buffer to the writer task: Head and Tail count bytes since the start
     * of the application; Head - Tail bytes wait. */
    osal_id_t       BufferMutex;
    osal_id_t       DataSem;    /* given by the main task after each append */
    osal_id_t       FileMutex;  /* the file between writer and start/stop */
    osal_id_t       WriterDone;
    uint64          Head;
    uint64          Tail;
    uint32          BufferMax;  /* largest fill of the run, bytes */
    volatile bool   WriterStop;
    volatile bool   WriteFailed; /* set by the writer, handled by the main task */
    volatile uint32 WriteErrors;
    volatile int64  MaxWriteNs;

    RECORDER_APP_HkTlm_t HkTlm;
    OST_FaultSet_t       Faults;
} RECORDER_APP_Data_t;

extern RECORDER_APP_Data_t RECORDER_APP_Data;

/* recorder_app_file.c */
bool         RECORDER_APP_ValidRunId(const char *RunId);
CFE_Status_t RECORDER_APP_Start(const char *RunId);
void         RECORDER_APP_Stop(const char *Reason);
void         RECORDER_APP_Record(const CFE_SB_Buffer_t *Buf);
void         RECORDER_APP_CheckSpace(void);
void         RECORDER_APP_CheckWriter(void);
void         RECORDER_APP_WriterTask(void);

/* recorder_app_encode.c: the message as on the ground link. */
CFE_Status_t RECORDER_APP_Encode(const CFE_SB_Buffer_t *Buf, const void **Encoded, size_t *Size);

/* Command handlers (EDS dispatch, recorder_app.c). */
CFE_Status_t RECORDER_APP_NoopCmd(const RECORDER_APP_NoopCmd_t *Msg);
CFE_Status_t RECORDER_APP_ResetCountersCmd(const RECORDER_APP_ResetCountersCmd_t *Msg);
CFE_Status_t RECORDER_APP_ClearLatchedFaultsCmd(const RECORDER_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t RECORDER_APP_StartRecordingCmd(const RECORDER_APP_StartRecordingCmd_t *Msg);
CFE_Status_t RECORDER_APP_StopRecordingCmd(const RECORDER_APP_StopRecordingCmd_t *Msg);
CFE_Status_t RECORDER_APP_SendHkCmd(const RECORDER_APP_SendHkCmd_t *Msg);
CFE_Status_t RECORDER_APP_WakeupCmd(const RECORDER_APP_WakeupCmd_t *Msg);

#endif /* RECORDER_APP_H */
