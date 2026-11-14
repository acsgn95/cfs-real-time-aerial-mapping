/*
 * INGEST_APP: source adapter for the injector link (ADR-012 section 1a).
 *
 * Tasks (ADR-015 section 2):
 *   main task  commands, housekeeping, 1 Hz wake-up: staleness faults
 *   UDP task   telemetry datagrams -> TelemetrySample
 *   TCP task   image frames -> data store -> ImageArrived; SessionCalibration
 *
 * Each counter is written by one task only and read by the main task for
 * housekeeping. The fault set is shared and protected by FaultMutex.
 */
#ifndef INGEST_APP_H
#define INGEST_APP_H

#include "cfe.h"
#include "ingest_app_eds_typedefs.h"
#include "ingest_app_platform_cfg.h"
#include "ost_common.h"

#include <stdbool.h>

/* Faults of INGEST_APP, in event identifier order (60 + index). */
#define INGEST_APP_FAULT_COUNT 5

typedef struct
{
    /* Written by the UDP task */
    uint32 TelemetryReceived;
    uint32 TelemetryRejected;
    uint32 TelemetryLost;
    uint32 HeartbeatsReceived;
    uint32 InjectorSamplesSent;
    uint32 InjectorImagesSent;
    int64  LastTelemetryMono; /* 0: none */

    /* Written by the TCP task */
    uint32 ImagesReceived;
    uint32 ImagesDropped;
    uint32 FramesRejected;
    uint32 Connections;
    uint64 ImageBytes;
    int64  LastImageMono;   /* 0: none */
    int64  SessionStartMono; /* when the session became ACTIVE */
} INGEST_APP_Counters_t;

typedef struct
{
    uint16 CommandCounter;
    uint16 CommandErrorCounter;
    uint32 RunStatus;

    CFE_SB_PipeId_t  CommandPipe;
    CFE_TBL_Handle_t TblHandle;
    bool             ConfigValid;
    INGEST_APP_ConfigTable_t Config;

    INGEST_APP_HkTlm_t HkTlm;

    OST_FaultSet_t Faults;
    osal_id_t      FaultMutex;

    /* Sockets wanted open (start/stop session); read by the receivers. */
    volatile bool SocketsWanted;
    /* Session state, written by the TCP task only. */
    volatile INGEST_APP_SessionState_Enum_t State;
    volatile uint32                         SessionId;

    INGEST_APP_Counters_t Counters;
} INGEST_APP_Data_t;

extern INGEST_APP_Data_t INGEST_APP_Data;

/* Fault set access from any task. */
void INGEST_APP_FaultReport(OST_FaultId_Enum_t Id, const char *Text);
void INGEST_APP_FaultRaise(OST_FaultId_Enum_t Id, const char *Text);
void INGEST_APP_FaultResolve(OST_FaultId_Enum_t Id);

/* Child task entry points (ingest_app_net.c). */
void INGEST_APP_UdpTask(void);
void INGEST_APP_TcpTask(void);

/* Command handlers (EDS dispatch, ingest_app.c). */
CFE_Status_t INGEST_APP_NoopCmd(const INGEST_APP_NoopCmd_t *Msg);
CFE_Status_t INGEST_APP_ResetCountersCmd(const INGEST_APP_ResetCountersCmd_t *Msg);
CFE_Status_t INGEST_APP_ClearLatchedFaultsCmd(const INGEST_APP_ClearLatchedFaultsCmd_t *Msg);
CFE_Status_t INGEST_APP_StartSessionCmd(const INGEST_APP_StartSessionCmd_t *Msg);
CFE_Status_t INGEST_APP_StopSessionCmd(const INGEST_APP_StopSessionCmd_t *Msg);
CFE_Status_t INGEST_APP_SendHkCmd(const INGEST_APP_SendHkCmd_t *Msg);
CFE_Status_t INGEST_APP_WakeupCmd(const INGEST_APP_WakeupCmd_t *Msg);

#endif /* INGEST_APP_H */
