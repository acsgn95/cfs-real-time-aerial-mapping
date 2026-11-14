/*
 * INGEST_APP receivers: UDP telemetry and TCP image frames (ADR-003,
 * ADR-016 section 7). Decoding and validation are in libs/link.
 *
 * Images are read from the socket directly into the data store (ADR-013):
 * no copy. Each ImageArrived message carries the one reference count of its
 * JPEG object, owned by the consumer (ALIGN_APP). INGEST_APP never drops an
 * image it could store (ADR-015 section 3); an image that does not fit is
 * read and discarded, counted and reported as DATASTORE_QUOTA.
 */
#include "ingest_app.h"
#include "ingest_app_eventids.h"

#include "cfe_core_api_base_msgids.h"
#include "cfe_mission_eds_designparameters.h"
#include "link_c_api.h"
#include "ost_datastore.h"

#include <string.h>

#define INGEST_APP_SCHEMA_VERSION 1
#define INGEST_APP_DATAGRAM_MAX   2048
#define INGEST_APP_DRAIN_SIZE     65536

typedef enum
{
    INGEST_APP_READ_OK,
    INGEST_APP_READ_CLOSED, /* peer closed the connection */
    INGEST_APP_READ_ERROR,  /* socket error, or stop requested */
} INGEST_APP_ReadResult_t;

static bool INGEST_APP_Running(void)
{
    return INGEST_APP_Data.RunStatus == CFE_ES_RunStatus_APP_RUN;
}

static CFE_SB_MsgId_t INGEST_APP_TlmMid(uint16 TopicId)
{
    return CFE_SB_ValueToMsgId(CFE_PLATFORM_TLM_TOPICID_TO_MIDV(TopicId));
}

static void INGEST_APP_CloseSocket(osal_id_t *Sock)
{
    if (OS_ObjectIdDefined(*Sock))
    {
        OS_close(*Sock);
        *Sock = OS_OBJECT_ID_UNDEFINED;
    }
}

static bool INGEST_APP_OpenSocket(osal_id_t *Sock, OS_SocketType_t Type, uint16 Port, const char *Name)
{
    OS_SockAddr_t Addr;
    int32         Status;

    Status = OS_SocketOpen(Sock, OS_SocketDomain_INET, Type);
    if (Status == OS_SUCCESS)
    {
        OS_SocketAddrInit(&Addr, OS_SocketDomain_INET);
        OS_SocketAddrSetPort(&Addr, Port);
        Status = OS_SocketBind(*Sock, &Addr); /* stream sockets also listen */
    }
    if (Status != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(INGEST_APP_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR, "%s port %u: socket not opened (%ld)",
                          Name, (unsigned)Port, (long)Status);
        INGEST_APP_CloseSocket(Sock);
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------- UDP */

static void INGEST_APP_PublishSample(INGEST_APP_TelemetrySampleTlm_t *Msg, const OST_Link_Datagram_t *D, int64 EntryMono)
{
    INGEST_APP_TelemetrySample_Payload_t *P = &Msg->Payload;
    const OST_Link_Telemetry_t           *T = &D->u.telemetry;

    P->Header.SchemaVersion = INGEST_APP_SCHEMA_VERSION;
    P->Header.Flags         = 0;
    ++P->Header.Sequence;
    P->MeasurementUtc = T->time_ns;
    P->RecordedUtc    = T->recorded_ns;
    P->SessionId      = T->session_id;
    P->LinkSequence   = D->header.sequence;
    P->Latitude       = T->latitude_deg;
    P->Longitude      = T->longitude_deg;
    P->Height         = T->height_m;
    P->AttitudeNedFromBody.W         = T->q_n_b[0];
    P->AttitudeNedFromBody.X         = T->q_n_b[1];
    P->AttitudeNedFromBody.Y         = T->q_n_b[2];
    P->AttitudeNedFromBody.Z         = T->q_n_b[3];
    P->Accuracy.PositionStdNorth     = T->std_position_m[0];
    P->Accuracy.PositionStdEast      = T->std_position_m[1];
    P->Accuracy.PositionStdDown      = T->std_position_m[2];
    P->Accuracy.AttitudeStdRoll      = T->std_attitude_deg[0];
    P->Accuracy.AttitudeStdPitch     = T->std_attitude_deg[1];
    P->Accuracy.AttitudeStdYaw       = T->std_attitude_deg[2];
    P->Accuracy.Fix                  = T->fix;
    P->Accuracy.Spare1               = 0;
    P->Accuracy.Spare2               = 0;
    P->Trace.EntryMono  = EntryMono;
    P->Trace.ExitMono   = OST_TimeMonoNs();
    P->Trace.PublishUtc = OST_TimeUtcNs();

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Msg->TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(Msg->TelemetryHeader), true);
}

void INGEST_APP_UdpTask(void)
{
    static uint8                    Buf[INGEST_APP_DATAGRAM_MAX];
    static INGEST_APP_TelemetrySampleTlm_t Msg;
    INGEST_APP_Counters_t          *C    = &INGEST_APP_Data.Counters;
    osal_id_t                       Sock = OS_OBJECT_ID_UNDEFINED;
    OST_Link_Datagram_t             D;
    const char                     *Reason = NULL;
    uint32                          LastSeq = 0;
    bool                            HaveSeq = false;
    int32                           N;
    int64                           EntryMono;

    memset(&Msg, 0, sizeof(Msg));
    CFE_MSG_Init(CFE_MSG_PTR(Msg.TelemetryHeader),
                 INGEST_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_TELEMETRY_SAMPLE_TLM_TOPICID), sizeof(Msg));

    while (INGEST_APP_Running())
    {
        if (!INGEST_APP_Data.SocketsWanted)
        {
            INGEST_APP_CloseSocket(&Sock);
            OS_TaskDelay(INGEST_APP_SOCKET_TIMEOUT_MS);
            continue;
        }
        if (!OS_ObjectIdDefined(Sock))
        {
            if (!INGEST_APP_OpenSocket(&Sock, OS_SocketType_DATAGRAM, INGEST_APP_Data.Config.UdpPort, "UDP"))
            {
                OS_TaskDelay(1000);
                continue;
            }
            HaveSeq = false;
        }

        N = OS_SocketRecvFrom(Sock, Buf, sizeof(Buf), NULL, INGEST_APP_SOCKET_TIMEOUT_MS);
        if (N == OS_ERROR_TIMEOUT)
        {
            continue;
        }
        if (N < 0)
        {
            CFE_EVS_SendEvent(INGEST_APP_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR, "UDP receive failed (%ld)",
                              (long)N);
            INGEST_APP_CloseSocket(&Sock);
            OS_TaskDelay(1000);
            continue;
        }
        EntryMono = OST_TimeMonoNs();

        if (OST_Link_DecodeDatagram(Buf, (size_t)N, &D, &Reason) != OST_SUCCESS)
        {
            ++C->TelemetryRejected;
            CFE_EVS_SendEvent(INGEST_APP_TLM_REJECTED_EID, CFE_EVS_EventType_ERROR, "datagram of %ld bytes rejected: %s",
                              (long)N, Reason);
            INGEST_APP_FaultReport(OST_FaultId_LINK_FORMAT, Reason);
            continue;
        }

        /* Sequence numbers count all datagrams of the channel; a smaller
         * number means the injector started again. */
        if (HaveSeq && D.header.sequence > LastSeq)
        {
            C->TelemetryLost += D.header.sequence - LastSeq - 1;
        }
        LastSeq = D.header.sequence;
        HaveSeq = true;

        if (D.header.type == OST_LINK_TYPE_HEARTBEAT)
        {
            ++C->HeartbeatsReceived;
            C->InjectorSamplesSent = D.u.heartbeat.samples_sent;
            C->InjectorImagesSent  = D.u.heartbeat.images_sent;
            continue;
        }
        INGEST_APP_PublishSample(&Msg, &D, EntryMono);
        ++C->TelemetryReceived;
        C->LastTelemetryMono = OST_TimeMonoNs();
    }
    INGEST_APP_CloseSocket(&Sock);
}

/* ------------------------------------------------------------------- TCP */

static INGEST_APP_ReadResult_t INGEST_APP_ReadExact(osal_id_t Sock, void *Dest, uint32 Size)
{
    uint8 *P   = (uint8 *)Dest;
    uint32 Got = 0;
    int32  N;

    while (Got < Size)
    {
        N = OS_TimedRead(Sock, P + Got, Size - Got, INGEST_APP_SOCKET_TIMEOUT_MS);
        if (N > 0)
        {
            Got += (uint32)N;
        }
        else if (N == 0)
        {
            return INGEST_APP_READ_CLOSED;
        }
        else if (N != OS_ERROR_TIMEOUT || !INGEST_APP_Data.SocketsWanted || !INGEST_APP_Running())
        {
            return INGEST_APP_READ_ERROR;
        }
    }
    return INGEST_APP_READ_OK;
}

static INGEST_APP_ReadResult_t INGEST_APP_Drain(osal_id_t Sock, uint32 Size)
{
    static uint8            Scratch[INGEST_APP_DRAIN_SIZE];
    INGEST_APP_ReadResult_t R = INGEST_APP_READ_OK;
    uint32                  Chunk;

    while (Size > 0 && R == INGEST_APP_READ_OK)
    {
        Chunk = (Size < sizeof(Scratch)) ? Size : (uint32)sizeof(Scratch);
        R     = INGEST_APP_ReadExact(Sock, Scratch, Chunk);
        Size -= Chunk;
    }
    return R;
}

/* A frame that breaks the protocol: the frame boundaries can no longer be
 * trusted, so the connection is closed. */
static INGEST_APP_ReadResult_t INGEST_APP_RejectFrame(const char *Reason)
{
    ++INGEST_APP_Data.Counters.FramesRejected;
    CFE_EVS_SendEvent(INGEST_APP_FRAME_REJECTED_EID, CFE_EVS_EventType_ERROR,
                      "image channel: frame rejected (%s), connection closed", Reason);
    INGEST_APP_FaultReport(OST_FaultId_LINK_FORMAT, Reason);
    return INGEST_APP_READ_ERROR;
}

static void INGEST_APP_PublishCalibration(const OST_Link_SessionStart_t *S)
{
    static INGEST_APP_SessionCalibrationTlm_t Msg;
    INGEST_APP_SessionCalibration_Payload_t  *P = &Msg.Payload;
    uint32                                    Seq = P->Header.Sequence;

    memset(&Msg, 0, sizeof(Msg));
    CFE_MSG_Init(CFE_MSG_PTR(Msg.TelemetryHeader),
                 INGEST_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_SESSION_CALIBRATION_TLM_TOPICID), sizeof(Msg));
    P->Header.SchemaVersion = INGEST_APP_SCHEMA_VERSION;
    P->Header.Sequence      = Seq + 1;
    P->SessionId            = S->session_id;
    P->StartUtc             = S->start_ns;
    P->RecordedStartUtc     = S->recorded_start_ns;
    P->TimeFactor           = S->time_factor;
    P->Width                = S->width;
    P->Height               = S->height;
    P->Fx                   = S->fx;
    P->Fy                   = S->fy;
    P->Cx                   = S->cx;
    P->Cy                   = S->cy;
    P->K1                   = S->k1;
    P->K2                   = S->k2;
    P->K3                   = S->k3;
    P->P1                   = S->p1;
    P->P2                   = S->p2;
    P->BodyFromCamera.W     = S->q_b_c[0];
    P->BodyFromCamera.X     = S->q_b_c[1];
    P->BodyFromCamera.Y     = S->q_b_c[2];
    P->BodyFromCamera.Z     = S->q_b_c[3];
    P->CameraInBody.X       = S->t_b_c[0];
    P->CameraInBody.Y       = S->t_b_c[1];
    P->CameraInBody.Z       = S->t_b_c[2];
    OST_CopyText(P->CameraName, sizeof(P->CameraName), S->camera_name);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Msg.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(Msg.TelemetryHeader), true);
}

static INGEST_APP_ReadResult_t INGEST_APP_ReceiveImage(osal_id_t Sock, const OST_Link_Header_t *H, int64 EntryMono)
{
    static INGEST_APP_ImageArrivedTlm_t Msg;
    static bool                         MsgReady = false;
    INGEST_APP_Counters_t              *C        = &INGEST_APP_Data.Counters;
    uint8                               Fixed[OST_LINK_IMAGE_SIZE];
    OST_Link_Image_t                    Img;
    OST_ObjectRef_t                     Ref;
    void                               *Data = NULL;
    const char                         *Reason = NULL;
    INGEST_APP_ReadResult_t             R;
    OST_Status_t                        Status;

    if (!MsgReady)
    {
        memset(&Msg, 0, sizeof(Msg));
        CFE_MSG_Init(CFE_MSG_PTR(Msg.TelemetryHeader),
                     INGEST_APP_TlmMid(EdsParam_CFE_MISSION_INGEST_APP_IMAGE_ARRIVED_TLM_TOPICID), sizeof(Msg));
        MsgReady = true;
    }

    R = INGEST_APP_ReadExact(Sock, Fixed, sizeof(Fixed));
    if (R != INGEST_APP_READ_OK)
    {
        return R;
    }
    if (OST_Link_DecodeImage(H, Fixed, sizeof(Fixed), &Img, &Reason) != OST_SUCCESS)
    {
        return INGEST_APP_RejectFrame(Reason);
    }
    if (INGEST_APP_Data.State != INGEST_APP_SessionState_ACTIVE || Img.session_id != INGEST_APP_Data.SessionId)
    {
        return INGEST_APP_RejectFrame("image outside its session");
    }

    Status = OST_DS_Allocate(OST_DATASTORE_JPEG, Img.data_size, &Ref, &Data);
    if (Status != OST_SUCCESS)
    {
        ++C->ImagesDropped;
        CFE_EVS_SendEvent(INGEST_APP_IMAGE_DROPPED_EID, CFE_EVS_EventType_ERROR,
                          "image %lu: not stored (%s), %lu bytes discarded", (unsigned long)Img.image_seq,
                          OST_StatusName(Status), (unsigned long)Img.data_size);
        INGEST_APP_FaultReport(OST_FaultId_DATASTORE_QUOTA, "JPEG not stored");
        return INGEST_APP_Drain(Sock, Img.data_size);
    }
    R = INGEST_APP_ReadExact(Sock, Data, Img.data_size);
    if (R != INGEST_APP_READ_OK)
    {
        OST_DS_Abort(&Ref);
        return R;
    }
    OST_DS_Seal(&Ref);

    {
        INGEST_APP_ImageArrived_Payload_t *P = &Msg.Payload;
        P->Header.SchemaVersion = INGEST_APP_SCHEMA_VERSION;
        ++P->Header.Sequence;
        P->CaptureUtc  = Img.capture_ns;
        P->RecordedUtc = Img.recorded_ns;
        P->SessionId   = Img.session_id;
        P->ImageSeq    = Img.image_seq;
        P->Jpeg        = Ref;
        P->JpegSize    = Img.data_size;
        P->Trace.EntryMono  = EntryMono;
        P->Trace.ExitMono   = OST_TimeMonoNs();
        P->Trace.PublishUtc = OST_TimeUtcNs();
    }
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Msg.TelemetryHeader));
    if (CFE_SB_TransmitMsg(CFE_MSG_PTR(Msg.TelemetryHeader), true) != CFE_SUCCESS)
    {
        OST_DS_Release(&Ref); /* nobody owns the count */
    }
    ++C->ImagesReceived;
    C->ImageBytes += Img.data_size;
    C->LastImageMono = OST_TimeMonoNs();
    return INGEST_APP_READ_OK;
}

static INGEST_APP_ReadResult_t INGEST_APP_ReceiveFrame(osal_id_t Sock)
{
    uint8                   Head[OST_LINK_HEADER_SIZE];
    uint8                   Payload[OST_LINK_SESSION_START_SIZE];
    OST_Link_Header_t       H;
    OST_Link_SessionStart_t S;
    OST_Link_SessionEnd_t   E;
    const char             *Reason = NULL;
    INGEST_APP_ReadResult_t R;
    int64                   EntryMono;

    R = INGEST_APP_ReadExact(Sock, Head, sizeof(Head));
    if (R != INGEST_APP_READ_OK)
    {
        return R;
    }
    EntryMono = OST_TimeMonoNs();
    if (OST_Link_DecodeHeader(Head, sizeof(Head), &H, &Reason) != OST_SUCCESS)
    {
        return INGEST_APP_RejectFrame(Reason);
    }

    switch (H.type)
    {
        case OST_LINK_TYPE_IMAGE:
            return INGEST_APP_ReceiveImage(Sock, &H, EntryMono);

        case OST_LINK_TYPE_SESSION_START:
            R = INGEST_APP_ReadExact(Sock, Payload, H.length);
            if (R != INGEST_APP_READ_OK)
            {
                return R;
            }
            if (OST_Link_DecodeSessionStart(&H, Payload, H.length, &S, &Reason) != OST_SUCCESS)
            {
                return INGEST_APP_RejectFrame(Reason);
            }
            INGEST_APP_Data.SessionId                 = S.session_id;
            INGEST_APP_Data.Counters.SessionStartMono = OST_TimeMonoNs();
            INGEST_APP_Data.State                     = INGEST_APP_SessionState_ACTIVE;
            INGEST_APP_PublishCalibration(&S);
            CFE_EVS_SendEvent(INGEST_APP_SESSION_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "session %lu started: camera %s, %lux%lu px, time factor %.3f",
                              (unsigned long)S.session_id, S.camera_name, (unsigned long)S.width,
                              (unsigned long)S.height, S.time_factor);
            return INGEST_APP_READ_OK;

        case OST_LINK_TYPE_SESSION_END:
            R = INGEST_APP_ReadExact(Sock, Payload, H.length);
            if (R != INGEST_APP_READ_OK)
            {
                return R;
            }
            if (OST_Link_DecodeSessionEnd(&H, Payload, H.length, &E, &Reason) != OST_SUCCESS)
            {
                return INGEST_APP_RejectFrame(Reason);
            }
            INGEST_APP_Data.State = INGEST_APP_SessionState_CONNECTED;
            CFE_EVS_SendEvent(INGEST_APP_SESSION_END_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "session %lu ended: images %lu sent, %lu received, %lu dropped; telemetry %lu, %lu lost",
                              (unsigned long)E.session_id, (unsigned long)E.images_sent,
                              (unsigned long)INGEST_APP_Data.Counters.ImagesReceived,
                              (unsigned long)INGEST_APP_Data.Counters.ImagesDropped,
                              (unsigned long)INGEST_APP_Data.Counters.TelemetryReceived,
                              (unsigned long)INGEST_APP_Data.Counters.TelemetryLost);
            return INGEST_APP_READ_OK;

        default:
            return INGEST_APP_RejectFrame("telemetry message on the image channel");
    }
}

void INGEST_APP_TcpTask(void)
{
    osal_id_t     Listen = OS_OBJECT_ID_UNDEFINED;
    osal_id_t     Conn   = OS_OBJECT_ID_UNDEFINED;
    OS_SockAddr_t Peer;
    char          PeerText[48];
    int32         Status;

    while (INGEST_APP_Running())
    {
        if (!INGEST_APP_Data.SocketsWanted)
        {
            INGEST_APP_CloseSocket(&Conn);
            INGEST_APP_CloseSocket(&Listen);
            INGEST_APP_Data.State = INGEST_APP_SessionState_STOPPED;
            OS_TaskDelay(INGEST_APP_SOCKET_TIMEOUT_MS);
            continue;
        }
        if (!OS_ObjectIdDefined(Listen))
        {
            if (!INGEST_APP_OpenSocket(&Listen, OS_SocketType_STREAM, INGEST_APP_Data.Config.TcpPort, "TCP"))
            {
                OS_TaskDelay(1000);
                continue;
            }
            INGEST_APP_Data.State = INGEST_APP_SessionState_LISTENING;
        }
        if (!OS_ObjectIdDefined(Conn))
        {
            Status = OS_SocketAccept(Listen, &Conn, &Peer, INGEST_APP_SOCKET_TIMEOUT_MS);
            if (Status != OS_SUCCESS)
            {
                Conn = OS_OBJECT_ID_UNDEFINED;
                continue;
            }
            ++INGEST_APP_Data.Counters.Connections;
            INGEST_APP_Data.State = INGEST_APP_SessionState_CONNECTED;
            if (OS_SocketAddrToString(PeerText, sizeof(PeerText), &Peer) != OS_SUCCESS)
            {
                OST_CopyText(PeerText, sizeof(PeerText), "?");
            }
            CFE_EVS_SendEvent(INGEST_APP_CONNECTED_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "image channel: connection from %s", PeerText);
        }

        if (INGEST_APP_ReceiveFrame(Conn) != INGEST_APP_READ_OK)
        {
            if (INGEST_APP_Data.State == INGEST_APP_SessionState_ACTIVE)
            {
                CFE_EVS_SendEvent(INGEST_APP_DISCONNECTED_INF_EID, CFE_EVS_EventType_ERROR,
                                  "session %lu: image connection lost before SESSION_END",
                                  (unsigned long)INGEST_APP_Data.SessionId);
            }
            else
            {
                CFE_EVS_SendEvent(INGEST_APP_DISCONNECTED_INF_EID, CFE_EVS_EventType_INFORMATION,
                                  "image channel: connection closed");
            }
            INGEST_APP_CloseSocket(&Conn);
            INGEST_APP_Data.State = INGEST_APP_SessionState_LISTENING;
        }
    }
    INGEST_APP_CloseSocket(&Conn);
    INGEST_APP_CloseSocket(&Listen);
}
