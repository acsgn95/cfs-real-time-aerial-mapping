/*
 * RECORDER_APP run directory: recording file, manifest and free space
 * (recorder_app.h).
 */
#include "recorder_app.h"
#include "recorder_app_eventids.h"
#include "recorder_app_version.h"

#include "recorder_app_eds_designparameters.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define RECORDER_APP_HEADER_SIZE 64u
#define RECORDER_APP_FORMAT      1u

static void RECORDER_APP_PutU32(uint8 *p, uint32 v)
{
    p[0] = (uint8)v, p[1] = (uint8)(v >> 8), p[2] = (uint8)(v >> 16), p[3] = (uint8)(v >> 24);
}

static void RECORDER_APP_PutI64(uint8 *p, int64 v)
{
    RECORDER_APP_PutU32(p, (uint32)((uint64)v & 0xFFFFFFFFu));
    RECORDER_APP_PutU32(p + 4, (uint32)((uint64)v >> 32));
}

bool RECORDER_APP_ValidRunId(const char *RunId)
{
    size_t i;

    if (RunId[0] == '\0' || RunId[0] == '.')
    {
        return false;
    }
    for (i = 0; RunId[i] != '\0'; ++i)
    {
        if (!isalnum((unsigned char)RunId[i]) && RunId[i] != '-' && RunId[i] != '_' && RunId[i] != '.')
        {
            return false;
        }
    }
    /* The run directory is one path component: OSAL limits it to
     * OS_MAX_FILE_NAME - 1 characters. */
    return i <= RECORDER_APP_RUN_ID_MAX && i < OS_MAX_FILE_NAME;
}

static OST_Status_t RECORDER_APP_WriteAll(osal_id_t File, const void *Data, size_t Size)
{
    const uint8 *p = (const uint8 *)Data;
    int32        n;

    while (Size > 0)
    {
        n = OS_write(File, p, Size);
        if (n <= 0)
        {
            return OST_ERR_IO;
        }
        p += n;
        Size -= (size_t)n;
    }
    return OST_SUCCESS;
}

static uint8 RECORDER_APP_Buffer[RECORDER_APP_BUFFER_BYTES];

static void RECORDER_APP_Give(void)
{
    OS_BinSemGive(RECORDER_APP_Data.DataSem);
}

static uint64 RECORDER_APP_Waiting(void)
{
    uint64 n;

    OS_MutSemTake(RECORDER_APP_Data.BufferMutex);
    n = RECORDER_APP_Data.Head - RECORDER_APP_Data.Tail;
    OS_MutSemGive(RECORDER_APP_Data.BufferMutex);
    return n;
}

/* Copies into the buffer at the head; the caller checked the space. */
static void RECORDER_APP_Put(uint64 At, const void *Data, size_t Size)
{
    const uint8 *p     = (const uint8 *)Data;
    size_t       Off   = (size_t)(At % RECORDER_APP_BUFFER_BYTES);
    size_t       First = RECORDER_APP_BUFFER_BYTES - Off;

    if (First > Size)
    {
        First = Size;
    }
    memcpy(RECORDER_APP_Buffer + Off, p, First);
    memcpy(RECORDER_APP_Buffer, p + First, Size - First);
}

/* manifest.json of the run: written at the start and rewritten at the
 * stop. The ground tools add what the flight software does not know (git
 * commit, injector parameters, sampler; ADR-017 section 4). */
static void RECORDER_APP_WriteManifest(const char *State)
{
    RECORDER_APP_Data_t *D = &RECORDER_APP_Data;
    char                 Path[OS_MAX_PATH_LEN];
    char                 Text[4096];
    size_t               Len;
    osal_id_t            File;
    uint32               i;

    snprintf(Path, sizeof(Path), "%s/%s", D->RunDir, RECORDER_APP_MANIFEST_NAME);
    Len = (size_t)snprintf(Text, sizeof(Text),
                           "{\n  \"run_id\": \"%s\",\n  \"state\": \"%s\",\n  \"recorder_version\": \"%s\",\n"
                           "  \"format\": %u,\n  \"recording\": \"%s\",\n  \"start_utc_ns\": %lld,\n"
                           "  \"stop_utc_ns\": %lld,\n  \"records\": %lu,\n  \"bytes\": %llu,\n"
                           "  \"encode_errors\": %lu,\n  \"write_errors\": %lu,\n  \"dropped\": %lu,\n"
                           "  \"buffer_max_bytes\": %lu,\n  \"max_write_ms\": %.1f,\n  \"subscriptions\": [",
                           D->RunId, State, RECORDER_APP_VERSION, RECORDER_APP_FORMAT, RECORDER_APP_RECORDING_NAME,
                           (long long)D->StartUtc, strcmp(State, "recording") == 0 ? 0LL : (long long)OST_TimeUtcNs(),
                           (unsigned long)D->Records, (unsigned long long)D->Bytes, (unsigned long)D->EncodeErrors,
                           (unsigned long)D->WriteErrors, (unsigned long)D->Dropped, (unsigned long)D->BufferMax,
                           (double)D->MaxWriteNs * 1e-6);
    for (i = 0; i < EdsParam_RECORDER_APP_MAX_SUBSCRIPTIONS && D->Config.Subscriptions[i].MsgId != 0 &&
                Len + 32 < sizeof(Text);
         ++i)
    {
        Len += (size_t)snprintf(Text + Len, sizeof(Text) - Len, "%s\"0x%04lX\"", i > 0 ? ", " : "",
                                (unsigned long)D->Config.Subscriptions[i].MsgId);
    }
    Len += (size_t)snprintf(Text + Len, sizeof(Text) - Len, "]\n}\n");

    if (OS_OpenCreate(&File, Path, OS_FILE_FLAG_CREATE | OS_FILE_FLAG_TRUNCATE, OS_WRITE_ONLY) != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_FILE_ERR_EID, CFE_EVS_EventType_ERROR, "manifest %s not written", Path);
        return;
    }
    if (RECORDER_APP_WriteAll(File, Text, Len) != OST_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_FILE_ERR_EID, CFE_EVS_EventType_ERROR, "manifest %s not written", Path);
    }
    OS_close(File);
}

CFE_Status_t RECORDER_APP_Start(const char *RunId)
{
    RECORDER_APP_Data_t *D = &RECORDER_APP_Data;
    char                 Path[OS_MAX_PATH_LEN];
    uint8                Header[RECORDER_APP_HEADER_SIZE];
    char                 Out[OS_MAX_PATH_LEN];
    time_t               Now;
    struct tm            Utc;
    int32                Status;

    if (D->Recording)
    {
        RECORDER_APP_Stop("new run");
    }
    D->StartUtc = OST_TimeUtcNs();
    if (RunId == NULL || RunId[0] == '\0')
    {
        Now = (time_t)(D->StartUtc / 1000000000);
        gmtime_r(&Now, &Utc);
        strftime(D->RunId, sizeof(D->RunId), "run_%Y%m%d_%H%M%S", &Utc);
    }
    else
    {
        snprintf(D->RunId, sizeof(D->RunId), "%s", RunId);
    }

    snprintf(Out, sizeof(Out), "%.*s", (int)sizeof(D->Config.OutputDir), D->Config.OutputDir);
    snprintf(D->RunDir, sizeof(D->RunDir), "%s/%s", Out, D->RunId);
    OS_mkdir(Out, 0);  /* may exist */
    Status = OS_mkdir(D->RunDir, 0);
    snprintf(Path, sizeof(Path), "%s/%s", D->RunDir, RECORDER_APP_RECORDING_NAME);
    if (Status == OS_SUCCESS)
    {
        /* An existing run directory is not overwritten: run identifiers name
         * measurements. */
        OS_MutSemTake(D->FileMutex);
        Status = OS_OpenCreate(&D->File, Path, OS_FILE_FLAG_CREATE | OS_FILE_FLAG_TRUNCATE, OS_WRITE_ONLY);
        if (Status != OS_SUCCESS)
        {
            D->File = OS_OBJECT_ID_UNDEFINED;
        }
        OS_MutSemGive(D->FileMutex);
    }
    if (Status != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_FILE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "run %s not started: %s not created (%ld; the run directory must not exist)", D->RunId,
                          Path, (long)Status);
        OST_FaultSet_Report(&D->Faults, OST_FaultId_RECORDING_FAILED, "recording file not created");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    /* Header: magic, size, format, start time, run identifier (ASCII,
     * zero padded); little-endian. */
    memset(Header, 0, sizeof(Header));
    memcpy(Header, "OSTREC\0\0", 8);
    RECORDER_APP_PutU32(Header + 8, RECORDER_APP_HEADER_SIZE);
    RECORDER_APP_PutU32(Header + 12, RECORDER_APP_FORMAT);
    RECORDER_APP_PutI64(Header + 16, D->StartUtc);
    memcpy(Header + 24, D->RunId, strlen(D->RunId) < 32 ? strlen(D->RunId) : 32);
    OS_MutSemTake(D->FileMutex);
    Status = RECORDER_APP_WriteAll(D->File, Header, sizeof(Header)) == OST_SUCCESS ? OS_SUCCESS : OS_ERROR;
    if (Status != OS_SUCCESS)
    {
        OS_close(D->File);
        D->File = OS_OBJECT_ID_UNDEFINED;
    }
    OS_MutSemGive(D->FileMutex);
    if (Status != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(RECORDER_APP_FILE_ERR_EID, CFE_EVS_EventType_ERROR, "run %s: header not written",
                          D->RunId);
        OST_FaultSet_Report(&D->Faults, OST_FaultId_RECORDING_FAILED, "recording file not written");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    D->Recording    = true;
    D->Records      = 0;
    D->Bytes        = sizeof(Header);
    D->EncodeErrors = 0;
    D->Dropped      = 0;
    D->WriteErrors  = 0;
    D->MaxWriteNs   = 0;
    D->BufferMax    = 0;
    D->WriteFailed  = false;
    ++D->Runs;
    RECORDER_APP_WriteManifest("recording");
    CFE_EVS_SendEvent(RECORDER_APP_START_INF_EID, CFE_EVS_EventType_INFORMATION, "recording run %s to %s", D->RunId,
                      D->RunDir);
    return CFE_SUCCESS;
}

void RECORDER_APP_Stop(const char *Reason)
{
    RECORDER_APP_Data_t *D = &RECORDER_APP_Data;

    int64 Start = OST_TimeMonoNs();

    if (!D->Recording)
    {
        return;
    }
    D->Recording = false;
    /* The writer empties the buffer first. */
    while (RECORDER_APP_Waiting() > 0 && OST_TimeMonoNs() - Start < (int64)RECORDER_APP_DRAIN_WAIT_MS * 1000000)
    {
        RECORDER_APP_Give();
        OS_TaskDelay(10);
    }
    OS_MutSemTake(D->FileMutex);
    OS_close(D->File);
    D->File = OS_OBJECT_ID_UNDEFINED;
    OS_MutSemGive(D->FileMutex);
    /* Anything left (the writer did not finish in time) is discarded. */
    OS_MutSemTake(D->BufferMutex);
    D->Tail = D->Head;
    OS_MutSemGive(D->BufferMutex);
    RECORDER_APP_WriteManifest("stopped");
    CFE_EVS_SendEvent(RECORDER_APP_STOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "run %s stopped (%s): %lu records, %.1f MB, %lu without EDS, %lu dropped, %lu write errors, "
                      "longest write %.0f ms",
                      D->RunId, Reason, (unsigned long)D->Records, (double)D->Bytes / (1024.0 * 1024.0),
                      (unsigned long)D->EncodeErrors, (unsigned long)D->Dropped, (unsigned long)D->WriteErrors,
                      (double)D->MaxWriteNs * 1e-6);
}

void RECORDER_APP_Record(const CFE_SB_Buffer_t *Buf)
{
    RECORDER_APP_Data_t *D = &RECORDER_APP_Data;
    const void          *Encoded = NULL;
    size_t               Size    = 0;
    uint8                Frame[12];
    uint64               Used;

    if (!D->Recording)
    {
        return;
    }
    RECORDER_APP_CheckWriter();
    if (!D->Recording)
    {
        return;
    }
    if (RECORDER_APP_Encode(Buf, &Encoded, &Size) != CFE_SUCCESS)
    {
        CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

        CFE_MSG_GetMsgId(&Buf->Msg, &MsgId);
        ++D->EncodeErrors;
        CFE_EVS_SendEvent(RECORDER_APP_ENCODE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "message 0x%04lX has no EDS encoding, not recorded",
                          (unsigned long)CFE_SB_MsgIdToValue(MsgId));
        return;
    }
    RECORDER_APP_PutU32(Frame, (uint32)Size);
    RECORDER_APP_PutI64(Frame + 4, OST_TimeUtcNs());
    OS_MutSemTake(D->BufferMutex);
    Used = D->Head - D->Tail;
    if (Used + sizeof(Frame) + Size > RECORDER_APP_BUFFER_BYTES)
    {
        OS_MutSemGive(D->BufferMutex);
        ++D->Dropped;
        CFE_EVS_SendEvent(RECORDER_APP_WRITE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "run %s: buffer full (writer behind), message not recorded", D->RunId);
        return;
    }
    RECORDER_APP_Put(D->Head, Frame, sizeof(Frame));
    RECORDER_APP_Put(D->Head + sizeof(Frame), Encoded, Size);
    D->Head += sizeof(Frame) + Size;
    Used += sizeof(Frame) + Size;
    if (Used > D->BufferMax)
    {
        D->BufferMax = (uint32)Used;
    }
    OS_MutSemGive(D->BufferMutex);
    RECORDER_APP_Give();
    ++D->Records;
    D->Bytes += sizeof(Frame) + Size;
    if (D->Config.MaxRecordingMb > 0 && D->Bytes >= (uint64)D->Config.MaxRecordingMb * 1024u * 1024u)
    {
        CFE_EVS_SendEvent(RECORDER_APP_DISK_EID, CFE_EVS_EventType_ERROR, "run %s reached %lu MB, recording stops",
                          D->RunId, (unsigned long)D->Config.MaxRecordingMb);
        RECORDER_APP_Stop("size limit");
    }
}

void RECORDER_APP_CheckSpace(void)
{
    RECORDER_APP_Data_t *D = &RECORDER_APP_Data;
    OS_statvfs_t         Stat;
    char                 Out[OS_MAX_PATH_LEN];
    double               FreeMb;

    snprintf(Out, sizeof(Out), "%.*s", (int)sizeof(D->Config.OutputDir), D->Config.OutputDir);
    if (OS_FileSysStatVolume(Out, &Stat) != OS_SUCCESS)
    {
        return;
    }
    FreeMb    = (double)Stat.blocks_free * (double)Stat.block_size / (1024.0 * 1024.0);
    D->FreeMb = (float)FreeMb;
    if (FreeMb < (double)D->Config.WarnFreeMb)
    {
        OST_FaultSet_Raise(&D->Faults, OST_FaultId_DISK_SPACE_LOW, "output disk space low");
    }
    else
    {
        OST_FaultSet_Resolve(&D->Faults, OST_FaultId_DISK_SPACE_LOW);
    }
    if (D->Recording && FreeMb < (double)D->Config.MinFreeMb)
    {
        CFE_EVS_SendEvent(RECORDER_APP_DISK_EID, CFE_EVS_EventType_ERROR,
                          "run %s: %.0f MB free, below %lu MB, recording stops", D->RunId, FreeMb,
                          (unsigned long)D->Config.MinFreeMb);
        RECORDER_APP_Stop("disk space");
    }
}

/* A failed write of the writer task stops the run (main task: the fault
 * set and the run belong to it). */
void RECORDER_APP_CheckWriter(void)
{
    RECORDER_APP_Data_t *D = &RECORDER_APP_Data;

    if (D->Recording && D->WriteFailed)
    {
        CFE_EVS_SendEvent(RECORDER_APP_WRITE_ERR_EID, CFE_EVS_EventType_ERROR, "run %s: write failed, recording stops",
                          D->RunId);
        OST_FaultSet_Report(&D->Faults, OST_FaultId_RECORDING_FAILED, "recording file not written");
        RECORDER_APP_Stop("write failed");
    }
}

void RECORDER_APP_WriterTask(void)
{
    RECORDER_APP_Data_t *D = &RECORDER_APP_Data;
    uint64               Tail, Waiting;
    size_t               Off, Chunk;
    int64                T0, Ns;
    OST_Status_t         Status;

    for (;;)
    {
        OS_BinSemTimedWait(D->DataSem, 100);
        for (;;)
        {
            OS_MutSemTake(D->BufferMutex);
            Tail    = D->Tail;
            Waiting = D->Head - D->Tail;
            OS_MutSemGive(D->BufferMutex);
            if (Waiting == 0)
            {
                break;
            }
            Off   = (size_t)(Tail % RECORDER_APP_BUFFER_BYTES);
            Chunk = RECORDER_APP_BUFFER_BYTES - Off;
            if (Chunk > Waiting)
            {
                Chunk = (size_t)Waiting;
            }
            OS_MutSemTake(D->FileMutex);
            if (OS_ObjectIdDefined(D->File) && !D->WriteFailed)
            {
                T0     = OST_TimeMonoNs();
                Status = RECORDER_APP_WriteAll(D->File, RECORDER_APP_Buffer + Off, Chunk);
                Ns     = OST_TimeMonoNs() - T0;
                if (Ns > D->MaxWriteNs)
                {
                    D->MaxWriteNs = Ns;
                }
                if (Status != OST_SUCCESS)
                {
                    ++D->WriteErrors;
                    D->WriteFailed = true;
                }
            }
            OS_MutSemGive(D->FileMutex);
            OS_MutSemTake(D->BufferMutex);
            if (D->Tail == Tail) /* not discarded by a stop meanwhile */
            {
                D->Tail += Chunk;
            }
            OS_MutSemGive(D->BufferMutex);
        }
        if (D->WriterStop)
        {
            break;
        }
    }
    OS_BinSemGive(D->WriterDone);
}
