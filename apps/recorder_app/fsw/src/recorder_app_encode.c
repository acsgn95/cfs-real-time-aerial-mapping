/*
 * RECORDER_APP: encoding of a Software Bus message as on the ground link
 * (EDS packed), so that a recording holds the same bytes as TO_LAB sends
 * and the ground tools decode both alike.
 *
 * Follows TO_LAB's EDS encoder (vendor/cfs/apps/to_lab/fsw/src/
 * to_lab_eds_encode.c, NASA, Apache License 2.0), reduced to messages with
 * an EDS definition: telemetry header and payload packed through the EDS
 * runtime database of the mission.
 */
#include "recorder_app.h"

#include "cfe_config.h"
#include "cfe_hdr_eds_datatypes.h"
#include "cfe_mission_eds_interface_parameters.h"
#include "cfe_mission_eds_parameters.h"
#include "cfe_missionlib_api.h"
#include "cfe_missionlib_runtime.h"
#include "edslib_datatypedb.h"
#include "edslib_global.h"
#include "edslib_intfdb.h"

#include <string.h>

CFE_Status_t RECORDER_APP_Encode(const CFE_SB_Buffer_t *Buf, const void **Encoded, size_t *Size)
{
    static EdsPackedBuffer_CFE_HDR_TelemetryHeader_t Packed; /* main task only */
    static const EdsLib_Id_t TELEMETRY_ID = EDSLIB_INTF_ID(EDS_INDEX(CFE_SB), EdsCommand_CFE_SB_Telemetry_indication_DECLARATION);
    const EdsLib_DatabaseObject_t        *EdsDb = CFE_Config_GetObjPointer(CFE_CONFIGID_MISSION_EDS_DB);
    CFE_MissionLib_TopicInfo_t            TopicInfo;
    CFE_SB_SoftwareBus_PubSub_Interface_t PubSub;
    CFE_SB_Publisher_Component_t          Publisher;
    EdsLib_SizeInfo_t                     MaxSize;
    EdsLib_SizeInfo_t                     Processed;
    EdsLib_Id_t                           EdsId;
    CFE_MSG_Size_t                        MsgSize = 0;

    *Encoded = NULL;
    *Size    = 0;
    if (CFE_MSG_GetSize(&Buf->Msg, &MsgSize) != CFE_SUCCESS)
    {
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    memset(&MaxSize, 0, sizeof(MaxSize));
    memset(&Processed, 0, sizeof(Processed));
    MaxSize.Bits  = EdsLib_OCTETS_TO_BITS(sizeof(Packed));
    MaxSize.Bytes = MsgSize;

    CFE_MissionLib_Get_PubSub_Parameters(&PubSub, &Buf->Msg.BaseMsg);
    CFE_MissionLib_UnmapPublisherComponent(&Publisher, &PubSub);
    if (CFE_MissionLib_GetTopicInfo(&CFE_SOFTWAREBUS_INTERFACE, Publisher.Telemetry.TopicId, &TopicInfo) !=
            CFE_MISSIONLIB_SUCCESS ||
        !EdsLib_Is_Valid(TopicInfo.ParentIntfId))
    {
        return CFE_STATUS_UNKNOWN_MSG_ID; /* no EDS definition */
    }
    if (EdsLib_IntfDB_FindAllArgumentTypes(EdsDb, TELEMETRY_ID, TopicInfo.ParentIntfId, &EdsId, 1) != EDSLIB_SUCCESS ||
        EdsLib_DataTypeDB_PackPartialObjectVarSize(EdsDb, &EdsId, Packed, Buf, &MaxSize, &Processed) !=
            EDSLIB_SUCCESS ||
        EdsLib_DataTypeDB_FinalizePackedObjectVarSize(EdsDb, EdsId, Buf, Packed, &Processed) != EDSLIB_SUCCESS)
    {
        return CFE_STATUS_VALIDATION_FAILURE;
    }
    *Encoded = Packed;
    *Size    = EdsLib_BITS_TO_OCTETS(Processed.Bits);
    return CFE_SUCCESS;
}
