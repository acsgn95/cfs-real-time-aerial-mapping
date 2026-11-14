/*
 * Uses the C interface from C99, as INGEST_APP does: decode one telemetry
 * datagram. Returns 0 on success or the number of the failed check.
 */

#include "link_c_api.h"

int ost_link_c_api_usage(const uint8_t *datagram, size_t size)
{
    OST_Link_Datagram_t d;
    const char         *reason = NULL;

    if (OST_Link_DecodeDatagram(datagram, size, &d, &reason) != OST_SUCCESS)
    {
        return 1;
    }
    if (d.header.type != OST_LINK_TYPE_TELEMETRY || reason == NULL || reason[0] != '\0')
    {
        return 2;
    }
    if (d.u.telemetry.session_id != 7u)
    {
        return 3;
    }
    return 0;
}
