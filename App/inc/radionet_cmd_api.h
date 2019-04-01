/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADIONET_CMD_API_H__
#define __RADIONET_CMD_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_protocol.h"

void gwPacketHandler(radioprot_gw_packet_t *packet, uint8_t rssi);

#endif
