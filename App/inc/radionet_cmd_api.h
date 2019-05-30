/*!
 * \file		radionet_cmd_api.h
 *
 * \brief		Application command API - definition of RX packet handlers
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#ifndef __RADIONET_CMD_API_H__
#define __RADIONET_CMD_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "radio_protocol.h"

void rxGWPacketHandler(radioprot_gw_packet_t *packet);

#endif
