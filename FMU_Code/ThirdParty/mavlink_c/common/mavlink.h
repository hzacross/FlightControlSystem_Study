/** @file
 *  @brief MAVLink comm protocol built from common.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_PRIMARY_XML_IDX 0

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT 1
#endif

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length);
/* use efficient approach, see mavlink_helpers.h */
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

#include "version.h"
#include "common.h"

void Mavlin_RX_InterruptHandler(void);
void Loop_Mavlink_Parse(void);
void Mavlink_Init(void);
void mavlink_test(void);
void Mavlink_Rece_Enable(void);
#endif // MAVLINK_H
