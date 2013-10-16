#ifndef ATLANTRONIC_CANOPEN_H
#define ATLANTRONIC_CANOPEN_H

#include "atlantronic_can.h"

enum
{
	CANOPEN_NMT = 0,
	CANOPEN_SYNC,
	CANOPEN_EMERGENCY,
	CANOPEN_RX_PDO1,
	CANOPEN_TX_PDO1,
	CANOPEN_RX_PDO2,
	CANOPEN_TX_PDO2,
	CANOPEN_RX_PDO3,
	CANOPEN_TX_PDO3,
	CANOPEN_RX_PDO4,
	CANOPEN_TX_PDO4,
	CANOPEN_SDO_RES,
	CANOPEN_SDO_REQ,
	CANOPEN_RESERVED_13,
	CANOPEN_BOOTUP,
};

typedef void (*atlantronic_canopen_callback)(void* can_interface, void* opaque, struct can_msg msg, int type);

void atlantronic_canopen_tx(void* can_interface, struct can_msg msg);

int atlantronic_canopen_register_node(uint8_t nodeid, void* opaque, atlantronic_canopen_callback callback);

#endif
