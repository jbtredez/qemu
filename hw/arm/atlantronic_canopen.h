#ifndef ATLANTRONIC_CANOPEN_H
#define ATLANTRONIC_CANOPEN_H

#include "atlantronic_can.h"

enum
{
	CANOPEN_RX_PDO1         = 0x180,
	CANOPEN_RX_PDO2         = 0x280,
	CANOPEN_RX_PDO3         = 0x380,
	CANOPEN_SDO_RES         = 0x580,
	CANOPEN_SDO_RX          = 0x600,
	CANOPEN_BOOTUP          = 0x700,
};

typedef void (*atlantronic_canopen_callback)(void* can_interface, void* opaque, struct can_msg msg, int type);

void atlantronic_canopen_tx(void* can_interface, struct can_msg msg);

int atlantronic_canopen_register_node(uint8_t nodeid, void* opaque, atlantronic_canopen_callback callback);

#endif
