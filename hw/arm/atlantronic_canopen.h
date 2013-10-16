#ifndef ATLANTRONIC_CANOPEN_H
#define ATLANTRONIC_CANOPEN_H

#include "atlantronic_can.h"

typedef void (*atlantronic_canopen_callback)(void* opaque, struct can_msg msg, int type);

void atlantronic_canopen_tx(void* can_interface, struct can_msg msg);

int atlantronic_canopen_register_node(uint8_t nodeid, void* opaque, atlantronic_canopen_callback callback);

#endif
