#ifndef ATLANTRONIC_CANOPEN_H
#define ATLANTRONIC_CANOPEN_H

#include "atlantronic_can.h"
#include "hw/irq.h"

#define CANOPEN_MAX_NODE          128

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

struct canopen_node;
struct atlantronic_canopen;

typedef void (*atlantronic_canopen_callback)(struct atlantronic_canopen* canopen, struct canopen_node* node, struct can_msg msg, int type);

struct canopen_node
{
	uint8_t nodeid;
	uint8_t state;
	atlantronic_canopen_callback callback;
};

struct atlantronic_canopen
{
	struct canopen_node* node[CANOPEN_MAX_NODE];
	qemu_irq* irq_id;
	qemu_irq* irq_size;
	qemu_irq* irq_data_l;
	qemu_irq* irq_data_h;
};

void atlantronic_canopen_init(struct atlantronic_canopen* s, qemu_irq* irq_id, qemu_irq* irq_size, qemu_irq* irq_data_l, qemu_irq* irq_data_h);

void atlantronic_canopen_write_bus(struct atlantronic_canopen* s, struct can_msg msg);

void atlantronic_canopen_tx(struct atlantronic_canopen* s, struct can_msg msg);

int atlantronic_canopen_register_node(struct atlantronic_canopen* s, uint8_t nodeid, struct canopen_node* node, atlantronic_canopen_callback callback);

#endif
