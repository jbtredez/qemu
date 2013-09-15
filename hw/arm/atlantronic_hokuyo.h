#ifndef ATLANTRONIC_HOKUYO_H
#define ATLANTRONIC_HOKUYO_H

#define HOKUYO_MAX_MES                 769

#include "hw/irq.h"
#include "atlantronic_tools.h"

struct atlantronic_hokuyo_state
{
	qemu_irq* irq_tx;
	QEMUTimer* timer;
	unsigned char rx_buffer[32];
	unsigned int rx_size;
	unsigned char tx_buffer[4096];
	unsigned int tx_size;
	double mes[HOKUYO_MAX_MES];
	int scan_ready;
	int send_scan_when_ready;
	uint64_t timer_count;
	struct atlantronic_vect3 pos_robot;   //!< poosition du robot
	struct atlantronic_vect3 pos_hokuyo;  //!< position du hokuyo dans le repere robot
	int clock_scale;
};

void atlantronic_hokuyo_in_recv_usart(struct atlantronic_hokuyo_state *s, unsigned char data);

int atlantronic_hokuyo_init(struct atlantronic_hokuyo_state *s, qemu_irq* irq_tx, struct atlantronic_vect3 pos_hokuyo);

enum
{
	HOKUYO_IRQ_IN_USART_DATA,
	HOKUYO_IRQ_IN_X,
	HOKUYO_IRQ_IN_Y,
	HOKUYO_IRQ_IN_ALPHA,
	HOKUYO_IRQ_IN_NUM
};

#endif
