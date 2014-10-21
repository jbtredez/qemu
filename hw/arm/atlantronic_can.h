#ifndef ATLANTRONIC_CAN_H
#define ATLANTRONIC_CAN_H

#include <stdint.h>

enum
{
	CAN_IRQ_OUT_TX0,
	CAN_IRQ_OUT_RX0,
	CAN_IRQ_OUT_CAN1_MSG_ID,
	CAN_IRQ_OUT_CAN1_MSG_SIZE,
	CAN_IRQ_OUT_CAN1_MSG_DATA_L,
	CAN_IRQ_OUT_CAN1_MSG_DATA_H,
	CAN_IRQ_OUT_MAX,
};

enum
{
	CAN_IRQ_IN_CAN1_MSG_ID,
	CAN_IRQ_IN_CAN1_MSG_SIZE,
	CAN_IRQ_IN_CAN1_MSG_DATA_L,
	CAN_IRQ_IN_CAN1_MSG_DATA_H,
	CAN_IRQ_IN_MAX,
};

struct can_msg
{
	uint32_t id; //!< 11 bits ou 29 bits si étendue
	// accès aux données par tableau ou par 2 mots de 32 bits (pour le driver)
	union
	{
		uint8_t data[8]; //!< données (de 0 à  8 octets)
		struct {
			uint32_t low;
			uint32_t high;
		} _data;
	};
	unsigned char size; //!< taille
	unsigned char format; //!< format (standard ou étendu)
	unsigned char type; //!< type
} __attribute__((packed));

#endif
