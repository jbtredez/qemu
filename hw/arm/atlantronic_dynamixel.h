#ifndef ATLANTRONIC_DYNAMIXEL_H
#define ATLANTRONIC_DYNAMIXEL_H


struct atlantronic_dynamixel_state
{
	qemu_irq* irq_tx;
	unsigned char rx_buffer[1024];
	unsigned int rx_size;
	unsigned char tx_buffer[1024];
	unsigned int tx_size;
	float theta;
	int disconnected; //!< deconnexion du bus (defaillance)
	unsigned char control_table[50]; //!< table de controle, donnes en eeprom et en ram
};

enum atlantronic_dynamixel_type
{
	DYNAMIXEL_AX12 = 12,
	DYNAMIXEL_RX24F = 24,
};

int atlantronic_dynamixel_init(struct atlantronic_dynamixel_state *s, qemu_irq* irq_tx, unsigned char id, enum atlantronic_dynamixel_type type);

void atlantronic_dynamixel_in_recv_usart(struct atlantronic_dynamixel_state *s, unsigned char data);

#endif
