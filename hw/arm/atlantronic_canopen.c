#include "atlantronic_canopen.h"
#include <stdio.h>

#define CANOPEN_MAX_NODE          128

#define CANOPEN_RX_PDO1          0x180
#define CANOPEN_RX_PDO2          0x280
#define CANOPEN_RX_PDO3          0x380
#define CANOPEN_SDO_RES          0x580
#define CANOPEN_BOOTUP           0x700

enum
{
	NMT_OPERATIONAL        =  0x01,
	NMT_STOP               =  0x02,
	NMT_PRE_OPERATIONAL    =  0x80,
	NMT_RESET_AP           =  0x81,
	NMT_RESET_COM          =  0x82,
};

struct canopen_node
{
	uint8_t nodeid;
	uint8_t state;
	void* opaque;
	void* can_interface;
	atlantronic_canopen_callback callback;
};

struct canopen_node atlantronic_canopen_node[128];
static void atlantronic_canopen_set_nmt(void* can_interface, int nodeid, uint8_t new_state);
static void init_canopen(void) __attribute__((constructor));

static void init_canopen(void)
{
	int i = 0;
	for(i = 0; i < CANOPEN_MAX_NODE; i++)
	{
		atlantronic_canopen_node[i].nodeid = 0;
	}
}

void atlantronic_canopen_tx(void* can_interface, struct can_msg msg)
{
	unsigned int nodeid = 0;
	int type = 0;
	int i;

#if 1
	// debug
	char buffer[1024];
	int res = snprintf(buffer, sizeof(buffer), "\r[qemu] CAN msg id %d size %d data ", msg.id, msg.size);
	for(i=0; i < msg.size && res > 0; i++)
	{
		res += snprintf(buffer + res, sizeof(buffer) - res, " %2.2x", msg.data[i]);
	}
	printf("%s\n", buffer);
#endif

	if( msg.id > 0x380 && msg.id < 0x400)
	{
		// PDO 3
		nodeid = msg.id - 0x380;
		type = CANOPEN_RX_PDO3;
	}
	else if( msg.id > 0x280 && msg.id < 0x300)
	{
		// PDO 2
		nodeid = msg.id - 0x280;
		type = CANOPEN_RX_PDO2;
	}
	else if( msg.id > 0x180 && msg.id < 0x200)
	{
		// PDO 1
		nodeid = msg.id - 0x180;
		type = CANOPEN_RX_PDO1;
	}
	else if(msg.id > 0x580 && msg.id < 0x600)
	{
		// SDO
		nodeid = msg.id - 0x580;
		type = CANOPEN_SDO_RES;
	}
	else if(msg.id > 0x700 && msg.id < 0x780)
	{
		// bootup
		nodeid = msg.id - 0x700;
		type = CANOPEN_BOOTUP;
	}
	else if( msg.id == 0)
	{
		// NMT
		int new_state = msg.data[0];
		// on regarde si c'est un etat valide
		if( new_state == NMT_OPERATIONAL ||
			new_state == NMT_STOP ||
			new_state == NMT_PRE_OPERATIONAL ||
			new_state == NMT_RESET_AP ||
			new_state == NMT_RESET_COM )
		{
			nodeid = msg.data[1];
			if( nodeid == 0)
			{
				// tout les noeuds
				for(i = 1; i < CANOPEN_MAX_NODE; i++)
				{
					atlantronic_canopen_set_nmt(can_interface, i, new_state);
				}
			}
			else
			{
				atlantronic_canopen_set_nmt(can_interface, nodeid, new_state);
			}
		}

		return;
	}

	if(nodeid > 0 && nodeid < CANOPEN_MAX_NODE)
	{
		if(atlantronic_canopen_node[nodeid].nodeid == nodeid)
		{
			// noeud parametre
			atlantronic_canopen_node[nodeid].callback(atlantronic_canopen_node[nodeid].opaque, msg, type);
		}
	}
}

void atlantronic_canopen_set_nmt(void* can_interface, int nodeid, uint8_t new_state)
{
	if(nodeid <= 0 || nodeid >= CANOPEN_MAX_NODE)
	{
		return;
	}

	// noeud parametre
	if(atlantronic_canopen_node[nodeid].nodeid == nodeid)
	{
		atlantronic_canopen_node[nodeid].state = new_state;
		if( new_state == NMT_RESET_COM || new_state == NMT_RESET_AP)
		{
			atlantronic_canopen_node[nodeid].can_interface = can_interface;
			// envoi du bootup
			printf("nodeid base %x id %x\n", nodeid, atlantronic_canopen_node[nodeid].nodeid);
			struct can_msg msg;
			msg.id = 0x700 + (uint32_t)nodeid;
			msg.size = 0;
			atlantronic_can_rx(can_interface, msg);
		}
	}
}

int atlantronic_canopen_register_node(uint8_t nodeid, void* opaque, atlantronic_canopen_callback callback)
{
	int res = -1;

	if(nodeid > 0 && nodeid < CANOPEN_MAX_NODE)
	{
		atlantronic_canopen_node[nodeid].nodeid = nodeid;
		atlantronic_canopen_node[nodeid].state = NMT_RESET_COM;
		atlantronic_canopen_node[nodeid].opaque = opaque;
		atlantronic_canopen_node[nodeid].can_interface = NULL;
		atlantronic_canopen_node[nodeid].callback = callback;
		res = 0;
	}

	return res;
}
