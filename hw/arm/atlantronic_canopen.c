#include "atlantronic_canopen.h"
#include <stdio.h>


enum
{
	NMT_OPERATIONAL        =  0x01,
	NMT_STOP               =  0x02,
	NMT_PRE_OPERATIONAL    =  0x80,
	NMT_RESET_AP           =  0x81,
	NMT_RESET_COM          =  0x82,
};

static void atlantronic_canopen_set_nmt(struct atlantronic_canopen* s, int nodeid, uint8_t new_state);

void atlantronic_canopen_init(struct atlantronic_canopen* s, qemu_irq* irq_id, qemu_irq* irq_size, qemu_irq* irq_data_l, qemu_irq* irq_data_h)
{
	int i = 0;
	for(i = 0; i < CANOPEN_MAX_NODE; i++)
	{
		s->node[i] = NULL;
	}
	s->irq_id = irq_id;
	s->irq_size = irq_size;
	s->irq_data_l = irq_data_l;
	s->irq_data_h = irq_data_h;
}

void atlantronic_canopen_write_bus(struct atlantronic_canopen* s, struct can_msg msg)
{
	qemu_set_irq(*s->irq_data_l, msg._data.low);
	qemu_set_irq(*s->irq_data_h, msg._data.high);
	qemu_set_irq(*s->irq_size, msg.size);
	qemu_set_irq(*s->irq_id, msg.id); // envoi de l'id en dernier, indique la fin du message
}

void atlantronic_canopen_tx(struct atlantronic_canopen* s, struct can_msg msg)
{
	unsigned int nodeid = 0;
	int type = 0;
	int i;

#if 0
	// debug
	char buffer[1024];
	int res = snprintf(buffer, sizeof(buffer), "\r[qemu] CAN msg id %x size %d data ", msg.id, msg.size);
	for(i=0; i < msg.size && res > 0; i++)
	{
		res += snprintf(buffer + res, sizeof(buffer) - res, " %2.2x", msg.data[i]);
	}
	printf("%s\n", buffer);
#endif

	type = msg.id >> 7;
	nodeid = msg.id & 0x7F;

	if( type == CANOPEN_NMT )
	{
		if( nodeid != 0)
		{
			// TODO erreur;
			return;
		}

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
					atlantronic_canopen_set_nmt(s, i, new_state);
					// appel de la callback pour les actions specifiques a faire
					if(s->node[i])
					{
						s->node[i]->callback(s, s->node[i], msg, type);
					}
				}
			}
			else
			{
				atlantronic_canopen_set_nmt(s, nodeid, new_state);
				// appel de la callback pour les actions specifiques a faire
				if(nodeid > 0 && nodeid < CANOPEN_MAX_NODE && s->node[nodeid])
				{
					s->node[nodeid]->callback(s, s->node[nodeid], msg, type);
				}
			}
		}
	}
	else if( type == CANOPEN_SYNC )
	{
		if( nodeid != 0)
		{
			// TODO erreur;
			return;
		}

		// sync sur tout les noeuds
		for(i = 1; i < CANOPEN_MAX_NODE; i++)
		{
			// appel de la callback pour les actions specifiques a faire
			if(s->node[i])
			{
				s->node[i]->callback(s, s->node[i], msg, type);
			}
		}
	}
	else
	{
		if(nodeid > 0)
		{
			if(s->node[nodeid])
			{
				// noeud parametre
				s->node[nodeid]->callback(s, s->node[nodeid], msg, type);
			}
		}
	}
}

void atlantronic_canopen_set_nmt(struct atlantronic_canopen* s, int nodeid, uint8_t new_state)
{
	if(nodeid <= 0 || nodeid >= CANOPEN_MAX_NODE)
	{
		return;
	}

	// noeud parametre
	if(s->node[nodeid])
	{
		s->node[nodeid]->state = new_state;
		if( new_state == NMT_RESET_COM || new_state == NMT_RESET_AP)
		{
			// envoi du bootup
			struct can_msg msg;
			msg.id = 0x700 + (uint32_t)nodeid;
			msg.size = 0;
			atlantronic_canopen_write_bus(s, msg);
		}
	}
}

int atlantronic_canopen_register_node(struct atlantronic_canopen* s, uint8_t nodeid, struct canopen_node* node, atlantronic_canopen_callback callback)
{
	int res = -1;

	if(nodeid > 0 && nodeid < CANOPEN_MAX_NODE)
	{
		s->node[nodeid] = node;
		s->node[nodeid]->nodeid = nodeid;
		s->node[nodeid]->state = NMT_RESET_COM;
		s->node[nodeid]->callback = callback;
		res = 0;
	}

	return res;
}
