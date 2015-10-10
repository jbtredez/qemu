#include <stdlib.h>
#include <math.h>
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "kernel/rcc.h"
#include "atlantronic_hokuyo.h"

#define HOKUYO_MED                     384
#define HOKUYO_MAX_DISTANCE           4000
#define HOKUYO_PERIOD_TICK             100

#define HOKUYO_RES              (M_PI/512)
#define HOKUYO_ERROR                    20

static void atlantronic_hokuyo_send_buffer(struct atlantronic_hokuyo_state* s)
{
	int i = 0;
	for(i = 0; i < s->tx_size; i++)
	{
		qemu_set_irq(*s->irq_tx, s->tx_buffer[i]);
	}
}

static void atlantronic_hokuyo_encode16(unsigned char* data, uint16_t val)
{
	data[1] = (val & 0x3f) + 0x30;
	data[0] = ((val >> 6) & 0x3f) + 0x30;
}

// mise a jour des mesures
static void atlantronic_hokuyo_update(struct atlantronic_hokuyo_state* s)
{
	int i = 0;
	int j = 0;
	int res = 0;
	float dist_min = 0;

	struct atlantronic_vect3 hokuyo_pos_abs;

	atlantronic_vect3_loc_to_abs(&s->pos_robot, &s->pos_hokuyo, &hokuyo_pos_abs);

	struct atlantronic_vect2 a = { hokuyo_pos_abs.x, hokuyo_pos_abs.y};
	struct atlantronic_vect2 b;
	struct atlantronic_vect2 h;

	for(i = 0; i < HOKUYO_MAX_MES; i++)
	{
		b.x = a.x + HOKUYO_MAX_DISTANCE * cos(hokuyo_pos_abs.theta + (i - HOKUYO_MED) * HOKUYO_RES);
		b.y = a.y + HOKUYO_MAX_DISTANCE * sin(hokuyo_pos_abs.theta + (i - HOKUYO_MED) * HOKUYO_RES);
		dist_min = HOKUYO_MAX_DISTANCE;

		for(j = 0; j < atlantronic_static_obj_count; j++)
		{
			// on regarde uniquement les objets visible par le hokuyo
			if( atlantronic_static_obj[j].flags & OBJECT_SEEN_BY_HOKUYO )
			{
				int k = 0;
				for(k = 0; k < atlantronic_static_obj[j].polyline.size - 1; k++)
				{
					res = atlantronic_segment_intersection(a, b, atlantronic_static_obj[j].polyline.pt[k], atlantronic_static_obj[j].polyline.pt[k+1], &h);
					if( ! res )
					{
						float dx = h.x - a.x;
						float dy = h.y - a.y;
						float dist = sqrtf(dx * dx + dy * dy);
						if( dist < dist_min )
						{
							dist_min = dist;
						}
					}
				}
			}
		}

		s->mes[i] = dist_min + HOKUYO_ERROR * (2*(rand() / (RAND_MAX + 1.0)) - 1);
	}
}

static void atlantronic_hokuyo_gs(struct atlantronic_hokuyo_state* s)
{
	int start;
	int end;
	int cc;
	int i;

	atlantronic_hokuyo_update(s);

	sscanf((char*)s->rx_buffer + 2, "%4d%4d%2d", &start, &end, &cc);
	// message GSxxxxyyyycc\n : reponse GSxxxxyyyycc\n00P\n000P\n + data
	memcpy(s->tx_buffer, s->rx_buffer, s->rx_size);
	s->tx_size = s->rx_size;
	s->tx_size += sprintf((char*)(s->tx_buffer + s->rx_size), "00P\n0000P\n");
	unsigned char* buf = s->tx_buffer + s->tx_size;
	int size = 0;
	uint8_t sum = 0;

	for(i = start; i <= end; i++)
	{
		atlantronic_hokuyo_encode16(buf, s->mes[i]);
		sum += buf[0] + buf[1];
		buf += 2;
		size += 2;
		if(size == 64)
		{
			sum &= 0x3f;
			sum += 0x30;
			buf[0] = sum;
			buf[1] = '\n';
			buf += 2;
			size = 0;
			sum = 0;
		}
	}

	if( size > 0)
	{
		size = 0;
		sum &= 0x3f;
		sum += 0x30;
		buf[0] = sum;
		buf[1] = '\n';
		buf += 2;
	}
	buf[0] = '\n';
	buf++;

	s->tx_size = buf - s->tx_buffer;
	atlantronic_hokuyo_send_buffer(s);
	s->rx_size = 0;
}

static void atlantronic_hokuyo_systick_cb(void* arg)
{
	struct atlantronic_hokuyo_state *s = arg;

	if( s->systick_count == 0 )
	{
		// tout les 100ms
		if( s->send_scan_when_ready )
		{
			s->scan_ready = 0;
			s->send_scan_when_ready = 0;
			atlantronic_hokuyo_gs(s);
		}
		else
		{
			s->scan_ready = 1;
		}
	}
	s->systick_count = (s->systick_count + 1) % HOKUYO_PERIOD_TICK;
}

void atlantronic_hokuyo_in_recv_usart(struct atlantronic_hokuyo_state *s, unsigned char data)
{
	if( s->send_scan_when_ready )
	{
		// hokuyo occupe, en train de faire un scan
		return;
	}

	s->rx_buffer[s->rx_size] = data;
	s->rx_size++;

	if( data == '\n' )
	{
		// interpretation du message le message
		if(strncmp((const char*)s->rx_buffer, "SCIP2.0\n", s->rx_size) == 0)
		{
			// message SCIP2.0\n : reponse SCIP2.0\n00P\n\n
			s->tx_size = sprintf((char*)s->tx_buffer, "SCIP2.0\n00P\n\n");
			atlantronic_hokuyo_send_buffer(s);
			s->rx_size = 0;
		}
		else if(strncmp((const char*)s->rx_buffer, "BM", 2) == 0)
		{
			// message BMxxx\n : reponse BMxxx\n00P\n\n
			memcpy(s->tx_buffer, s->rx_buffer, s->rx_size);
			s->tx_size = s->rx_size;
			s->tx_size += sprintf((char*)(s->tx_buffer + s->rx_size), "00P\n\n");
			atlantronic_hokuyo_send_buffer(s);
			s->rx_size = 0;
		}
		else if(strncmp((const char*)s->rx_buffer, "HS", 2) == 0)
		{
			// message HSxxx\n : reponse HSxxx\n00P\n\n
			memcpy(s->tx_buffer, s->rx_buffer, s->rx_size);
			s->tx_size = s->rx_size;
			s->tx_size += sprintf((char*)(s->tx_buffer + s->rx_size), "00P\n\n");
			atlantronic_hokuyo_send_buffer(s);
			s->rx_size = 0;
		}
		else if(strncmp((const char*)s->rx_buffer, "GS", 2) == 0)
		{
			if( s->scan_ready )
			{
				s->scan_ready = 0;
				s->send_scan_when_ready = 0;
				atlantronic_hokuyo_gs(s);
			}
			else
			{
				s->send_scan_when_ready = 1;
			}
		}
		else
		{
			printf("hokuyo : unknown command : %.*s", s->rx_size, s->rx_buffer);
			s->rx_size = 0;
		}
	}

	s->rx_size = s->rx_size % sizeof(s->rx_buffer);
}

int atlantronic_hokuyo_init(struct atlantronic_hokuyo_state *s, qemu_irq* irq_tx, struct atlantronic_vect3 pos_hokuyo)
{
	if(irq_tx == NULL)
	{
		return -1;
	}

	s->pos_hokuyo = pos_hokuyo;
	s->irq_tx = irq_tx;
	s->scan_ready = 0;
	s->send_scan_when_ready = 0;
	s->rx_size = 0;
	s->systick_count = 0;
	systick_add_cb(atlantronic_hokuyo_systick_cb, s);
	return 0;
}
