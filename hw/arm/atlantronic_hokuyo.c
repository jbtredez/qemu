#include <stdlib.h>
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"

#define LINUX
#define STM32F10X_CL
#undef FALSE
#undef TRUE
#undef bool
#include "kernel/cpu/cpu.h"
#undef LINUX

#include "kernel/robot_parameters.h"
#include "atlantronic_hokuyo.h"
#include "atlantronic_tools.h"

#define HOKUYO_MAX_MES                 769
#define HOKUYO_MED                     384
#define HOKUYO_PERIOD_TICK         7200000
#define HOKUYO_MAX_DISTANCE           4000

#define HOKUYO_RES              (M_PI/512)
#define HOKUYO_ERROR                    20

struct atlantronic_hokuyo_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	qemu_irq irq;
	QEMUTimer* timer;
	unsigned char rx_buffer[32];
	unsigned int rx_size;
	unsigned char tx_buffer[4096];
	unsigned int tx_size;
	double mes[HOKUYO_MAX_MES];
	int scan_ready;
	int send_scan_when_ready;
	uint64_t timer_count;
	float x;
	float y;
	float alpha;
};

static void atlantronic_hokuyo_send_buffer(struct atlantronic_hokuyo_state* s)
{
	int i = 0;
	for(i = 0; i < s->tx_size; i++)
	{
		qemu_set_irq(s->irq, s->tx_buffer[i]);
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

	struct atlantronic_vect3 pos_robot = { s->x, s->y, s->alpha, cos(s->alpha), sin(s->alpha)};
	struct atlantronic_vect3 hokuyo_pos_loc;
	hokuyo_pos_loc.x =  PARAM_FOO_HOKUYO_X / 65536.0f;
	hokuyo_pos_loc.y = PARAM_FOO_HOKUYO_Y / 65536.0f;
	hokuyo_pos_loc.alpha = PARAM_FOO_HOKUYO_ALPHA * 2 * M_PI / (1<<26);
	hokuyo_pos_loc.ca = cos(hokuyo_pos_loc.alpha);
	hokuyo_pos_loc.sa = sin(hokuyo_pos_loc.alpha);
	struct atlantronic_vect3 hokuyo_pos_abs;

	atlantronic_vect3_loc_to_abs(&pos_robot, &hokuyo_pos_loc, &hokuyo_pos_abs);

	struct atlantronic_vect2 a = { hokuyo_pos_abs.x, hokuyo_pos_abs.y};
	struct atlantronic_vect2 b;
	struct atlantronic_vect2 h;

	for(i = 0; i < HOKUYO_MAX_MES; i++)
	{
		b.x = a.x + HOKUYO_MAX_DISTANCE * cos(hokuyo_pos_abs.alpha + (i - HOKUYO_MED) * HOKUYO_RES);
		b.y = a.y + HOKUYO_MAX_DISTANCE * sin(hokuyo_pos_abs.alpha + (i - HOKUYO_MED) * HOKUYO_RES);
		dist_min = HOKUYO_MAX_DISTANCE;

		for(j = 0; j < atlantronic_static_obj_count; j++)
		{
			int k = 0;
			for(k = 0; k < atlantronic_static_obj[j].size - 1; k++)
			{
				res = atlantronic_segment_intersection(a, b, atlantronic_static_obj[j].pt[k], atlantronic_static_obj[j].pt[k+1], &h);
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

static void atlantronic_timer_cb(void* arg)
{
	static int clock_scale = 1;
	struct atlantronic_hokuyo_state *s = arg;

	s->timer_count += HOKUYO_PERIOD_TICK;
	qemu_mod_timer(s->timer, s->timer_count);
	if( clock_scale >= system_clock_scale)
	{
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
		clock_scale = 0;
	}
	clock_scale++;
}

static void atlantronic_hokuyo_in_recv_usart(struct atlantronic_hokuyo_state *s, int level)
{
	if( s->send_scan_when_ready )
	{
		// hokuyo occupe, en train de faire un scan
		return;
	}

	level &= 0xff;

	s->rx_buffer[s->rx_size] = level;
	s->rx_size++;

	if( level == '\n' )
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
			if(s->timer_count == 0)
			{
				s->timer_count = qemu_get_clock_ns(vm_clock) + HOKUYO_PERIOD_TICK;
				qemu_mod_timer(s->timer, s->timer_count);
			}
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

static void atlantronic_hokuyo_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_hokuyo_state *s = opaque;

	switch(numPin)
	{
		case HOKUYO_IRQ_IN_USART_DATA:
			atlantronic_hokuyo_in_recv_usart(s, level);
			break;
		case HOKUYO_IRQ_IN_X:
			s->x = level / 65536.0f;
			break;
		case HOKUYO_IRQ_IN_Y:
			s->y = level / 65536.0f;
			break;
		case HOKUYO_IRQ_IN_ALPHA:
			s->alpha = level * 2 * M_PI / (1<<26);
			break;
	}
}

static int atlantronic_hokuyo_init(SysBusDevice * dev)
{
	struct atlantronic_hokuyo_state *s = OBJECT_CHECK(struct atlantronic_hokuyo_state, dev, "atlantronic-hokuyo");

	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), &s->irq, 1);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_hokuyo_in_recv, HOKUYO_IRQ_IN_NUM);

	s->timer_count = 0;
	s->timer = qemu_new_timer(vm_clock, 1, atlantronic_timer_cb, s);
	s->scan_ready = 0;
	s->send_scan_when_ready = 0;
	s->rx_size = 0;

    return 0;
}

static void atlantronic_hokuyo_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_hokuyo_init;
}

static TypeInfo atlantronic_hokuyo_info =
{
	.name          = "atlantronic-hokuyo",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_hokuyo_state),
	.class_init    = atlantronic_hokuyo_class_init,
};

static void atlantronic_hokuyo_register_types(void)
{
	type_register_static(&atlantronic_hokuyo_info);
}

type_init(atlantronic_hokuyo_register_types);
