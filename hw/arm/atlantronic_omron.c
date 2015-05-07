#include <stdlib.h>
#include <math.h>
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "kernel/rcc.h"
#include "atlantronic_omron.h"

int atlantronic_omron_init(struct atlantronic_omron *s, qemu_irq* irq_io, struct atlantronic_vect3 pos_omron, float range)
{
	if(irq_io == NULL)
	{
		return -1;
	}

	s->pos_omron = pos_omron;
	s->irq_io = irq_io;
	s->range = range;

	return 0;
}

int atlantronic_omron_update(struct atlantronic_omron *s, struct atlantronic_vect3 pos_robot)
{
	int j = 0;
	int res = 0;
	float dist_min = 0;

	struct atlantronic_vect3 omron_pos_abs;

	atlantronic_vect3_loc_to_abs(&pos_robot, &s->pos_omron, &omron_pos_abs);

	struct atlantronic_vect2 a = { omron_pos_abs.x, omron_pos_abs.y};
	struct atlantronic_vect2 b;
	struct atlantronic_vect2 h;


	b.x = a.x + s->range * cos(omron_pos_abs.theta);
	b.y = a.y + s->range * sin(omron_pos_abs.theta);
	dist_min = s->range;

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

	if( dist_min < s->range )
	{
		qemu_set_irq(*s->irq_io, 0);
	}
	else
	{
		qemu_set_irq(*s->irq_io, 1);
	}

	return 0;
}
