#ifndef ATLANTRONIC_OMRON_H
#define ATLANTRONIC_OMRON_H

#include "hw/irq.h"
#include "atlantronic_tools.h"

struct atlantronic_omron
{
	qemu_irq* irq_io;
	float range;
	struct atlantronic_vect3 pos_omron;   //!< position du omron dans le repere robot
	int object_flags;
	int inverted;                            //!< inversion du signal 0 / 1
};

int atlantronic_omron_init(struct atlantronic_omron *s, qemu_irq* irq_io, struct atlantronic_vect3 pos_omron, float range, int object_flags, int inverted);

int atlantronic_omron_update(struct atlantronic_omron *s, struct atlantronic_vect3 pos_robot);

#endif
