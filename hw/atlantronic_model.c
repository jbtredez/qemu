#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/otgd_fs_regs.h"

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"
#include "exec-memory.h"

#define IRQ_OUT_NUM        2
#define IRQ_IN_NUM         6
#define PWM_NUM            4
#define ENCODER_NUM        2

struct atlantronic_model_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	qemu_irq irq[IRQ_OUT_NUM];
	int32_t pwm[PWM_NUM];
	int32_t enc[ENCODER_NUM];
};

static void atlantronic_model_reset(struct atlantronic_model_state* s)
{
	int i = 0;
	for(i = 0; i < PWM_NUM; i++)
	{
		s->pwm[i] = 0;
	}

	for(i = 0; i < ENCODER_NUM; i++)
	{
		s->enc[i] = 0;
	}
}

static void atlantronic_model_in_recv(void * opaque, int numPin, int level)
{
    struct atlantronic_model_state *s = opaque;

	if(numPin < ENCODER_NUM)
	{
		s->enc[numPin] = level;
	}
	else if(numPin < PWM_NUM + ENCODER_NUM)
	{
		s->pwm[numPin - ENCODER_NUM] = level;
	}

	// TODO méthode de déclenchement du calcul du modele ?
}

static int atlantronic_model_init(SysBusDevice * dev)
{
    struct atlantronic_model_state *s = FROM_SYSBUS(struct atlantronic_model_state, dev);

	// memory_region_init_ram_ptr
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(&dev->qdev, s->irq, IRQ_OUT_NUM);
	qdev_init_gpio_in(&dev->qdev, atlantronic_model_in_recv, IRQ_IN_NUM);

	atlantronic_model_reset(s);

    return 0;
}

static void atlantronic_model_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_model_init;
}

static TypeInfo atlantronic_model_info =
{
	.name          = "atlantronic-model",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_model_state),
	.class_init    = atlantronic_model_class_init,
};

static void atlantronic_model_register_types(void)
{
	type_register_static(&atlantronic_model_info);
}

type_init(atlantronic_model_register_types);
