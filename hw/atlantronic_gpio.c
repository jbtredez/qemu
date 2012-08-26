#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/otgd_fs_regs.h"

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"
#include "exec-memory.h"

#define PIN_NUM        8

struct atlantronic_gpio_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	GPIO_TypeDef gpio;
	qemu_irq irq[PIN_NUM];
};

static void atlantronic_gpio_write(void *opaque, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	struct atlantronic_gpio_state* s = opaque;

	switch(offset)
	{
		case offsetof(GPIO_TypeDef, CRL):
			s->gpio.CRL = val;
			break;
		case offsetof(GPIO_TypeDef, CRH):
			s->gpio.CRH = val;
			break;
		case offsetof(GPIO_TypeDef, IDR):
			printf("Error : GPIO forbiden write acces IDR (offset %x), val %lx\n", offset, val);
			break;
		case offsetof(GPIO_TypeDef, ODR):
			s->gpio.ODR = val & 0xFFFF;
			break;
		case offsetof(GPIO_TypeDef, BSRR):
			s->gpio.BSRR = val;
			break;
		case offsetof(GPIO_TypeDef, BRR):
			s->gpio.BRR = val & 0xFFFF;
			break;
		case offsetof(GPIO_TypeDef, LCKR):
			s->gpio.LCKR = val & 0x1FFFF;
			break;
		default:
			printf("Error : GPIO forbiden write acces offset %x, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_gpio_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_gpio_state* s = opaque;

	switch(offset)
	{
		case offsetof(GPIO_TypeDef, CRL):
			res = s->gpio.CRL;
			break;
		case offsetof(GPIO_TypeDef, CRH):
			res = s->gpio.CRH;
			break;
		case offsetof(GPIO_TypeDef, IDR):
			res = s->gpio.IDR;
			break;
		case offsetof(GPIO_TypeDef, ODR):
			res = s->gpio.ODR;
			break;
		case offsetof(GPIO_TypeDef, BSRR):
			printf("Error : GPIO BSRR forbiden read acces (offset %x)\n", offset);
			break;
		case offsetof(GPIO_TypeDef, BRR):
			printf("Error : GPIO BRR forbiden read acces (offset %x)\n", offset);
			break;
		case offsetof(GPIO_TypeDef, LCKR):
			res = s->gpio.LCKR;
			break;
		default:
			printf("Error : GPIO forbiden read acces offset %x\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_gpio_reset(GPIO_TypeDef* gpio)
{
	gpio->CRL  = 0x44444444;
	gpio->CRH  = 0x44444444;
	gpio->IDR  = 0x00;
	gpio->ODR  = 0x00;
	gpio->BSRR = 0x00;
	gpio->BRR  = 0x00;
	gpio->LCKR = 0x00;
}

static const MemoryRegionOps atlantronic_gpio_ops =
{
	.read = atlantronic_gpio_read,
	.write = atlantronic_gpio_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_gpio_in_recv(void * opaque, int numPin, int level)
{
    struct atlantronic_gpio_state *s = opaque;

	if(level)
	{
		s->gpio.IDR |= (1 << numPin);
	}
	else
	{
		s->gpio.IDR &= ~(1 << numPin);
	}
}

static int atlantronic_gpio_init(SysBusDevice * dev)
{
    struct atlantronic_gpio_state *s = FROM_SYSBUS(struct atlantronic_gpio_state, dev);

	memory_region_init_io(&s->iomem, &atlantronic_gpio_ops, s, "atlantronic_gpio", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(&dev->qdev, s->irq, PIN_NUM);
	qdev_init_gpio_in(&dev->qdev, atlantronic_gpio_in_recv, PIN_NUM);

	atlantronic_gpio_reset(&s->gpio);

    return 0;
}

static void atlantronic_gpio_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_gpio_init;
}

static TypeInfo atlantronic_gpio_info =
{
	.name          = "atlantronic-gpio",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_gpio_state),
	.class_init    = atlantronic_gpio_class_init,
};

static void atlantronic_gpio_register_types(void)
{
	type_register_static(&atlantronic_gpio_info);
}

type_init(atlantronic_gpio_register_types);