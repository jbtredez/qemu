#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "atlantronic_gpio.h"

enum gpio_mode
{
	GPIO_MODE_IN   = 0x00,
	GPIO_MODE_OUT  = 0x01,
	GPIO_MODE_AF   = 0x02,
	GPIO_MODE_AN   = 0x03
};

struct atlantronic_gpio_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	GPIO_TypeDef gpio;
	qemu_irq irq[GPIO_IRQ_OUT_NUM];
};

static void atlantronic_gpio_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_gpio_state* s = opaque;
	int32_t diff = 0;
	int i = 0;

	switch(offset)
	{
		W_ACCESS(GPIO_TypeDef, s->gpio, MODER, val);
		W_ACCESS(GPIO_TypeDef, s->gpio, OTYPER, val);
		W_ACCESS(GPIO_TypeDef, s->gpio, OSPEEDR, val);
		W_ACCESS(GPIO_TypeDef, s->gpio, PUPDR, val);
		case offsetof(GPIO_TypeDef, IDR):
			printf("Error : GPIO forbiden write acces IDR (offset %lx), val %lx\n", offset, val);
			break;
		case offsetof(GPIO_TypeDef, ODR):
			val &= 0xFFFF;
			diff = s->gpio.ODR ^ val;
			s->gpio.ODR = val;
			for( i = 0; i < PIN_NUM; i++)
			{
				int mode = (s->gpio.MODER >> (2 * i)) & 0x03;

				if( mode != GPIO_MODE_IN && ((diff >> i) & 0x01))
				{
					qemu_set_irq(s->irq[i], (val >> i) & 0x01);
				}
			}
			break;
		case offsetof(GPIO_TypeDef, BSRRL):
			// atomic bit set
			s->gpio.ODR |= val & 0xff;
			break;
		case offsetof(GPIO_TypeDef, BSRRH):
			// atomic bit reset
			s->gpio.ODR &= ~(val & 0xff);
			break;
		W_ACCESS(GPIO_TypeDef, s->gpio, LCKR, val);
		W_ACCESS(GPIO_TypeDef, s->gpio, AFR[0], val);
		W_ACCESS(GPIO_TypeDef, s->gpio, AFR[1], val);
		default:
			printf("Error : GPIO forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_gpio_state* s = opaque;

	switch(offset)
	{
		R_ACCESS(GPIO_TypeDef, s->gpio, MODER, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, OTYPER, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, OSPEEDR, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, PUPDR, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, IDR, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, ODR, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, BSRRL, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, BSRRH, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, LCKR, res);
		R_ACCESS(GPIO_TypeDef, s->gpio, AFR[0], res);
		R_ACCESS(GPIO_TypeDef, s->gpio, AFR[1], res);
		default:
			printf("Error : GPIO forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_gpio_reset(GPIO_TypeDef* gpio)
{
	gpio->MODER = 0x00;
	gpio->OTYPER = 0x00;
	gpio->OSPEEDR = 0x00;
	gpio->PUPDR = 0x00;
	gpio->IDR = 0x00;
	gpio->ODR = 0x00;
	gpio->BSRRL = 0x00;
	gpio->BSRRH = 0x00;
	gpio->LCKR = 0x00;
	gpio->AFR[0] = 0x00;
	gpio->AFR[1] = 0x00;
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

	int mode = (s->gpio.MODER >> (2 * numPin)) & 0x03;
	uint32_t oldLevel = s->gpio.IDR;
	numPin &= 0x0f;

	switch(mode)
	{
		default:
		case GPIO_MODE_IN:
			// io 0 / 1
			if(level)
			{
				level = 1;
				s->gpio.IDR |= (1 << numPin);
			}
			else
			{
				s->gpio.IDR &= ~(1 << numPin);
			}
			if( oldLevel != s->gpio.IDR )
			{
				qemu_set_irq(s->irq[GPIO_IRQ_OUT_CHANGE_PIN], numPin + (level << 4));
			}
			break;
		case GPIO_MODE_OUT:
			// pin configurée en sortie, erreur
			hw_error("conflict, try to set gpio output pin %d lv %d", numPin, level);
			break;
		case GPIO_MODE_AF:
			// pin configuree en alternate function,  erreur
			hw_error("conflict, try to set gpio AF pin %d lv %d", numPin, level);
			break;
		case GPIO_MODE_AN:
			// AN, envoi vers le module adc
			qemu_set_irq(s->irq[numPin], level);
			break;
	}
}

static int atlantronic_gpio_init(SysBusDevice * dev)
{
	struct atlantronic_gpio_state *s = OBJECT_CHECK(struct atlantronic_gpio_state, dev, "atlantronic-gpio");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_gpio_ops, s, "atlantronic_gpio", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, GPIO_IRQ_OUT_NUM);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_gpio_in_recv, PIN_NUM);

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
