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

enum
{
	ATLANTRONIC_USART_IRQ_HW = 0,
	ATLANTRONIC_USART_IRQ_DMAR,
	ATLANTRONIC_USART_IRQ_DEVICE_TX,
	ATLANTRONIC_USART_IRQ_DEVICE_RX,
	ATLANTRONIC_USART_IRQ_MAX,
};

struct atlantronic_usart_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	USART_TypeDef usart;
	qemu_irq irq[ATLANTRONIC_USART_IRQ_MAX];
};

static void atlantronic_usart_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_usart_state* s = opaque;

	switch(offset)
	{
		case offsetof(USART_TypeDef, SR):
			s->usart.SR &= (val | 0xfffffc9f);
			break;
		case offsetof(USART_TypeDef, DR):
			// usart actif et transmission activée
			if( (s->usart.CR1 & USART_CR1_UE) && (s->usart.CR1 & USART_CR1_TE))
			{
				// IT device
				qemu_set_irq(s->irq[ATLANTRONIC_USART_IRQ_DEVICE_RX], val & 0x1ff);

				s->usart.SR |= USART_SR_TXE;
				if( s->usart.CR1 & USART_CR1_TXEIE)
				{
					// si TXEIE => it usart : on a traité l'octet
					qemu_set_irq(s->irq[ATLANTRONIC_USART_IRQ_HW], 1);
				}
			}
			break;
		case offsetof(USART_TypeDef, BRR):
			s->usart.BRR = val & 0xffff;
			break;
		case offsetof(USART_TypeDef, CR1):
			s->usart.CR1 = val & 0x3fff;
			break;
		case offsetof(USART_TypeDef, CR2):
			s->usart.CR2 = val & 0x7f7f;
			break;
		case offsetof(USART_TypeDef, CR3):
			s->usart.CR3 = val & 0x7ff;
			break;
		case offsetof(USART_TypeDef, GTPR):
			s->usart.GTPR = val & 0xffff;
			break;
		default:
			printf("Error : USART forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_usart_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_usart_state* s = opaque;

	switch(offset)
	{
		case offsetof(USART_TypeDef, SR):
			res = s->usart.SR;
			break;
		case offsetof(USART_TypeDef, DR):
			s->usart.SR &= ~USART_SR_RXNE;
			res = s->usart.DR;
			// IT device
			qemu_set_irq(s->irq[ATLANTRONIC_USART_IRQ_DEVICE_TX], 1);
			break;
		case offsetof(USART_TypeDef, BRR):
			res = s->usart.BRR;
			break;
		case offsetof(USART_TypeDef, CR1):
			res = s->usart.CR1;
			break;
		case offsetof(USART_TypeDef, CR2):
			res = s->usart.CR2;
			break;
		case offsetof(USART_TypeDef, CR3):
			res = s->usart.CR3;
			break;
		case offsetof(USART_TypeDef, GTPR):
			res = s->usart.GTPR;
			break;
		default:
			printf("Error : USART forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_usart_reset(USART_TypeDef* usart)
{
	usart->SR   = 0xC0;
	usart->DR   = 0x00;
	usart->BRR  = 0x00;
	usart->CR1  = 0x00;
	usart->CR2  = 0x00;
	usart->CR3  = 0x00;
	usart->GTPR = 0x00;
}

static const MemoryRegionOps atlantronic_usart_ops =
{
	.read = atlantronic_usart_read,
	.write = atlantronic_usart_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_usart_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_usart_state *s = opaque;

	// usart activé et reception activée
	if( (s->usart.CR1 & USART_CR1_UE) && (s->usart.CR1 & USART_CR1_RE) )
	{
		s->usart.DR = level & 0x1ff;
		s->usart.SR |= USART_SR_RXNE;

		if( s->usart.CR1 & USART_CR1_RXNEIE )
		{
			// it reception : un octet a lire
			qemu_set_irq(s->irq[ATLANTRONIC_USART_IRQ_HW], 1);
		}

		if( s->usart.CR3 & USART_CR3_DMAR )
		{
			// buffer dma de reception => it dmar : un octet a lire
			qemu_set_irq(s->irq[ATLANTRONIC_USART_IRQ_DMAR], 1);
		}
	}
}

static int atlantronic_usart_init(SysBusDevice * dev)
{
	struct atlantronic_usart_state *s = OBJECT_CHECK(struct atlantronic_usart_state, dev, "atlantronic-usart");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_usart_ops, s, "atlantronic_usart", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, ATLANTRONIC_USART_IRQ_MAX);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_usart_in_recv, 1);

	atlantronic_usart_reset(&s->usart);

    return 0;
}

static void atlantronic_usart_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_usart_init;
}

static TypeInfo atlantronic_usart_info =
{
	.name          = "atlantronic-usart",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_usart_state),
	.class_init    = atlantronic_usart_class_init,
};

static void atlantronic_usart_register_types(void)
{
	type_register_static(&atlantronic_usart_info);
}

type_init(atlantronic_usart_register_types);
