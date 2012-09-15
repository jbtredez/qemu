#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"
#include "exec-memory.h"

#define DMA_CHAN_NUMBER                7

struct atlantronic_dma_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	DMA_TypeDef dma;
	qemu_irq irq[DMA_CHAN_NUMBER];
};

static void atlantronic_dma_write(void *opaque, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	struct atlantronic_dma_state* s = opaque;
	int i = 0;

	switch(offset)
	{
		case offsetof(DMA_TypeDef, IFCR):
			for(i = 0; i < DMA_CHAN_NUMBER; i++)
			{
				int32_t v = val & 0x0f;
				if( v & 0x1)
				{
					// clear complet des flags du chan i
					s->dma.ISR &= ~(0x0f << (4 * i) );
					qemu_set_irq(s->irq[i], 0);
				}
				else
				{
					s->dma.ISR &= ~((v & 0x0f) << (4 * i) );
					if( v & 0x02 )
					{
						qemu_set_irq(s->irq[i], 0);
					}
				}
				val >>= 4;
			}
			break;
		case offsetof(DMA_TypeDef, ISR):
		default:
			printf("Error : DMA forbiden write acces offset %x, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_dma_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_dma_state* s = opaque;

	switch(offset)
	{
		case offsetof(DMA_TypeDef, ISR):
			res = s->dma.ISR;
			break;
		case offsetof(DMA_TypeDef, IFCR):
			res = s->dma.IFCR;
			break;
		default:
			printf("Error : DMA forbiden read acces offset %x\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_dma_reset(DMA_TypeDef* dma)
{
	dma->ISR = 0x00;
	dma->IFCR = 0x00;
}

static const MemoryRegionOps atlantronic_dma_ops =
{
	.read = atlantronic_dma_read,
	.write = atlantronic_dma_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_dma_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_dma_state *s = opaque;

	level &= 0x0f;
	uint32_t old_val = (s->dma.ISR >> (4 * numPin)) & 0x0f;
	s->dma.ISR |= level << (4 * numPin);

	uint32_t diff_raise = (old_val ^ level) & level;
	if( diff_raise & 0x02 )
	{
		qemu_set_irq(s->irq[numPin], 1);
	}

	// TODO voir / autres irq (erreur et half transfert)
}

static int atlantronic_dma_init(SysBusDevice * dev)
{
	struct atlantronic_dma_state *s = FROM_SYSBUS(struct atlantronic_dma_state, dev);

	memory_region_init_io(&s->iomem, &atlantronic_dma_ops, s, "atlantronic_dma", 0x08);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(&dev->qdev, s->irq, DMA_CHAN_NUMBER);
	qdev_init_gpio_in(&dev->qdev, atlantronic_dma_in_recv, DMA_CHAN_NUMBER);

	atlantronic_dma_reset(&s->dma);

    return 0;
}

static void atlantronic_dma_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_dma_init;
}

static TypeInfo atlantronic_dma_info =
{
	.name          = "atlantronic-dma",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_dma_state),
	.class_init    = atlantronic_dma_class_init,
};

static void atlantronic_dma_register_types(void)
{
	type_register_static(&atlantronic_dma_info);
}

type_init(atlantronic_dma_register_types);
