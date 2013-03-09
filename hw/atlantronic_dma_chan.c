#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"

struct atlantronic_dma_chan_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	MemoryRegion ram;
	DMA_Channel_TypeDef dma_chan;
	uint32_t cndtr_init;
	uint32_t cpar_init;
	uint32_t cmar_init;
	qemu_irq irq;
};

static void atlantronic_update_dma(struct atlantronic_dma_chan_state *s)
{
	if( ! s->dma_chan.CCR & 0x01 )
	{
		// dma desactive
		return;
	}

	if( ! s->dma_chan.CNDTR )
	{
		// il n'y a plus rien a copier
		return;
	}

	int dir = (s->dma_chan.CCR >> 4) & 0x01;
	int pinc = (s->dma_chan.CCR >> 6) & 0x01;
	int minc = (s->dma_chan.CCR >> 7) & 0x01;
	int psize = 1 << ((s->dma_chan.CCR >> 8) & 0x3);
	int msize = 1 << ((s->dma_chan.CCR >> 10) & 0x3);

	int64_t val;
	if( dir )
	{
		// sens mem -> perif
		cpu_physical_memory_read(s->dma_chan.CMAR, &val, psize);
		cpu_physical_memory_write(s->dma_chan.CPAR, &val, msize);
	}
	else
	{
		// sens perif -> mem
		cpu_physical_memory_read(s->dma_chan.CPAR, &val, psize);
		cpu_physical_memory_write(s->dma_chan.CMAR, &val, msize);
	}

	s->dma_chan.CPAR += psize * pinc;
	s->dma_chan.CMAR += msize * minc;
	s->dma_chan.CNDTR--;
	if( ! s->dma_chan.CNDTR )
	{
		s->dma_chan.CPAR = s->cpar_init;
		s->dma_chan.CMAR = s->cmar_init;
		// si circ : rechargement cndtr
		if( ((s->dma_chan.CCR >> 5) & 0x01 ) && ! dir )
		{
			s->dma_chan.CNDTR = s->cndtr_init;
		}

		// IRQ fin de transfert
		qemu_set_irq(s->irq, 2);
	}
}

static void atlantronic_dma_chan_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_dma_chan_state* s = opaque;

	switch(offset)
	{
		case offsetof(DMA_Channel_TypeDef, CCR):
			if( ! (s->dma_chan.CCR & 0x01) && (val & 0x01) )
			{
				s->cndtr_init = s->dma_chan.CNDTR;
				s->cpar_init = s->dma_chan.CPAR;
				s->cmar_init = s->dma_chan.CMAR;
				if( (s->dma_chan.CCR >> 4) & 0x01 )
				{
					// mem -> perif
					while( s->dma_chan.CNDTR )
					{
						atlantronic_update_dma(s);
					}
				}
			}			
			s->dma_chan.CCR = val;
			break;
		case offsetof(DMA_Channel_TypeDef, CNDTR):
			s->dma_chan.CNDTR = val;
			break;
		case offsetof(DMA_Channel_TypeDef, CPAR):
			s->dma_chan.CPAR = val & 0xfffffffe;
			break;
		case offsetof(DMA_Channel_TypeDef, CMAR):
			s->dma_chan.CMAR = val & 0xfffffffe;
			break;
		default:
			printf("Error : DMA Channel forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_dma_chan_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_dma_chan_state* s = opaque;

	switch(offset)
	{
		case offsetof(DMA_Channel_TypeDef, CCR):
			res = s->dma_chan.CCR;
			break;
		case offsetof(DMA_Channel_TypeDef, CNDTR):
			res = s->dma_chan.CNDTR;
			break;
		case offsetof(DMA_Channel_TypeDef, CPAR):
			res = s->dma_chan.CPAR;
			break;
		case offsetof(DMA_Channel_TypeDef, CMAR):
			res = s->dma_chan.CMAR;
			break;
		default:
			printf("Error : DMA Channel forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_dma_chan_reset(DMA_Channel_TypeDef* chan)
{
	chan->CCR = 0x00;
	chan->CNDTR = 0x00;
	chan->CPAR = 0x00;
	chan->CMAR = 0x00;
}

static const MemoryRegionOps atlantronic_dma_chan_ops =
{
	.read = atlantronic_dma_chan_read,
	.write = atlantronic_dma_chan_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_dma_chan_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_dma_chan_state *s = opaque;
	atlantronic_update_dma(s);
}

static int atlantronic_dma_chan_init(SysBusDevice * dev)
{
	struct atlantronic_dma_chan_state *s = FROM_SYSBUS(struct atlantronic_dma_chan_state, dev);

	memory_region_init_io(&s->iomem, &atlantronic_dma_chan_ops, s, "atlantronic_dma_chan", 0x010);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(&dev->qdev, &s->irq, 1);
	qdev_init_gpio_in(&dev->qdev, atlantronic_dma_chan_in_recv, 1);

	atlantronic_dma_chan_reset(&s->dma_chan);

    return 0;
}

static void atlantronic_dma_chan_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_dma_chan_init;
}

static TypeInfo atlantronic_dma_chan_info =
{
	.name          = "atlantronic-dma-chan",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_dma_chan_state),
	.class_init    = atlantronic_dma_chan_class_init,
};

static void atlantronic_dma_chan_register_types(void)
{
	type_register_static(&atlantronic_dma_chan_info);
}

type_init(atlantronic_dma_chan_register_types);
