#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"

struct atlantronic_dma_stream_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	MemoryRegion ram;
	DMA_Stream_TypeDef dma_stream;
	uint32_t cndtr_init;
	uint32_t cpar_init;
	uint32_t cmar_init;
	qemu_irq irq;
};

static void atlantronic_update_dma(struct atlantronic_dma_stream_state *s)
{
	if( ! s->dma_stream.CR & DMA_SxCR_EN )
	{
		// dma desactive
		return;
	}

	if( ! s->dma_stream.NDTR )
	{
		// il n'y a plus rien a copier
		return;
	}

	int dir = (s->dma_stream.CR >> 6) & 0x03;
	int pinc = (s->dma_stream.CR >> 9) & 0x01;
	int minc = (s->dma_stream.CR >> 10) & 0x01;
	int psize = 1 << ((s->dma_stream.CR >> 11) & 0x3);
	int msize = 1 << ((s->dma_stream.CR >> 13) & 0x3);

	int64_t val;
	if( dir == 0x00 )
	{
		// sens perif -> mem
		cpu_physical_memory_read(s->dma_stream.PAR, &val, psize);
		cpu_physical_memory_write(s->dma_stream.M0AR, &val, msize);
	}
	else
	{
		// sens mem -> perif
		cpu_physical_memory_read(s->dma_stream.M0AR, &val, psize);
		cpu_physical_memory_write(s->dma_stream.PAR, &val, msize);
	}

	s->dma_stream.PAR += psize * pinc;
	s->dma_stream.M0AR += msize * minc;
	s->dma_stream.NDTR--;

	if( ! s->dma_stream.NDTR )
	{
		s->dma_stream.PAR = s->cpar_init;
		s->dma_stream.M0AR = s->cmar_init;
		// si circ : rechargement cndtr
		if( ((s->dma_stream.CR >> 8) & 0x01 ) && ! dir )
		{

			s->dma_stream.NDTR = s->cndtr_init;
		}

		// IRQ fin de transfert : TCIF
		qemu_set_irq(s->irq, 0x20);
	}
}

static void atlantronic_dma_stream_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_dma_stream_state* s = opaque;

	switch(offset)
	{
		case offsetof(DMA_Stream_TypeDef, CR):
			if( ! (s->dma_stream.CR & DMA_SxCR_EN) && (val & DMA_SxCR_EN) )
			{
				s->cndtr_init = s->dma_stream.NDTR;
				s->cpar_init = s->dma_stream.PAR;
				s->cmar_init = s->dma_stream.M0AR;
				if( (s->dma_stream.CR >> 6) & 0x01 )
				{
					// mem -> perif
					while( s->dma_stream.NDTR )
					{
						atlantronic_update_dma(s);
					}
				}
			}
			else if( (s->dma_stream.CR & DMA_SxCR_EN) && ! (val & DMA_SxCR_EN))
			{
				// desactivation dma
				s->dma_stream.PAR = s->cpar_init;
				s->dma_stream.M0AR = s->cmar_init;
			}
			s->dma_stream.CR = val;
			break;
		case offsetof(DMA_Stream_TypeDef, NDTR):
			s->dma_stream.NDTR = val;
			break;
		case offsetof(DMA_Stream_TypeDef, PAR):
			s->dma_stream.PAR = val & 0xfffffffe;
			break;
		case offsetof(DMA_Stream_TypeDef, M0AR):
			s->dma_stream.M0AR = val & 0xfffffffe;
			break;
		default:
			printf("Error : DMA Channel forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_dma_stream_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_dma_stream_state* s = opaque;

	switch(offset)
	{
		case offsetof(DMA_Stream_TypeDef, CR):
			res = s->dma_stream.CR;
			break;
		case offsetof(DMA_Stream_TypeDef, NDTR):
			res = s->dma_stream.NDTR;
			break;
		case offsetof(DMA_Stream_TypeDef, PAR):
			res = s->dma_stream.PAR;
			break;
		case offsetof(DMA_Stream_TypeDef, M0AR):
			res = s->dma_stream.M0AR;
			break;
		default:
			printf("Error : DMA Channel forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_dma_stream_reset(DMA_Stream_TypeDef* chan)
{
	chan->CR = 0x00;
	chan->NDTR = 0x00;
	chan->PAR = 0x00;
	chan->M0AR = 0x00;
}

static const MemoryRegionOps atlantronic_dma_stream_ops =
{
	.read = atlantronic_dma_stream_read,
	.write = atlantronic_dma_stream_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_dma_stream_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_dma_stream_state *s = opaque;
	atlantronic_update_dma(s);
}

static int atlantronic_dma_stream_init(SysBusDevice * dev)
{
	struct atlantronic_dma_stream_state *s = OBJECT_CHECK(struct atlantronic_dma_stream_state, dev, "atlantronic-dma-stream");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_dma_stream_ops, s, "atlantronic_dma_stream", sizeof(DMA_Stream_TypeDef));
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), &s->irq, 1);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_dma_stream_in_recv, 1);

	atlantronic_dma_stream_reset(&s->dma_stream);

    return 0;
}

static void atlantronic_dma_stream_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_dma_stream_init;
}

static TypeInfo atlantronic_dma_stream_info =
{
	.name          = "atlantronic-dma-stream",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_dma_stream_state),
	.class_init    = atlantronic_dma_stream_class_init,
};

static void atlantronic_dma_stream_register_types(void)
{
	type_register_static(&atlantronic_dma_stream_info);
}

type_init(atlantronic_dma_stream_register_types);
