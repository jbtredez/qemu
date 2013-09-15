#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "atlantronic_spi.h"

struct atlantronic_spi_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	SPI_TypeDef spi;
	qemu_irq irq[SPI_IRQ_OUT_MAX];
	int halfDuplexLink;
	uint32_t cs;
};

static void atlantronic_spi_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_spi_state* s = opaque;

	switch(offset)
	{
		W_ACCESS(SPI_TypeDef, s->spi, CR1, val);
		W_ACCESS(SPI_TypeDef, s->spi, CR2, val);
		W_ACCESS(SPI_TypeDef, s->spi, SR, val);
		case offsetof(SPI_TypeDef, DR):
			// spi actif
			if( s->spi.CR1 & SPI_CR1_SPE )
			{
				// IT device
				int csx = 1;
				int i = 0;
				for(i = SPI_IRQ_OUT_DEVICE0_RX; i <=SPI_IRQ_OUT_DEVICE2_RX; i++ )
				{
					if( ! (s->cs & csx) )
					{
						qemu_set_irq(s->irq[i], val & 0xff);
					}
					csx <<= 1;
				}

				s->spi.SR |= USART_SR_TXE;
				if( s->spi.CR2 & SPI_CR2_TXEIE)
				{
					// si TXEIE => it spi : on a traité l'octet
					qemu_set_irq(s->irq[SPI_IRQ_OUT_HW], 1);
				}
			}
			break;
		W_ACCESS(SPI_TypeDef, s->spi, CRCPR, val);
		W_ACCESS(SPI_TypeDef, s->spi, RXCRCR, val);
		W_ACCESS(SPI_TypeDef, s->spi, TXCRCR, val);
		W_ACCESS(SPI_TypeDef, s->spi, I2SCFGR, val);
		W_ACCESS(SPI_TypeDef, s->spi, I2SPR, val);
		default:
			printf("Error : SPI forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_spi_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_spi_state* s = opaque;

	switch(offset)
	{
		R_ACCESS(SPI_TypeDef, s->spi, CR1, res);
		R_ACCESS(SPI_TypeDef, s->spi, CR2, res);
		R_ACCESS(SPI_TypeDef, s->spi, SR, res);
		case offsetof(SPI_TypeDef, DR):
			s->spi.SR &= ~SPI_SR_RXNE;
			res = s->spi.DR;
			break;
		R_ACCESS(SPI_TypeDef, s->spi, CRCPR, res);
		R_ACCESS(SPI_TypeDef, s->spi, RXCRCR, res);
		R_ACCESS(SPI_TypeDef, s->spi, TXCRCR, res);
		R_ACCESS(SPI_TypeDef, s->spi, I2SCFGR, res);
		R_ACCESS(SPI_TypeDef, s->spi, I2SPR, res);
		default:
			printf("Error : SPI forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_spi_reset(SPI_TypeDef* spi)
{
	spi->CR1 = 0x00;
	spi->CR2 = 0x00;
	spi->SR = 0x02;
	spi->DR = 0x00;
	spi->CRCPR = 0x07;
	spi->RXCRCR = 0x00;
	spi->TXCRCR = 0x00;
	spi->I2SCFGR = 0x00;
	spi->I2SPR = 0x02;
}

static const MemoryRegionOps atlantronic_spi_ops =
{
	.read = atlantronic_spi_read,
	.write = atlantronic_spi_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_spi_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_spi_state *s = opaque;

	if( numPin == SPI_IRQ_IN_RX )
	{
		// spi activé
		if( s->spi.CR1 & SPI_CR1_SPE )
		{
			s->spi.DR = level & 0x1ff;
			s->spi.SR |= SPI_SR_RXNE;

			if( s->spi.CR1 & SPI_CR2_RXNEIE )
			{
				// it reception : un octet a lire
				qemu_set_irq(s->irq[SPI_IRQ_OUT_HW], 1);
			}

			if( s->spi.CR2 & SPI_CR2_RXDMAEN )
			{
				// buffer dma de reception => it dmar : un octet a lire
				qemu_set_irq(s->irq[SPI_IRQ_OUT_DMAR], 1);
			}
		}
	}
	else if( numPin >= SPI_IRQ_IN_CS0 && numPin <= SPI_IRQ_IN_CS2 )
	{
		if( level )
		{
			s->cs |= 1 << (numPin - SPI_IRQ_IN_CS0);
		}
		else
		{
			s->cs &= ~(1 << (numPin - SPI_IRQ_IN_CS0));
		}
	}
}

static int atlantronic_spi_init(SysBusDevice * dev)
{
	struct atlantronic_spi_state *s = OBJECT_CHECK(struct atlantronic_spi_state, dev, "atlantronic-spi");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_spi_ops, s, "atlantronic_spi", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, SPI_IRQ_OUT_MAX);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_spi_in_recv, SPI_IRQ_IN_MAX);

	s->halfDuplexLink = 0;
	s->cs = 0xffffffff;
	atlantronic_spi_reset(&s->spi);

	return 0;
}

static void atlantronic_spi_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_spi_init;
}

static TypeInfo atlantronic_spi_info =
{
	.name          = "atlantronic-spi",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_spi_state),
	.class_init    = atlantronic_spi_class_init,
};

static void atlantronic_spi_register_types(void)
{
	type_register_static(&atlantronic_spi_info);
}

type_init(atlantronic_spi_register_types);
