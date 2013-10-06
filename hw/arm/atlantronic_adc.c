#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"

#define AN_NUM            16

struct atlantronic_adc_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	ADC_TypeDef adc;
	int16_t an[AN_NUM];
	qemu_irq irq;
};

static void atlantronic_adc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_adc_state* s = opaque;

	switch(offset)
	{
		case offsetof(ADC_TypeDef, CR1):
			s->adc.CR1 = val & 0xCFFFFF;
			break;
		case offsetof(ADC_TypeDef, CR2):
			s->adc.CR2 = val & 0xFEFF0F;
			if( val & ADC_CR2_SWSTART)
			{
				// on donne les valeurs au module dma
				int num_conv = ((s->adc.SQR1 & ADC_SQR1_L) >> 20) + 1;
				int i = 0;
				uint32_t sq = s->adc.SQR3;
				for( i = 0; i < MIN(num_conv,6) ; i++)
				{
					int id = sq & 0x1f;
					sq = sq >> 5;
					s->adc.DR = s->an[id];
					qemu_set_irq(s->irq, 1);
				}

				num_conv -= 6;
				sq = s->adc.SQR2;
				for( i = 0; i < MIN(num_conv,6) ; i++)
				{
					int id = sq & 0x1f;
					sq = sq >> 5;
					s->adc.DR = s->an[id];
					qemu_set_irq(s->irq, 1);
				}

				num_conv -= 6;
				sq = s->adc.SQR3;
				for( i = 0; i < MIN(num_conv,4) ; i++)
				{
					int id = sq & 0x1f;
					sq = sq >> 5;
					s->adc.DR = s->an[id];
					qemu_set_irq(s->irq, 1);
				}
			}
			break;
		case offsetof(ADC_TypeDef, SMPR1):
			s->adc.SMPR1 = val & 0xFFFFFF;
			break;
		case offsetof(ADC_TypeDef, SMPR2):
			s->adc.SMPR2 = val & 0x3FFFFFFF;
			break;
		case offsetof(ADC_TypeDef, JOFR1):
			s->adc.JOFR1 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, JOFR2):
			s->adc.JOFR2 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, JOFR3):
			s->adc.JOFR3 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, JOFR4):
			s->adc.JOFR4 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, HTR):
			s->adc.HTR = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, LTR):
			s->adc.LTR = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, SQR1):
			s->adc.SQR1 = val & 0xFFFFFF;
			break;
		case offsetof(ADC_TypeDef, SQR2):
			s->adc.SQR2 = val & 0x3FFFFFFF;
			break;
		case offsetof(ADC_TypeDef, SQR3):
			s->adc.SQR3 = val & 0x3FFFFFFF;
			break;
		case offsetof(ADC_TypeDef, JSQR):
			s->adc.JSQR = val & 0x3FFFFF;
			break;
		case offsetof(ADC_TypeDef, SR):
		case offsetof(ADC_TypeDef, JDR1):
		case offsetof(ADC_TypeDef, JDR2):
		case offsetof(ADC_TypeDef, JDR3):
		case offsetof(ADC_TypeDef, JDR4):
		case offsetof(ADC_TypeDef, DR):
		default:
			printf("Error : ADC forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_adc_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_adc_state* s = opaque;

	switch(offset)
	{
		case offsetof(ADC_TypeDef, SR):
			res = s->adc.SR;
			break;
		case offsetof(ADC_TypeDef, CR1):
			res = s->adc.CR1;
			break;
		case offsetof(ADC_TypeDef, CR2):
			res = s->adc.CR2;
			break;
		case offsetof(ADC_TypeDef, SMPR1):
			res = s->adc.SMPR1;
			break;
		case offsetof(ADC_TypeDef, SMPR2):
			res = s->adc.SMPR2;
			break;
		case offsetof(ADC_TypeDef, JOFR1):
			res = s->adc.JOFR1;
			break;
		case offsetof(ADC_TypeDef, JOFR2):
			res = s->adc.JOFR2;
			break;
		case offsetof(ADC_TypeDef, JOFR3):
			res = s->adc.JOFR3;
			break;
		case offsetof(ADC_TypeDef, JOFR4):
			res = s->adc.JOFR4;
			break;
		case offsetof(ADC_TypeDef, HTR):
			res = s->adc.HTR;
			break;
		case offsetof(ADC_TypeDef, LTR):
			res = s->adc.LTR;
			break;
		case offsetof(ADC_TypeDef, SQR1):
			res = s->adc.SQR1;
			break;
		case offsetof(ADC_TypeDef, SQR2):
			res = s->adc.SQR2;
			break;
		case offsetof(ADC_TypeDef, SQR3):
			res = s->adc.SQR3;
			break;
		case offsetof(ADC_TypeDef, JSQR):
			res = s->adc.JSQR;
			break;
		case offsetof(ADC_TypeDef, JDR1):
			res = s->adc.JDR1;
			break;
		case offsetof(ADC_TypeDef, JDR2):
			res = s->adc.JDR2;
			break;
		case offsetof(ADC_TypeDef, JDR3):
			res = s->adc.JDR3;
			break;
		case offsetof(ADC_TypeDef, JDR4):
			res = s->adc.JDR4;
			break;
		case offsetof(ADC_TypeDef, DR):
			res = s->adc.DR;
			break;
		default:
			printf("Error : ADC forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_adc_reset(ADC_TypeDef* adc)
{
	adc->SR    = 0x00;
	adc->CR1   = 0x00;
	adc->CR2   = 0x00;
	adc->SMPR1 = 0x00;
	adc->SMPR2 = 0x00;
	adc->JOFR1 = 0x00;
	adc->JOFR2 = 0x00;
	adc->JOFR3 = 0x00;
	adc->JOFR4 = 0x00;
	adc->HTR   = 0x0fff;
	adc->LTR   = 0x00;
	adc->SQR1  = 0x00;
	adc->SQR2  = 0x00;
	adc->SQR3  = 0x00;
	adc->JSQR  = 0x00;
	adc->JDR1  = 0x00;
	adc->JDR2  = 0x00;
	adc->JDR3  = 0x00;
	adc->JDR4  = 0x00;
	adc->DR    = 0x00;
}

static const MemoryRegionOps atlantronic_adc_ops =
{
	.read = atlantronic_adc_read,
	.write = atlantronic_adc_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_adc_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_adc_state *s = opaque;

	s->an[numPin] = level;
}

static int atlantronic_adc_init(SysBusDevice * dev)
{
	struct atlantronic_adc_state *s = OBJECT_CHECK(struct atlantronic_adc_state, dev, "atlantronic-adc");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_adc_ops, s, "atlantronic_adc", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), &s->irq, 1);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_adc_in_recv, AN_NUM);

	atlantronic_adc_reset(&s->adc);

    return 0;
}

static void atlantronic_adc_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_adc_init;
}

static TypeInfo atlantronic_adc_info =
{
	.name          = "atlantronic-adc",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_adc_state),
	.class_init    = atlantronic_adc_class_init,
};

static void atlantronic_adc_register_types(void)
{
	type_register_static(&atlantronic_adc_info);
}

type_init(atlantronic_adc_register_types);
