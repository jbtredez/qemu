#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "kernel/rcc.h"

#define AN_NUM            16
#define ADC_DT           0.001f       //!< modele a 1ms
#define ADC_PERIOD_TICK  (int)(RCC_SYSCLK*ADC_DT)

struct atlantronic_adc_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	ADC_TypeDef adc;
	int16_t an[AN_NUM];
	qemu_irq irq;
	QEMUTimer* timer;
	uint64_t timer_count;
};

static void atlantronic_adc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_adc_state* s = opaque;

	switch(offset)
	{
		W_ACCESS(ADC_TypeDef, s->adc, CR1, val);
		W_ACCESS(ADC_TypeDef, s->adc, CR2, val);
		W_ACCESS(ADC_TypeDef, s->adc, SMPR1, val);
		W_ACCESS(ADC_TypeDef, s->adc, SMPR2, val);
		W_ACCESS(ADC_TypeDef, s->adc, JOFR1, val);
		W_ACCESS(ADC_TypeDef, s->adc, JOFR2, val);
		W_ACCESS(ADC_TypeDef, s->adc, JOFR3, val);
		W_ACCESS(ADC_TypeDef, s->adc, JOFR4, val);
		W_ACCESS(ADC_TypeDef, s->adc, HTR, val);
		W_ACCESS(ADC_TypeDef, s->adc, LTR, val);
		W_ACCESS(ADC_TypeDef, s->adc, SQR1, val);
		W_ACCESS(ADC_TypeDef, s->adc, SQR2, val);
		W_ACCESS(ADC_TypeDef, s->adc, SQR3, val);
		W_ACCESS(ADC_TypeDef, s->adc, JSQR, val);
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
		R_ACCESS(ADC_TypeDef, s->adc, SR, res);
		R_ACCESS(ADC_TypeDef, s->adc, CR1, res);
		R_ACCESS(ADC_TypeDef, s->adc, CR2, res);
		R_ACCESS(ADC_TypeDef, s->adc, SMPR1, res);
		R_ACCESS(ADC_TypeDef, s->adc, SMPR2, res);
		R_ACCESS(ADC_TypeDef, s->adc, JOFR1, res);
		R_ACCESS(ADC_TypeDef, s->adc, JOFR2, res);
		R_ACCESS(ADC_TypeDef, s->adc, JOFR3, res);
		R_ACCESS(ADC_TypeDef, s->adc, JOFR4, res);
		R_ACCESS(ADC_TypeDef, s->adc, HTR, res);
		R_ACCESS(ADC_TypeDef, s->adc, LTR, res);
		R_ACCESS(ADC_TypeDef, s->adc, SQR1, res);
		R_ACCESS(ADC_TypeDef, s->adc, SQR2, res);
		R_ACCESS(ADC_TypeDef, s->adc, SQR3, res);
		R_ACCESS(ADC_TypeDef, s->adc, JSQR, res);
		R_ACCESS(ADC_TypeDef, s->adc, JDR1, res);
		R_ACCESS(ADC_TypeDef, s->adc, JDR2, res);
		R_ACCESS(ADC_TypeDef, s->adc, JDR3, res);
		R_ACCESS(ADC_TypeDef, s->adc, JDR4, res);
		R_ACCESS(ADC_TypeDef, s->adc, DR, res);
		default:
			printf("Error : ADC forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_adc_timer_cb(void* arg)
{
	struct atlantronic_adc_state *s = arg;

	s->timer_count += ADC_PERIOD_TICK;
	timer_mod(s->timer, s->timer_count);

	if( s->adc.CR2 & ADC_CR2_SWSTART)
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
		sq = s->adc.SQR1;
		for( i = 0; i < MIN(num_conv,4) ; i++)
		{
			int id = sq & 0x1f;
			sq = sq >> 5;
			s->adc.DR = s->an[id];
			qemu_set_irq(s->irq, 1);
		}
		if( ! (s->adc.CR2 & ADC_CR2_CONT) )
		{
			s->adc.CR2 &= ~ADC_CR2_SWSTART;
		}
	}
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

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_adc_ops, s, "atlantronic_adc", 0x200);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), &s->irq, 1);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_adc_in_recv, AN_NUM);

	atlantronic_adc_reset(&s->adc);

	s->timer = timer_new(QEMU_CLOCK_VIRTUAL, 1, atlantronic_adc_timer_cb, s);
	s->timer_count = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ADC_PERIOD_TICK;
	timer_mod(s->timer, s->timer_count);

	// TODO hack vBat a 24V
	s->an[8] = 24 * 4096/39;

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
