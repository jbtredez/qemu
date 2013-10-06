#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"

struct atlantronic_rcc_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	RCC_TypeDef rcc;
};

static void atlantronic_rcc_write_CR(RCC_TypeDef* rcc, uint32_t val)
{
	uint64_t write_mask = 0x1509FF01;
	if(! (rcc->CR & RCC_CR_HSION))
	{
		write_mask |= RCC_CR_HSEBYP;
	}

	rcc->CR = (rcc->CR & ~write_mask) | (val & write_mask);

	if(rcc->CR & RCC_CR_HSION)
	{
		// horloge déjà prête
		rcc->CR |= RCC_CR_HSIRDY;
	}
	else
	{
		rcc->CR &= ~RCC_CR_HSIRDY;
	}

	if(rcc->CR & RCC_CR_HSEON)
	{
		// horloge déjà prête
		rcc->CR |= RCC_CR_HSERDY;
	}
	else
	{
		rcc->CR &= ~RCC_CR_HSERDY;
	}

	if(rcc->CR & RCC_CR_PLLON)
	{
		// PLL prête
		rcc->CR |= RCC_CR_PLLRDY;
	}
	else
	{
		rcc->CR &= ~RCC_CR_PLLRDY;
	}
}

static void atlantronic_rcc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_rcc_state* s = opaque;

	switch(offset)
	{
		case offsetof(RCC_TypeDef, CR):
			atlantronic_rcc_write_CR(&s->rcc, val);
			break;
		W_ACCESS(RCC_TypeDef, s->rcc, PLLCFGR, val);
		case offsetof(RCC_TypeDef, CFGR):
			s->rcc.CFGR = val;
			if(val & RCC_CFGR_SW_PLL)
			{
				// PLL utilisée pour SYSCLK
				s->rcc.CFGR &= ~RCC_CFGR_SWS;
				s->rcc.CFGR |= RCC_CFGR_SWS_PLL;
			}
			break;
		W_ACCESS(RCC_TypeDef, s->rcc, CIR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB1RSTR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB2RSTR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB3RSTR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, APB1RSTR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, APB2RSTR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB1ENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB2ENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB3ENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, APB1ENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, APB2ENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB1LPENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB2LPENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, AHB3LPENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, APB1LPENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, APB2LPENR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, BDCR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, CSR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, SSCGR, val);
		W_ACCESS(RCC_TypeDef, s->rcc, PLLI2SCFGR, val);
		default:
			printf("Error : RCC forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_rcc_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_rcc_state* s = opaque;

	switch(offset)
	{
		R_ACCESS(RCC_TypeDef, s->rcc, CR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, PLLCFGR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, CFGR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, CIR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB1RSTR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB2RSTR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB3RSTR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, APB1RSTR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, APB2RSTR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB1ENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB2ENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB3ENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, APB1ENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, APB2ENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB1LPENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB2LPENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, AHB3LPENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, APB1LPENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, APB2LPENR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, BDCR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, CSR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, SSCGR, res);
		R_ACCESS(RCC_TypeDef, s->rcc, PLLI2SCFGR, res);
		default:
			printf("Error : RCC forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_rcc_reset(RCC_TypeDef* rcc)
{
	rcc->CR = 0x00000083;
	rcc->PLLCFGR = 0x24003010;
	rcc->CFGR = 0x00;
	rcc->CIR = 0x00;
	rcc->AHB1RSTR = 0x00;
	rcc->AHB2RSTR = 0x00;
	rcc->AHB3RSTR = 0x00;
	rcc->APB1RSTR = 0x00;
	rcc->APB2RSTR = 0x00;
	rcc->AHB1ENR = 0x00100000;
	rcc->AHB2ENR = 0x00;
	rcc->AHB3ENR = 0x00;
	rcc->APB1ENR = 0x00;
	rcc->APB2ENR = 0x00;
	rcc->AHB1LPENR = 0x7E6791FF;
	rcc->AHB2LPENR = 0x000000F1;
	rcc->AHB3LPENR = 0x00000001;
	rcc->APB1LPENR = 0x36FEC9FF;
	rcc->APB2LPENR = 0x00075F33;
	rcc->BDCR = 0x00;
	rcc->CSR = 0x0E000000;
	rcc->SSCGR = 0x00;
	rcc->PLLI2SCFGR = 0x20003000;
}

static const MemoryRegionOps atlantronic_rcc_ops =
{
	.read = atlantronic_rcc_read,
	.write = atlantronic_rcc_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static int atlantronic_rcc_init(SysBusDevice * dev)
{
    struct atlantronic_rcc_state *s = OBJECT_CHECK(struct atlantronic_rcc_state, dev, "atlantronic-rcc");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_rcc_ops, s, "atlantronic_rcc", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	atlantronic_rcc_reset(&s->rcc);

    return 0;
}

static void atlantronic_rcc_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_rcc_init;
}

static TypeInfo atlantronic_rcc_info =
{
	.name          = "atlantronic-rcc",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_rcc_state),
	.class_init    = atlantronic_rcc_class_init,
};

static void atlantronic_rcc_register_types(void)
{
	type_register_static(&atlantronic_rcc_info);
}

type_init(atlantronic_rcc_register_types);
