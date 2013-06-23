#define LINUX
#define STM32F10X_CL
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/stm32f1xx/otgd_fs_regs.h"

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"

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

	if(rcc->CR & RCC_CR_PLL2ON)
	{
		// PLL2 prête
		rcc->CR |= RCC_CR_PLL2RDY;
	}
	else
	{
		rcc->CR &= ~RCC_CR_PLL2RDY;
	}

	if(rcc->CR & RCC_CR_PLL3ON)
	{
		// PLL3 prête
		rcc->CR |= RCC_CR_PLL3RDY;
	}
	else
	{
		rcc->CR &= ~RCC_CR_PLL3RDY;
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
		case offsetof(RCC_TypeDef, CFGR):
			s->rcc.CFGR = val;
			if(val & RCC_CFGR_SW_PLL)
			{
				// PLL utilisée pour SYSCLK
				s->rcc.CFGR &= ~RCC_CFGR_SWS;
				s->rcc.CFGR |= RCC_CFGR_SWS_1;
			}
			break;
		case offsetof(RCC_TypeDef, CIR):
			s->rcc.CIR = val;
			break;
		case offsetof(RCC_TypeDef, APB2RSTR):
			s->rcc.APB2RSTR = val;
			break;
		case offsetof(RCC_TypeDef, APB1RSTR):
			s->rcc.APB1RSTR = val;
			break;
		case offsetof(RCC_TypeDef, AHBENR):
			s->rcc.AHBENR = val;
			break;
		case offsetof(RCC_TypeDef, APB1ENR):
			s->rcc.APB1ENR = val;
			break;
		case offsetof(RCC_TypeDef, APB2ENR):
			s->rcc.APB2ENR = val;
			break;
		case offsetof(RCC_TypeDef, BDCR):
			s->rcc.BDCR = val;
			if(val & RCC_BDCR_LSEON)
			{
				printf("Error : pas de LSE\n");
			}
			break;
		case offsetof(RCC_TypeDef, CSR):
			s->rcc.CSR = val;
			break;
		case offsetof(RCC_TypeDef, AHBRSTR):
			s->rcc.AHBRSTR = val;
			break;
		case offsetof(RCC_TypeDef, CFGR2):
			s->rcc.CFGR2 = val;
			break;
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
		case offsetof(RCC_TypeDef, CR):
			res = s->rcc.CR & (uint32_t)0xFF0FFFFB;
			break;
		case offsetof(RCC_TypeDef, CFGR):
			res = s->rcc.CFGR;
			break; 
		case offsetof(RCC_TypeDef, CIR):
			res = s->rcc.CIR;
			break;
		case offsetof(RCC_TypeDef, APB2RSTR):
			res = s->rcc.APB2RSTR;
			break;
		case offsetof(RCC_TypeDef, APB1RSTR):
			res = s->rcc.APB1RSTR;
			break;
		case offsetof(RCC_TypeDef, AHBENR):
			res = s->rcc.AHBENR;
			break;
		case offsetof(RCC_TypeDef, APB1ENR):
			res = s->rcc.APB1ENR;
			break;
		case offsetof(RCC_TypeDef, APB2ENR):
			res = s->rcc.APB2ENR;
			break;
		case offsetof(RCC_TypeDef, BDCR):
			res = s->rcc.BDCR;
			break;
		case offsetof(RCC_TypeDef, CSR):
			res = s->rcc.CSR;
			break;
		case offsetof(RCC_TypeDef, AHBRSTR):
			res = s->rcc.AHBRSTR;
			break;
		case offsetof(RCC_TypeDef, CFGR2):
			res = s->rcc.CFGR2;
			break;
		default:
			printf("Error : RCC forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_rcc_reset(RCC_TypeDef* rcc)
{
	rcc->CR = 0x00000083;
	rcc->CFGR = 0x00;
	rcc->CIR = 0x00;
	rcc->APB2RSTR = 0x00;
	rcc->APB1RSTR = 0x00;
	rcc->AHBENR = 0x14;
	rcc->APB2ENR = 0x00;
	rcc->APB1ENR = 0x00;
	rcc->BDCR = 0x00;
	rcc->CSR = 0x0C000000;
	rcc->AHBRSTR = 0x00;
	rcc->CFGR2 = 0x00;
}

static const MemoryRegionOps atlantronic_rcc_ops =
{
	.read = atlantronic_rcc_read,
	.write = atlantronic_rcc_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static int atlantronic_rcc_init(SysBusDevice * dev)
{
    struct atlantronic_rcc_state *s = FROM_SYSBUS(struct atlantronic_rcc_state, dev);

	memory_region_init_io(&s->iomem, &atlantronic_rcc_ops, s, "atlantronic_rcc", 0x400);
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
