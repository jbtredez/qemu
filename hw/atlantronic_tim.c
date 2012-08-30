#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/otgd_fs_regs.h"

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"
#include "exec-memory.h"

#define IRQ_NUM           5

struct atlantronic_tim_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	TIM_TypeDef tim;
	qemu_irq irq[IRQ_NUM];
};

static void atlantronic_tim_write(void *opaque, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	struct atlantronic_tim_state* s = opaque;

	switch(offset)
	{
		case offsetof(TIM_TypeDef, CR1):
			s->tim.CR1 = val & 0x3FF;
			break;
		case offsetof(TIM_TypeDef, CR2):
			s->tim.CR2 = val & 0x7FFD;
			break;
		case offsetof(TIM_TypeDef, SMCR):
			s->tim.SMCR = val;
			break;
		case offsetof(TIM_TypeDef, DIER):
			s->tim.DIER = val & 0x7FFF;
			break;
		case offsetof(TIM_TypeDef, SR):
			s->tim.SR = val;
			break;
		case offsetof(TIM_TypeDef, EGR):
			s->tim.EGR = val & 0xFF;
			break;
		case offsetof(TIM_TypeDef, CCMR1):
			s->tim.CCMR1 = val;
			break;
		case offsetof(TIM_TypeDef, CCMR2):
			s->tim.CCMR2 = val;
			break;
		case offsetof(TIM_TypeDef, CCER):
			s->tim.CCER = val;
			break;
		case offsetof(TIM_TypeDef, CNT):
			qemu_set_irq(s->irq[0], (s->tim.CCR1 << 16) / (s->tim.ARR + 1) );
			s->tim.CNT = val;
			break;
		case offsetof(TIM_TypeDef, PSC):
			s->tim.PSC = val;
			break;
		case offsetof(TIM_TypeDef, ARR):
			if(val > TIM_ARR_ARR)
			{
				val = TIM_ARR_ARR;
			}
			s->tim.ARR = val;
			break;
		case offsetof(TIM_TypeDef, RCR):
			s->tim.RCR = val;
			break;
		case offsetof(TIM_TypeDef, CCR1):
			if(val > s->tim.ARR)
			{
				val = s->tim.ARR;
			}
			s->tim.CCR1 = val;
			qemu_set_irq(s->irq[1], (val << 16) / (s->tim.ARR + 1 ));
			break;
		case offsetof(TIM_TypeDef, CCR2):
			if(val > s->tim.ARR)
			{
				val = s->tim.ARR;
			}
			s->tim.CCR2 = val;
			qemu_set_irq(s->irq[2], (val << 16) / (s->tim.ARR + 1 ));
			break;
		case offsetof(TIM_TypeDef, CCR3):
			if(val > s->tim.ARR)
			{
				val = s->tim.ARR;
			}
			s->tim.CCR3 = val;
			qemu_set_irq(s->irq[3], (val << 16) / (s->tim.ARR + 1 ));
			break;
		case offsetof(TIM_TypeDef, CCR4):
			if(val > s->tim.ARR)
			{
				val = s->tim.ARR;
			}
			s->tim.CCR4 = val;
			qemu_set_irq(s->irq[4], (val << 16) / (s->tim.ARR + 1 ));
			break;
		case offsetof(TIM_TypeDef, BDTR):
			s->tim.BDTR = val;
			break;
		case offsetof(TIM_TypeDef, DCR):
			s->tim.DCR = val;
			break;
		case offsetof(TIM_TypeDef, DMAR):
			s->tim.DMAR = val;
			break;
		default:
			printf("Error : TIM forbiden write acces offset %x, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_tim_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_tim_state* s = opaque;

	switch(offset)
	{
		case offsetof(TIM_TypeDef, CR1):
			res = s->tim.CR1;
			break;
		case offsetof(TIM_TypeDef, CR2):
			res = s->tim.CR2;
			break;
		case offsetof(TIM_TypeDef, SMCR):
			res = s->tim.SMCR;
			break;
		case offsetof(TIM_TypeDef, DIER):
			res = s->tim.DIER;
			break;
		case offsetof(TIM_TypeDef, SR):
			res = s->tim.SR;
			break;
		case offsetof(TIM_TypeDef, EGR):
			res = s->tim.EGR;
			break;
		case offsetof(TIM_TypeDef, CCMR1):
			res = s->tim.CCMR1;
			break;
		case offsetof(TIM_TypeDef, CCMR2):
			res = s->tim.CCMR2;
			break;
		case offsetof(TIM_TypeDef, CCER):
			res = s->tim.CCER;
			break;
		case offsetof(TIM_TypeDef, CNT):
			res = s->tim.CNT;
			break;
		case offsetof(TIM_TypeDef, PSC):
			res = s->tim.PSC;
			break;
		case offsetof(TIM_TypeDef, ARR):
			res = s->tim.ARR;
			break;
		case offsetof(TIM_TypeDef, RCR):
			res = s->tim.RCR;
			break;
		case offsetof(TIM_TypeDef, CCR1):
			res = s->tim.CCR1;
			break;
		case offsetof(TIM_TypeDef, CCR2):
			res = s->tim.CCR2;
			break;
		case offsetof(TIM_TypeDef, CCR3):
			res = s->tim.CCR3;
			break;
		case offsetof(TIM_TypeDef, CCR4):
			res = s->tim.CCR4;
			break;
		case offsetof(TIM_TypeDef, BDTR):
			res = s->tim.BDTR;
			break;
		case offsetof(TIM_TypeDef, DCR):
			res = s->tim.DCR;
			break;
		case offsetof(TIM_TypeDef, DMAR):
			res = s->tim.DMAR;
			break;
		default:
			printf("Error : TIM forbiden read acces offset %x\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_tim_reset(TIM_TypeDef* tim)
{
	tim->CR1   = 0x00;
	tim->CR2   = 0x00;
	tim->SMCR  = 0x00;
	tim->DIER  = 0x00;
	tim->SR    = 0x00;
	tim->EGR   = 0x00;
	tim->CCMR1 = 0x00;
	tim->CCMR2 = 0x00;
	tim->CCER  = 0x00;
	tim->CNT   = 0x00;
	tim->PSC   = 0x00;
	tim->ARR   = 0x00;
	tim->RCR   = 0x00;
	tim->CCR1  = 0x00;
	tim->CCR2  = 0x00;
	tim->CCR3  = 0x00;
	tim->CCR4  = 0x00;
	tim->BDTR  = 0x00;
	tim->DCR   = 0x00;
	tim->DMAR  = 0x00;
}

static const MemoryRegionOps atlantronic_tim_ops =
{
	.read = atlantronic_tim_read,
	.write = atlantronic_tim_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_tim_in_recv(void * opaque, int numPin, int level)
{
    struct atlantronic_tim_state *s = opaque;

	s->tim.CNT = level;
}

static int atlantronic_tim_init(SysBusDevice * dev)
{
    struct atlantronic_tim_state *s = FROM_SYSBUS(struct atlantronic_tim_state, dev);

	memory_region_init_io(&s->iomem, &atlantronic_tim_ops, s, "atlantronic_tim", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(&dev->qdev, s->irq, IRQ_NUM);
	qdev_init_gpio_in(&dev->qdev, atlantronic_tim_in_recv, 1);

	atlantronic_tim_reset(&s->tim);

    return 0;
}

static void atlantronic_tim_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_tim_init;
}

static TypeInfo atlantronic_tim_info =
{
	.name          = "atlantronic-tim",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_tim_state),
	.class_init    = atlantronic_tim_class_init,
};

static void atlantronic_tim_register_types(void)
{
	type_register_static(&atlantronic_tim_info);
}

type_init(atlantronic_tim_register_types);
