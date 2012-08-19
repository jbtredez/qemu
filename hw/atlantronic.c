#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/otgd_fs_regs.h"

#include "sysbus.h"
#include "arm-misc.h"
//#include "devices.h"
//#include "qemu-timer.h"
#include "boards.h"
#include "exec-memory.h"

/////////////// GPIO

/////////////// FIN GPIO



///////////////// RCC
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

static void atlantronic_rcc_write(void *opaque, target_phys_addr_t offset, uint64_t val, unsigned size)
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
			printf("Error : RCC forbiden write acces offset %x, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_rcc_read(void *opaque, target_phys_addr_t offset, unsigned size)
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
			printf("Error : RCC forbiden read acces offset %x\n", offset);
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

////////////////// FIN RCC
////////////////// USB
struct atlantronic_usb_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	USB_OTG_GREGS gregs;
	USB_OTG_DEV dev;
	USB_OTG_DINEPS dineps[NUM_TX_FIFOS];
	USB_OTG_DOUTEPS douteps[NUM_TX_FIFOS];
	uint32_t fifo[USB_OTG_DATA_FIFO_SIZE/4 * NUM_TX_FIFOS];
	int32_t fifo_start[NUM_TX_FIFOS];
	int32_t fifo_end[NUM_TX_FIFOS];
	uint32_t pcgcctl;

	// fifos d'envoi
	int32_t tx_count[NUM_TX_FIFOS];
	qemu_irq irq;
};

static void atlantronic_usb_reset(struct atlantronic_usb_state* usb)
{
	int i;

	usb->gregs.GOTGCTL = 0x00000800;
	usb->gregs.GOTGINT = 0x00;
	usb->gregs.GAHBCFG = 0x00;
	usb->gregs.GUSBCFG = 0x00000A00;
	usb->gregs.GRSTCTL = 0x20000000;
	usb->gregs.GINTSTS = 0x04000020;
	usb->gregs.GINTMSK = 0x00;
	usb->gregs.GRXSTSR = 0x00;
	usb->gregs.GRXSTSP = 0x00;
	usb->gregs.GRXFSIZ = 0x00000200;
	usb->gregs.DIEPTXF0 = 0x00000200;
	usb->gregs.HNPTXSTS = 0x00080200;
	usb->gregs.GCCFG = 0x00;
	usb->gregs.CID = 0x00001000;
	usb->gregs.HPTXFSIZ = 0x02000600;
	usb->gregs.DIEPTXFx[0] = 0x02000400;
	usb->gregs.DIEPTXFx[1] = 0x02000400;
	usb->gregs.DIEPTXFx[2] = 0x02000400;

	usb->dev.DCFG = 0x02200000;
	usb->dev.DCTL = 0x00;
	usb->dev.DSTS = 0x00000010;
	usb->dev.DIEPMSK = 0x00;
	usb->dev.DOEPMSK = 0x00;
	usb->dev.DAINT = 0x00;
	usb->dev.DAINTMSK = 0x00;
	usb->dev.DVBUSDIS = 0x000017D7;
	usb->dev.DVBUSPULSE = 0x000005B8;
	usb->dev.DIEPEMPMSK = 0x00;

	for(i = 0; i < NUM_TX_FIFOS; i++)
	{
		usb->dineps[i].DIEPCTLx = 0x00;
		usb->dineps[i].DIEPINTx = 0x00000080;
		usb->dineps[i].DIEPTSIZx = 0x00;
		usb->dineps[i].DTXFSTSx = 0x00;

		usb->douteps[i].DOEPCTLx = 0x00008000;
		usb->douteps[i].DOEPINTx = 0x00000080;
		usb->douteps[i].DOEPTSIZx = 0x00;
		
		usb->fifo_start[i] = 0;
		usb->fifo_end[i] = 0;

		usb->tx_count[i] = 0;
	}

	usb->pcgcctl = 0x00;
}

static uint32_t atlantronic_usb_read32_fifo(struct atlantronic_usb_state *usb, int fifo_id)
{
	// TODO check sur start et end
	uint32_t data = usb->fifo[fifo_id*USB_OTG_DATA_FIFO_SIZE/4 + usb->fifo_start[fifo_id]];
	usb->fifo_start[fifo_id]++;
	if( usb->fifo_start[fifo_id] >= USB_OTG_DATA_FIFO_SIZE/4)
	{
		usb->fifo_start[fifo_id] = 0;
	}

	return data;
}

static void atlantronic_usb_write32_fifo(struct atlantronic_usb_state *usb, int fifo_id, uint32_t data)
{
	// TODO check sur start et end
	usb->fifo[fifo_id*USB_OTG_DATA_FIFO_SIZE/4 + usb->fifo_end[fifo_id]] = data;
	usb->fifo_end[fifo_id]++;
	if( usb->fifo_end[fifo_id] >= USB_OTG_DATA_FIFO_SIZE/4)
	{
		usb->fifo_end[fifo_id] = 0;
	}
}

static void atlantronic_usb_write_rx_fifo0_req(struct atlantronic_usb_state *usb, uint8_t request_type, uint8_t request, uint16_t wvalue, uint16_t windex, uint16_t length, uint8_t* data)
{
	int i = 0;
	atlantronic_usb_write32_fifo(usb, 0, request_type + (request << 8) + (wvalue << 16));
	atlantronic_usb_write32_fifo(usb, 0, windex + (length << 16));

	int max = 4*(length << 2);
	for(i = 0; i < max; i += 4)
	{
		atlantronic_usb_write32_fifo(usb, 0, *((uint32_t*) (data + i)));
	}

	switch(length - i)
	{
		case 1:
			atlantronic_usb_write32_fifo(usb, 0, data[i]);
			break;
		case 2:
			atlantronic_usb_write32_fifo(usb, 0, data[i] + (data[i+1] << 8) );
			break;
		case 3:
			atlantronic_usb_write32_fifo(usb, 0, data[i] + (data[i+1] << 8) + (data[i+1] << 16));
			break;
	}
}

static void atlantronic_usb_write_gregs(struct atlantronic_usb_state *usb, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	switch(offset)
	{
		case offsetof(USB_OTG_GREGS, GOTGCTL):
			usb->gregs.GOTGCTL = val;
			break;
		case offsetof(USB_OTG_GREGS, GOTGINT):
			usb->gregs.GOTGINT = val;
			break;
		case offsetof(USB_OTG_GREGS, GAHBCFG):
			usb->gregs.GAHBCFG = val;
			if( val & 0x01)
			{
				usb->gregs.GINTSTS |= 0x1000; // reset
				qemu_set_irq(usb->irq, 1);
			}
			break;
		case offsetof(USB_OTG_GREGS, GUSBCFG):
			usb->gregs.GUSBCFG = val;
			break;
		case offsetof(USB_OTG_GREGS, GRSTCTL):
			usb->gregs.GRSTCTL = val;
			break;
		case offsetof(USB_OTG_GREGS, GINTSTS):
			if( (val & 0x1000) && (usb->gregs.GINTSTS & 0x1000)) // clear bit d'IT reset
			{
				atlantronic_usb_write_rx_fifo0_req(usb, 0x00, 0x09, 0x01, 0x00, 0x00, NULL);
				usb->gregs.GRXSTSP |= 0xc0000;       // SETUP data packet received (PKTSTS)
				usb->gregs.GINTSTS |= 0x10;          // IT RX fifo
				usb->douteps[0].DOEPINTx = 0x08;    // EP0 -> STUP (SETUP phase done)
				usb->gregs.GINTSTS |= 0x80000;       // IT out EP
				qemu_set_irq(usb->irq, 1);
			}
			else
			{
				qemu_set_irq(usb->irq, 0);
			}
			usb->gregs.GINTSTS &= ~(usb->gregs.GINTSTS & val);
			break;
		case offsetof(USB_OTG_GREGS, GINTMSK):
			usb->gregs.GINTMSK = val;
			break;
		case offsetof(USB_OTG_GREGS, GRXSTSR):
			usb->gregs.GRXSTSR = val;
			break;
		case offsetof(USB_OTG_GREGS, GRXSTSP):
			usb->gregs.GRXSTSP = val;
			break;
		case offsetof(USB_OTG_GREGS, GRXFSIZ):
			usb->gregs.GRXFSIZ = val;
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXF0):
			usb->gregs.DIEPTXF0 = val;
			break;
		case offsetof(USB_OTG_GREGS, HNPTXSTS):
			usb->gregs.HNPTXSTS = val;
			break;
		case offsetof(USB_OTG_GREGS, GCCFG):
			usb->gregs.GCCFG = val;
			break;
		case offsetof(USB_OTG_GREGS, CID):
			usb->gregs.CID = val;
			break;
		case offsetof(USB_OTG_GREGS, HPTXFSIZ):
			usb->gregs.HPTXFSIZ = val;
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXFx[0]):
			usb->gregs.DIEPTXFx[0] = val;
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXFx[1]):
			usb->gregs.DIEPTXFx[1] = val;
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXFx[2]):
			usb->gregs.DIEPTXFx[2] = val;
			break;
		default:
			printf("Error : USB forbiden gregs write acces offset %x, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_usb_read_gregs(struct atlantronic_usb_state *usb, target_phys_addr_t offset, unsigned size)
{
	uint64_t val = 0;

	switch(offset)
	{
		case offsetof(USB_OTG_GREGS, GOTGCTL):
			val = usb->gregs.GOTGCTL;
			break;
		case offsetof(USB_OTG_GREGS, GOTGINT):
			val = usb->gregs.GOTGINT;
			break; 
		case offsetof(USB_OTG_GREGS, GAHBCFG):
			val = usb->gregs.GAHBCFG;
			break;
		case offsetof(USB_OTG_GREGS, GUSBCFG):
			val = usb->gregs.GUSBCFG;
			break;
		case offsetof(USB_OTG_GREGS, GRSTCTL):
			val = usb->gregs.GRSTCTL;
			break;
		case offsetof(USB_OTG_GREGS, GINTSTS):
			val = usb->gregs.GINTSTS;
			break;
		case offsetof(USB_OTG_GREGS, GINTMSK):
			val = usb->gregs.GINTMSK;
			break;
		case offsetof(USB_OTG_GREGS, GRXSTSR):
			val = usb->gregs.GRXSTSR;
			break;
		case offsetof(USB_OTG_GREGS, GRXSTSP):
			val = usb->gregs.GRXSTSP;
			break;
		case offsetof(USB_OTG_GREGS, GRXFSIZ):
			val = usb->gregs.GRXFSIZ;
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXF0):
			val = usb->gregs.DIEPTXF0;
			break;
		case offsetof(USB_OTG_GREGS, HNPTXSTS):
			val = usb->gregs.HNPTXSTS;
			break;
		case offsetof(USB_OTG_GREGS, GCCFG):
			val = usb->gregs.GCCFG;
			break;
		case offsetof(USB_OTG_GREGS, CID):
			val = usb->gregs.CID;
			break;
		case offsetof(USB_OTG_GREGS, HPTXFSIZ):
			val = usb->gregs.HPTXFSIZ;
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXFx[0]):
			val = usb->gregs.DIEPTXFx[0];
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXFx[1]):
			val = usb->gregs.DIEPTXFx[1];
			break;
		case offsetof(USB_OTG_GREGS, DIEPTXFx[2]):
			val = usb->gregs.DIEPTXFx[2];
			break;
		default:
			printf("Error : USB forbiden gregs read acces offset %x\n", offset);
			break;
	}

	return val;
}

static void atlantronic_usb_write_dev(struct atlantronic_usb_state *usb, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	switch(offset)
	{
		case offsetof(USB_OTG_DEV, DCFG):
			usb->dev.DCFG = val;
			break;
		case offsetof(USB_OTG_DEV, DCTL):
			usb->dev.DCTL = val;
			break;
		case offsetof(USB_OTG_DEV, DSTS):
			usb->dev.DSTS = val;
			break;
		case offsetof(USB_OTG_DEV, DIEPMSK):
			usb->dev.DIEPMSK = val;
			break;
		case offsetof(USB_OTG_DEV, DOEPMSK):
			usb->dev.DOEPMSK = val;
			break;
		case offsetof(USB_OTG_DEV, DAINT):
			usb->dev.DAINT = val;
			break;
		case offsetof(USB_OTG_DEV, DAINTMSK):
			usb->dev.DAINTMSK = val;
			break;
		case offsetof(USB_OTG_DEV, DVBUSDIS):
			usb->dev.DVBUSDIS = val;
			break;
		case offsetof(USB_OTG_DEV, DVBUSPULSE):
			usb->dev.DVBUSPULSE = val;
			break;
		case offsetof(USB_OTG_DEV, DIEPEMPMSK):
			usb->dev.DIEPEMPMSK = val;
			break;
		default:
			printf("Error : USB forbiden dev write acces offset %x\n", offset);
			break;
	}
}

static uint64_t atlantronic_usb_read_dev(struct atlantronic_usb_state *usb, target_phys_addr_t offset, unsigned size)
{
	uint64_t val = 0;

	switch(offset)
	{
		case offsetof(USB_OTG_DEV, DCFG):
			val = usb->dev.DCFG;
			break;
		case offsetof(USB_OTG_DEV, DCTL):
			val = usb->dev.DCTL;
			break;
		case offsetof(USB_OTG_DEV, DSTS):
			val = usb->dev.DSTS;
			break;
		case offsetof(USB_OTG_DEV, DIEPMSK):
			val = usb->dev.DIEPMSK;
			break;
		case offsetof(USB_OTG_DEV, DOEPMSK):
			val = usb->dev.DOEPMSK;
			break;
		case offsetof(USB_OTG_DEV, DAINT):
			val = usb->dev.DAINT;
			break;
		case offsetof(USB_OTG_DEV, DAINTMSK):
			val = usb->dev.DAINTMSK;
			break;
		case offsetof(USB_OTG_DEV, DVBUSDIS):
			val = usb->dev.DVBUSDIS;
			break;
		case offsetof(USB_OTG_DEV, DVBUSPULSE):
			val = usb->dev.DVBUSPULSE;
			break;
		case offsetof(USB_OTG_DEV, DIEPEMPMSK):
			val = usb->dev.DIEPEMPMSK;
			break;
		default:
			printf("Error : USB forbiden dev read acces offset %x\n", offset);
			break;
	}

	return val;
}

static void atlantronic_usb_write_dineps(struct atlantronic_usb_state *usb, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	int i = offset / sizeof(USB_OTG_DINEPS);

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden dineps write acces offset %x (=> i = %d)\n", offset, i);
		return;
	}

	printf("dineps : write acces offset %x (=> i = %d) -- val = %lx\n", offset, i, val);

	switch(offset % sizeof(USB_OTG_DINEPS))
	{
		case offsetof(USB_OTG_DINEPS, DIEPCTLx):
			if( val & 80000000 && i > 0) // EPNA et endpoint != 0
			{
				int xfrsiz = usb->dineps[i].DIEPTSIZx & 0x7ffff;
				if(xfrsiz > 0)
				{
					usb->dineps[i].DIEPINTx = 0x80; // txfempty et pas d'autres flags
					usb->dineps[i].DTXFSTSx = USB_OTG_DATA_FIFO_SIZE/4; // taille libre de la fifo
					usb->gregs.GINTSTS |= 0x40000;       // IT in EP (inepint)
					qemu_set_irq(usb->irq, 1);
				}
				else
				{
					qemu_set_irq(usb->irq, 0);
				}
			}
			else
			{
				qemu_set_irq(usb->irq, 0);
			}
			usb->dineps[i].DIEPCTLx = val;
			break;
		case offsetof(USB_OTG_DINEPS, DIEPINTx):
			usb->dineps[i].DIEPINTx = val;
			if(val == 0x80)
			{
				// Une fois le message lu, on met le flag xfrc (transfert terminé)
				int xfrsiz = usb->dineps[i].DIEPTSIZx & 0x7ffff;
				if(xfrsiz <= usb->tx_count[i])
				{
					printf("complete\n");
					usb->tx_count[i] = 0;
					usb->dineps[i].DIEPINTx |= 0x01; // xfrc
					usb->gregs.GINTSTS |= 0x40000;   // IT in EP (inepint)
					qemu_set_irq(usb->irq, 1);
				}
				else
				{
					qemu_set_irq(usb->irq, 0);
				}
			}
			else
			{
				qemu_set_irq(usb->irq, 0);
			}
			break;
		case offsetof(USB_OTG_DINEPS, DIEPTSIZx):
			usb->dineps[i].DIEPTSIZx = val;
			break;
		case offsetof(USB_OTG_DINEPS, DTXFSTSx):
			usb->dineps[i].DTXFSTSx = val;
			break;
		default:
			printf("Error : USB forbiden dineps write acces offset %x\n", offset);
			break;
	}
}

static uint64_t atlantronic_usb_read_dineps(struct atlantronic_usb_state *usb, target_phys_addr_t offset, unsigned size)
{
	uint64_t val = 0;
	int i = offset / sizeof(USB_OTG_DINEPS);

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden dineps read acces offset %x (=> i = %d)\n", offset, i);
		return 0;
	}

	switch(offset % sizeof(USB_OTG_DINEPS))
	{
		case offsetof(USB_OTG_DINEPS, DIEPCTLx):
			val = usb->dineps[i].DIEPCTLx;
			break;
		case offsetof(USB_OTG_DINEPS, DIEPINTx):
			val = usb->dineps[i].DIEPINTx;
			break;
		case offsetof(USB_OTG_DINEPS, DIEPTSIZx):
			val = usb->dineps[i].DIEPTSIZx;
			break;
		case offsetof(USB_OTG_DINEPS, DTXFSTSx):
			val = usb->dineps[i].DTXFSTSx;
			break;
		default:
			printf("Error : USB forbiden dineps read acces offset %x\n", offset);
			break;
	}

	return val;
}

static void atlantronic_usb_write_douteps(struct atlantronic_usb_state *usb, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	int i = offset / sizeof(USB_OTG_DOUTEPS);

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden douteps write acces offset %x (=> i = %d)\n", offset, i);
		return;
	}

	switch(offset % sizeof(USB_OTG_DOUTEPS))
	{
		case offsetof(USB_OTG_DOUTEPS, DOEPCTLx):
			usb->douteps[i].DOEPCTLx = val;
			break;
		case offsetof(USB_OTG_DOUTEPS, DOEPINTx):
			usb->douteps[i].DOEPINTx = val;
			break;
		case offsetof(USB_OTG_DOUTEPS, DOEPTSIZx):
			usb->douteps[i].DOEPTSIZx = val;
			break;
		default:
			printf("Error : USB forbiden douteps write acces offset %x\n", offset);
			break;
	}
}

static uint64_t atlantronic_usb_read_douteps(struct atlantronic_usb_state *usb, target_phys_addr_t offset, unsigned size)
{
	uint64_t val = 0;
	int i = offset / sizeof(USB_OTG_DOUTEPS);

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden douteps read acces offset %x (=> i = %d)\n", offset, i);
		return 0;
	}

	switch(offset % sizeof(USB_OTG_DOUTEPS))
	{
		case offsetof(USB_OTG_DOUTEPS, DOEPCTLx):
			val = usb->douteps[i].DOEPCTLx;
			break;
		case offsetof(USB_OTG_DOUTEPS, DOEPINTx):
			val = usb->douteps[i].DOEPINTx;
			break;
		case offsetof(USB_OTG_DOUTEPS, DOEPTSIZx):
			val = usb->douteps[i].DOEPTSIZx;
			break;
		default:
			printf("Error : USB forbiden douteps read acces offset %x\n", offset);
			break;
	}

	return val;
}

static uint64_t atlantronic_usb_read_fifo(struct atlantronic_usb_state *usb, target_phys_addr_t offset, unsigned size)
{
	int i = offset / USB_OTG_DATA_FIFO_SIZE;

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden fifo read acces offset %x (=> i = %d)\n", offset, i);
		return 0;
	}

	// on doit lire uniquement le premier élément
	if(offset - i* USB_OTG_DATA_FIFO_SIZE != 0)
	{
		printf("Error : USB forbiden fifo read acces offset %x (=> i = %d)\n", offset, i);
		return 0;
	}

	return atlantronic_usb_read32_fifo(usb, i);
}

static void atlantronic_usb_write_fifo(struct atlantronic_usb_state *usb, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	int i = offset / USB_OTG_DATA_FIFO_SIZE;

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden fifo read acces offset %x (=> i = %d)\n", offset, i);
		return;
	}

	// TODO : envoyer sur une socket ou pipe ?
	printf("Error : write fifo à implementer - offset %x (=> i = %d) val %lx\n", offset, i, val);

	usb->tx_count[i] += size;
}

static void atlantronic_usb_write(void *opaque, target_phys_addr_t offset, uint64_t val, unsigned size)
{
	struct atlantronic_usb_state* usb = opaque;
	if( /*offset >= USB_OTG_CORE_GLOBAL_REGS_OFFSET &&*/ offset < USB_OTG_CORE_GLOBAL_REGS_OFFSET + sizeof(USB_OTG_GREGS))
	{
		atlantronic_usb_write_gregs(usb, offset - USB_OTG_CORE_GLOBAL_REGS_OFFSET, val, size);
	}
	else if(offset >= USB_OTG_DEV_GLOBAL_REG_OFFSET && offset < USB_OTG_DEV_GLOBAL_REG_OFFSET + sizeof(USB_OTG_DEV))
	{
		atlantronic_usb_write_dev(usb, offset - USB_OTG_DEV_GLOBAL_REG_OFFSET, val, size);
	}
	else if(offset >= USB_OTG_DEV_IN_EP_REG_OFFSET && offset < USB_OTG_DEV_IN_EP_REG_OFFSET + sizeof(USB_OTG_DINEPS) * NUM_TX_FIFOS)
	{
		atlantronic_usb_write_dineps(usb, offset - USB_OTG_DEV_IN_EP_REG_OFFSET, val, size);
	}
	else if(offset >= USB_OTG_DEV_OUT_EP_REG_OFFSET && offset < USB_OTG_DEV_OUT_EP_REG_OFFSET + sizeof(USB_OTG_DOUTEPS) * NUM_TX_FIFOS)
	{
		atlantronic_usb_write_douteps(usb, offset - USB_OTG_DEV_OUT_EP_REG_OFFSET, val, size);
	}
	else if(offset >= USB_OTG_DATA_FIFO_OFFSET && offset < USB_OTG_DATA_FIFO_OFFSET + NUM_TX_FIFOS * USB_OTG_DATA_FIFO_SIZE)
	{
		atlantronic_usb_write_fifo(usb, offset - USB_OTG_DATA_FIFO_OFFSET, val, size);
	}
	else if( offset == USB_OTG_PCGCCTL_OFFSET)
	{
		usb->pcgcctl = val;
	}
	else
	{
		printf("Error : USB forbiden write acces offset %x\n", offset);
	}
}

static uint64_t atlantronic_usb_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
	uint64_t val = 0;

	struct atlantronic_usb_state* usb = opaque;
	if( /*offset >= USB_OTG_CORE_GLOBAL_REGS_OFFSET &&*/ offset < USB_OTG_CORE_GLOBAL_REGS_OFFSET + sizeof(USB_OTG_GREGS))
	{
		val = atlantronic_usb_read_gregs(usb, offset - USB_OTG_CORE_GLOBAL_REGS_OFFSET, size);
	}
	else if(offset >= USB_OTG_DEV_GLOBAL_REG_OFFSET && offset < USB_OTG_DEV_GLOBAL_REG_OFFSET + sizeof(USB_OTG_DEV))
	{
		val = atlantronic_usb_read_dev(usb, offset - USB_OTG_DEV_GLOBAL_REG_OFFSET, size);
	}
	else if(offset >= USB_OTG_DEV_IN_EP_REG_OFFSET && offset < USB_OTG_DEV_IN_EP_REG_OFFSET + sizeof(USB_OTG_DINEPS) * NUM_TX_FIFOS)
	{
		val = atlantronic_usb_read_dineps(usb, offset - USB_OTG_DEV_IN_EP_REG_OFFSET, size);
	}
	else if(offset >= USB_OTG_DEV_OUT_EP_REG_OFFSET && offset < USB_OTG_DEV_OUT_EP_REG_OFFSET + sizeof(USB_OTG_DOUTEPS) * NUM_TX_FIFOS)
	{
		val = atlantronic_usb_read_douteps(usb, offset - USB_OTG_DEV_OUT_EP_REG_OFFSET, size);
	}
	else if(offset >= USB_OTG_DATA_FIFO_OFFSET && offset < USB_OTG_DATA_FIFO_OFFSET + NUM_TX_FIFOS * USB_OTG_DATA_FIFO_SIZE)
	{
		val = atlantronic_usb_read_fifo(usb, offset - USB_OTG_DATA_FIFO_OFFSET, size);
	}
	else if( offset == USB_OTG_PCGCCTL_OFFSET)
	{
		val = usb->pcgcctl;
	}
	else
	{
		printf("Error : USB forbiden read acces offset %x\n", offset);
	}

	return val;
}

static const MemoryRegionOps atlantronic_usb_ops =
{
	.read = atlantronic_usb_read,
	.write = atlantronic_usb_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static int atlantronic_usb_init(SysBusDevice * dev)
{
    struct atlantronic_usb_state *s = FROM_SYSBUS(struct atlantronic_usb_state, dev);

	memory_region_init_io(&s->iomem, &atlantronic_usb_ops, s, "atlantronic_usb", 0x10000);
	sysbus_init_mmio(dev, &s->iomem);
	sysbus_init_irq(dev, &s->irq);

	atlantronic_usb_reset(s);

    return 0;
}

static void atlantronic_usb_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_usb_init;
}

static TypeInfo atlantronic_usb_info =
{
	.name          = "atlantronic-usb",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_usb_state),
	.class_init    = atlantronic_usb_class_init,
};

static void atlantronic_usb_register_types(void)
{
	type_register_static(&atlantronic_usb_info);
}

type_init(atlantronic_usb_register_types);

////////////////// FIN USB
static void atlantronic_robot_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
	configure_icount("0");
	system_clock_scale = 1; 

	MemoryRegion *address_space_mem = get_system_memory();

	const int flash_size = 256;
	const int sram_size = 64;
	qemu_irq* pic = armv7m_init(address_space_mem, flash_size, sram_size, kernel_filename, "cortex-m3");

	// rcc
	sysbus_create_simple("atlantronic-rcc", RCC_BASE, NULL);

	// usb
	DeviceState* usbDev = sysbus_create_simple("atlantronic-usb", USB_OTG_FS_BASE_ADDR, NULL);
	sysbus_connect_irq(sysbus_from_qdev(usbDev), 0, pic[OTG_FS_IRQn]); 
}

static QEMUMachine atlantronic_foo =
{
    .name = "atlantronic-foo",
    .desc = "Robot d'Atlantronic - carte foo",
    .init = atlantronic_robot_init,
};

static void atlantronic_init(void)
{
    qemu_register_machine(&atlantronic_foo);
}

machine_init(atlantronic_init);
