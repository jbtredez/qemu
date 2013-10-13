#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "qemu/thread.h"
#include "sysemu/char.h"
#include "atlantronic_cpu.h"
#include "kernel/driver/usb/stm32f4xx/usb_regs.h"
#include "kernel/driver/usb/stm32f4xx/usbd_def.h"

#define EVENT_DATA_SIZE             128
#define EVENT_NUM                  1024
#define NUM_TX_FIFOS                  4
#define USB_OTG_DATA_FIFO_SIZE   0x1000

enum atlantronic_event_type
{
	ATLANTRONIC_EVENT_SETUP,
	ATLANTRONIC_EVENT_DATA
};

struct atlantronic_usb_rx_event
{
	uint32_t type;                     //!< type :  setup ou data
	uint32_t ep;                       //!< numero d'endpoint
	uint32_t length;                   //!< taille des données
	union
	{
		uint8_t data[EVENT_DATA_SIZE];     //!< données
		struct
		{
			uint8_t setup_request_type;
			uint8_t setup_request;
			uint16_t setup_wvalue;
			uint16_t setup_windex;
			uint16_t setup_length;
		};
	};
};

struct atlantronic_usb_state
{
	SysBusDevice busdev;
	CharDriverState *chr;
	MemoryRegion iomem;
	USB_OTG_GREGS gregs;
	USB_OTG_DREGS dev;
	USB_OTG_INEPREGS dineps[NUM_TX_FIFOS];
	USB_OTG_OUTEPREGS douteps[NUM_TX_FIFOS];
	uint32_t fifo[USB_OTG_DATA_FIFO_SIZE/4 * NUM_TX_FIFOS];
	int32_t fifo_start[NUM_TX_FIFOS];
	int32_t fifo_end[NUM_TX_FIFOS];
	uint32_t pcgcctl;
	QemuThread rx_thread_id;
	struct atlantronic_usb_rx_event event[EVENT_NUM];
	uint32_t event_start;
	uint32_t event_end;
	QemuMutex event_mutex;
	QemuCond event_cond;
	QemuCond xfer_complete_cond;
	QemuCond setup_complete_cond;
	QemuMutex xfer_complete_mutex;

	// fifos d'envoi
	int32_t tx_count[NUM_TX_FIFOS];
	qemu_irq irq;
};

static int32_t atlantronic_usb_write_rx_fifo0_data(struct atlantronic_usb_state *usb, const uint8_t* data, uint16_t length);

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
	usb->gregs.DIEPTXF0_HNPTXFSIZ = 0x00000200;
	usb->gregs.HNPTXSTS = 0x00080200;
	usb->gregs.GI2CCTL = 0x00;
	usb->gregs.GCCFG = 0x00;
	usb->gregs.CID = 0x00001000;
	usb->gregs.HPTXFSIZ = 0x02000600;

	for(i = 0; i < USB_OTG_MAX_TX_FIFOS; i++)
	{
		usb->gregs.DIEPTXF[i] = 0x02000400;
	}

	usb->dev.DCFG = 0x02200000;
	usb->dev.DCTL = 0x00;
	usb->dev.DSTS = 0x00000010;
	usb->dev.DIEPMSK = 0x00;
	usb->dev.DOEPMSK = 0x00;
	usb->dev.DAINT = 0x00;
	usb->dev.DAINTMSK = 0x00;
	usb->dev.DVBUSDIS = 0x000017D7;
	usb->dev.DVBUSPULSE = 0x000005B8;
	usb->dev.DTHRCTL = 0x00;
	usb->dev.DIEPEMPMSK = 0x00;
	usb->dev.DEACHINT = 0x00;
	usb->dev.DEACHMSK = 0x00;
	usb->dev.DINEP1MSK = 0x00;
	usb->dev.DOUTEP1MSK = 0x00;

	for(i = 0; i < NUM_TX_FIFOS; i++)
	{
		usb->dineps[i].DIEPCTL = 0x00;
		usb->dineps[i].DIEPINT = 0x00000080;
		usb->dineps[i].DIEPTSIZ = 0x00;
		usb->dineps[i].DIEPDMA = 0x00;
		usb->dineps[i].DTXFSTS = 0x00;

		usb->douteps[i].DOEPCTL = 0x00008000;
		usb->douteps[i].DOUTEPFRM = 0x00;
		usb->douteps[i].DOEPINT = 0x00000080;
		usb->douteps[i].DOEPTSIZ = 0x00;
		usb->douteps[i].DOEPDMA = 0x00;

		usb->fifo_start[i] = 0;
		usb->fifo_end[i] = 0;

		usb->tx_count[i] = 0;
	}

	usb->pcgcctl = 0x00;
}

static void* atlantronic_usb_rx_thread(void* arg)
{
	struct atlantronic_usb_state* usb = arg;
	struct atlantronic_usb_rx_event ev;

	while(1)
	{
		qemu_mutex_lock(&usb->event_mutex);

		if(usb->event_start == usb->event_end)
		{
			qemu_cond_wait(&usb->event_cond, &usb->event_mutex);
		}
		ev = usb->event[usb->event_start];
		usb->event_start = (usb->event_start + 1) % EVENT_NUM;
		qemu_mutex_unlock(&usb->event_mutex);

		USB_OTG_GRXFSTS_TypeDef grxstsp;
		grxstsp.d32 = usb->gregs.GRXSTSP;
		if(ev.type == ATLANTRONIC_EVENT_DATA)
		{
			grxstsp.b.pktsts = 2;
		}

		atlantronic_usb_write_rx_fifo0_data(usb, ev.data, ev.length);
		grxstsp.b.chnum = ev.ep;
		grxstsp.b.bcnt = ev.length;

		usb->gregs.GRXSTSP = grxstsp.d32;

		qemu_mutex_lock(&usb->xfer_complete_mutex);
		usb->gregs.GINTSTS |= 0x10;          // IT RX fifo
		usb->gregs.GINTSTS |= 0x80000;       // IT out EP
		qemu_set_irq(usb->irq, 1);
		qemu_cond_wait(&usb->xfer_complete_cond, &usb->xfer_complete_mutex);

		if(ev.type == ATLANTRONIC_EVENT_SETUP && ev.ep == 0)
		{
			grxstsp.d32 = usb->gregs.GRXSTSP;
			grxstsp.b.pktsts = 6;  // SETUP data packet received (PKTSTS)
			usb->gregs.GRXSTSP = grxstsp.d32;
			// setup done
			usb->douteps[0].DOEPINT = 0x08;    // EP0 -> STUP (SETUP phase done)
			usb->gregs.GINTSTS |= 0x80000;     // IT out EP
			qemu_set_irq(usb->irq, 1);
			qemu_cond_wait(&usb->setup_complete_cond, &usb->xfer_complete_mutex);
		}

		qemu_mutex_unlock(&usb->xfer_complete_mutex);
	}

	return 0;
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

static int32_t atlantronic_usb_write_rx_fifo0_data(struct atlantronic_usb_state *usb, const uint8_t* data, uint16_t length)
{
	int i = 0;
	int count = length >> 2;
	int max = 4 * count;
	for(i = 0; i < max; i += 4)
	{
		atlantronic_usb_write32_fifo(usb, 0, *((uint32_t*) (data + i)));
	}

	switch(length - i)
	{
		case 1:
			count++;
			atlantronic_usb_write32_fifo(usb, 0, data[i]);
			break;
		case 2:
			count++;
			atlantronic_usb_write32_fifo(usb, 0, data[i] + (data[i+1] << 8) );
			break;
		case 3:
			count++;
			atlantronic_usb_write32_fifo(usb, 0, data[i] + (data[i+1] << 8) + (data[i+2] << 16));
			break;
	}

	return count;
}

static void atlantronic_usb_write_gregs(struct atlantronic_usb_state *usb, hwaddr offset, uint64_t val, unsigned size)
{
	switch(offset)
	{
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GOTGCTL, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GOTGINT, val);
		case offsetof(USB_OTG_GREGS, GAHBCFG):
			usb->gregs.GAHBCFG = val;
			if( val & 0x01)
			{
				usb->gregs.GINTSTS |= 0x1000; // reset
				qemu_set_irq(usb->irq, 1);
			}
			break;
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GUSBCFG, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GRSTCTL, val);
		case offsetof(USB_OTG_GREGS, GINTSTS):
			if( (val & 0x1000) && (usb->gregs.GINTSTS & 0x1000)) // clear bit d'IT reset
			{
				qemu_mutex_lock(&usb->event_mutex);
				if((usb->event_end + 1) % EVENT_NUM != usb->event_start)
				{
					//printf("[QEMU] set address 1\n");
					// il reste de la place
					usb->event[usb->event_end].type = ATLANTRONIC_EVENT_SETUP;
					usb->event[usb->event_end].ep = 0;
					usb->event[usb->event_end].length = 8;
					usb->event[usb->event_end].setup_request_type = 0x00;
					usb->event[usb->event_end].setup_request = USB_REQ_SET_ADDRESS;
					usb->event[usb->event_end].setup_wvalue = 0x0001;
					usb->event[usb->event_end].setup_windex = 0x0000;
					usb->event[usb->event_end].setup_length = 0x0000;
					usb->event_end = (usb->event_end + 1) % EVENT_NUM;
				}
				qemu_cond_signal(&usb->event_cond);
				qemu_mutex_unlock(&usb->event_mutex);
			}
			else
			{
				qemu_set_irq(usb->irq, 0);
			}
			usb->gregs.GINTSTS &= ~(usb->gregs.GINTSTS & val);
			break;
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GINTMSK, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GRXSTSR, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GRXSTSP, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GRXFSIZ, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF0_HNPTXFSIZ, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, HNPTXSTS, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GI2CCTL, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, GCCFG, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, CID, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, HPTXFSIZ, val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF[0], val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF[1], val);
		W_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF[2], val);
		default:
			printf("Error : USB forbiden gregs write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_usb_read_gregs(struct atlantronic_usb_state *usb, hwaddr offset, unsigned size)
{
	uint64_t val = 0;

	switch(offset)
	{
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GOTGCTL, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GOTGINT, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GAHBCFG, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GUSBCFG, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GRSTCTL, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GINTSTS, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GINTMSK, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GRXSTSR, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GRXSTSP, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GRXFSIZ, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF0_HNPTXFSIZ, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, HNPTXSTS, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GI2CCTL, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, GCCFG, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, CID, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, HPTXFSIZ, val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF[0], val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF[1], val);
		R_ACCESS(USB_OTG_GREGS, usb->gregs, DIEPTXF[2], val);
		default:
			printf("Error : USB forbiden gregs read acces offset %lx\n", offset);
			break;
	}

	return val;
}

static void atlantronic_usb_write_dev(struct atlantronic_usb_state *usb, hwaddr offset, uint64_t val, unsigned size)
{
	switch(offset)
	{
		case offsetof(USB_OTG_DREGS, DCFG):
			{
				USB_OTG_DCFG_TypeDef new_dcfg;
				USB_OTG_DCFG_TypeDef old_dcfg;
				new_dcfg.d32 = val;
				old_dcfg.d32 = usb->dev.DCFG;
				if(old_dcfg.b.devaddr == 0 && new_dcfg.b.devaddr != 0)
				{
					//printf("[QEMU] usb addressed (%d)\n", new_dcfg.b.devaddr);
					qemu_mutex_lock(&usb->event_mutex);

					if((usb->event_end + 1) % EVENT_NUM != usb->event_start)
					{
						// il reste de la place
						//printf("[QEMU] usb set configuration\n");
						usb->event[usb->event_end].type = ATLANTRONIC_EVENT_SETUP;
						usb->event[usb->event_end].ep = 0;
						usb->event[usb->event_end].length = 8;
						usb->event[usb->event_end].setup_request_type = 0x00;
						usb->event[usb->event_end].setup_request = USB_REQ_SET_CONFIGURATION;
						usb->event[usb->event_end].setup_wvalue = 0x0001;
						usb->event[usb->event_end].setup_windex = 0x0000;
						usb->event[usb->event_end].setup_length = 0x0000;
						usb->event_end = (usb->event_end + 1) % EVENT_NUM;
					}

					qemu_cond_signal(&usb->event_cond);
					qemu_mutex_unlock(&usb->event_mutex);
				}
				usb->dev.DCFG = val;
			}
			break;
		W_ACCESS(USB_OTG_DREGS, usb->dev, DCTL, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DSTS, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DIEPMSK, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DOEPMSK, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DAINT, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DAINTMSK, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DVBUSDIS, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DVBUSPULSE, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DTHRCTL, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DIEPEMPMSK, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DEACHINT, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DEACHMSK, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DINEP1MSK, val);
		W_ACCESS(USB_OTG_DREGS, usb->dev, DOUTEP1MSK, val);
		default:
			printf("Error : USB forbiden dev write acces offset %lx\n", offset);
			break;
	}
}

static uint64_t atlantronic_usb_read_dev(struct atlantronic_usb_state *usb, hwaddr offset, unsigned size)
{
	uint64_t val = 0;

	switch(offset)
	{
		R_ACCESS(USB_OTG_DREGS, usb->dev, DCFG, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DCTL, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DSTS, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DIEPMSK, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DOEPMSK, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DAINT, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DAINTMSK, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DVBUSDIS, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DVBUSPULSE, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DTHRCTL, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DIEPEMPMSK, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DEACHINT, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DEACHMSK, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DINEP1MSK, val);
		R_ACCESS(USB_OTG_DREGS, usb->dev, DOUTEP1MSK, val);
		default:
			printf("Error : USB forbiden dev read acces offset %lx\n", offset);
			break;
	}

	return val;
}

static void atlantronic_usb_write_dineps(struct atlantronic_usb_state *usb, hwaddr offset, uint64_t val, unsigned size)
{
	int i = offset / sizeof(USB_OTG_INEPREGS);

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden dineps write acces offset %lx (=> i = %d)\n", offset, i);
		return;
	}

	switch(offset % sizeof(USB_OTG_INEPREGS))
	{
		case offsetof(USB_OTG_INEPREGS, DIEPCTL):
			if( val & 80000000 && i > 0) // EPNA et endpoint != 0
			{
				int xfrsiz = usb->dineps[i].DIEPTSIZ & 0x7ffff;
				if(xfrsiz > 0)
				{
					usb->dineps[i].DIEPINT = 0x80; // txfempty et pas d'autres flags
					usb->dineps[i].DTXFSTS = USB_OTG_DATA_FIFO_SIZE/4; // taille libre de la fifo
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
			usb->dineps[i].DIEPCTL = val;
			break;
		case offsetof(USB_OTG_INEPREGS, DIEPINT):
			usb->dineps[i].DIEPINT = val;
			if(val == 0x80)
			{
				// Une fois le message lu, on met le flag xfrc (transfert terminé)
				int xfrsiz = usb->dineps[i].DIEPTSIZ & 0x7ffff;
				if(xfrsiz <= usb->tx_count[i])
				{
					usb->tx_count[i] = 0;
					usb->dineps[i].DIEPINT |= 0x01; // xfrc
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
		W_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DIEPDMA, val);
		W_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DIEPTSIZ, val);
		W_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DTXFSTS, val);
		default:
			printf("Error : USB forbiden dineps write acces offset %lx\n", offset);
			break;
	}
}

static uint64_t atlantronic_usb_read_dineps(struct atlantronic_usb_state *usb, hwaddr offset, unsigned size)
{
	uint64_t val = 0;
	int i = offset / sizeof(USB_OTG_INEPREGS);

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden dineps read acces offset %lx (=> i = %d)\n", offset, i);
		return 0;
	}

	switch(offset % sizeof(USB_OTG_INEPREGS))
	{
		R_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DIEPCTL, val);
		R_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DIEPINT, val);
		R_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DIEPDMA, val);
		R_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DIEPTSIZ, val);
		R_ACCESS(USB_OTG_INEPREGS, usb->dineps[i], DTXFSTS, val);
		default:
			printf("Error : USB forbiden dineps read acces offset %lx\n", offset);
			break;
	}

	return val;
}

static void atlantronic_usb_write_douteps(struct atlantronic_usb_state *usb, hwaddr offset, uint64_t val, unsigned size)
{
	int i = offset / sizeof(USB_OTG_OUTEPREGS);
	USB_OTG_DOEPINTn_TypeDef doepint;

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden douteps write acces offset %lx (=> i = %d)\n", offset, i);
		return;
	}

	switch(offset % sizeof(USB_OTG_OUTEPREGS))
	{
		W_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOEPCTL, val);
		W_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOUTEPFRM, val);
		case offsetof(USB_OTG_OUTEPREGS, DOEPINT):
			doepint.d32 = val;
			if(val != 0xff)
			{
				if(doepint.b.xfercompl)
				{
					// flag xfercompl
					qemu_mutex_lock(&usb->xfer_complete_mutex);
					usb->gregs.GINTSTS &= ~0x10;          // IT RX fifo
					usb->gregs.GINTSTS &= ~0x80000;       // IT out EP
					qemu_set_irq(usb->irq, 0);
					qemu_cond_signal(&usb->xfer_complete_cond);
					qemu_mutex_unlock(&usb->xfer_complete_mutex);
				}
				if(doepint.b.setup)
				{
					qemu_mutex_lock(&usb->xfer_complete_mutex);
					usb->gregs.GINTSTS &= ~0x80000;       // IT out EP
					//printf("[QEMU] paquet setup ok\n");
					qemu_set_irq(usb->irq, 0);
					qemu_cond_signal(&usb->setup_complete_cond);
					qemu_mutex_unlock(&usb->xfer_complete_mutex);
					doepint.b.xfercompl = 1;
				}
			}
			usb->douteps[i].DOEPINT = doepint.d32;
			break;
		W_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOEPTSIZ, val);
		W_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOEPDMA, val);
		default:
			printf("Error : USB forbiden douteps write acces offset %lx\n", offset);
			break;
	}
}

static uint64_t atlantronic_usb_read_douteps(struct atlantronic_usb_state *usb, hwaddr offset, unsigned size)
{
	uint64_t val = 0;
	int i = offset / sizeof(USB_OTG_OUTEPREGS);

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden douteps read acces offset %lx (=> i = %d)\n", offset, i);
		return 0;
	}

	switch(offset % sizeof(USB_OTG_OUTEPREGS))
	{
		R_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOEPCTL, val);
		R_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOUTEPFRM, val);
		R_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOEPINT, val);
		R_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOEPTSIZ, val);
		R_ACCESS(USB_OTG_OUTEPREGS, usb->douteps[i], DOEPDMA, val);
		default:
			printf("Error : USB forbiden douteps read acces offset %lx\n", offset);
			break;
	}

	return val;
}

static uint64_t atlantronic_usb_read_fifo(struct atlantronic_usb_state *usb, hwaddr offset, unsigned size)
{
	int i = offset / USB_OTG_DATA_FIFO_SIZE;

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden fifo read acces offset %lx (=> i = %d)\n", offset, i);
		return 0;
	}

	// on doit lire uniquement le premier élément
	if(offset - i* USB_OTG_DATA_FIFO_SIZE != 0)
	{
		printf("Error : USB forbiden fifo read acces offset %lx (=> i = %d)\n", offset, i);
		return 0;
	}

	return atlantronic_usb_read32_fifo(usb, i);
}

static void atlantronic_usb_write_fifo(struct atlantronic_usb_state *usb, hwaddr offset, uint64_t val, unsigned size)
{
	int i = offset / USB_OTG_DATA_FIFO_SIZE;

	if(i >= NUM_TX_FIFOS)
	{
		printf("Error : USB forbiden fifo write acces offset %lx (=> i = %d)\n", offset, i);
		return;
	}

	int remain = (usb->dineps[i].DIEPTSIZ & 0x7ffff) - usb->tx_count[i];
	if(remain > 0)
	{
		if(size > remain)
		{
			size = remain;
		}

		qemu_chr_fe_write(usb->chr, (const uint8_t *)&val, size);
	}

	usb->tx_count[i] += size;
}

static void atlantronic_usb_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_usb_state* usb = opaque;

	if( /*offset >= USB_OTG_CORE_GLOBAL_REGS_OFFSET &&*/ offset < USB_OTG_CORE_GLOBAL_REGS_OFFSET + sizeof(USB_OTG_GREGS))
	{
		atlantronic_usb_write_gregs(usb, offset - USB_OTG_CORE_GLOBAL_REGS_OFFSET, val, size);
	}
	else if(offset >= USB_OTG_DEV_GLOBAL_REG_OFFSET && offset < USB_OTG_DEV_GLOBAL_REG_OFFSET + sizeof(USB_OTG_DREGS))
	{
		atlantronic_usb_write_dev(usb, offset - USB_OTG_DEV_GLOBAL_REG_OFFSET, val, size);
	}
	else if(offset >= USB_OTG_DEV_IN_EP_REG_OFFSET && offset < USB_OTG_DEV_IN_EP_REG_OFFSET + sizeof(USB_OTG_INEPREGS) * NUM_TX_FIFOS)
	{
		atlantronic_usb_write_dineps(usb, offset - USB_OTG_DEV_IN_EP_REG_OFFSET, val, size);
	}
	else if(offset >= USB_OTG_DEV_OUT_EP_REG_OFFSET && offset < USB_OTG_DEV_OUT_EP_REG_OFFSET + sizeof(USB_OTG_OUTEPREGS) * NUM_TX_FIFOS)
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
		printf("Error : USB forbiden write acces offset %lx\n", offset);
	}
}

static uint64_t atlantronic_usb_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t val = 0;
	struct atlantronic_usb_state* usb = opaque;
	if( /*offset >= USB_OTG_CORE_GLOBAL_REGS_OFFSET &&*/ offset < USB_OTG_CORE_GLOBAL_REGS_OFFSET + sizeof(USB_OTG_GREGS))
	{
		val = atlantronic_usb_read_gregs(usb, offset - USB_OTG_CORE_GLOBAL_REGS_OFFSET, size);
	}
	else if(offset >= USB_OTG_DEV_GLOBAL_REG_OFFSET && offset < USB_OTG_DEV_GLOBAL_REG_OFFSET + sizeof(USB_OTG_DREGS))
	{
		val = atlantronic_usb_read_dev(usb, offset - USB_OTG_DEV_GLOBAL_REG_OFFSET, size);
	}
	else if(offset >= USB_OTG_DEV_IN_EP_REG_OFFSET && offset < USB_OTG_DEV_IN_EP_REG_OFFSET + sizeof(USB_OTG_INEPREGS) * NUM_TX_FIFOS)
	{
		val = atlantronic_usb_read_dineps(usb, offset - USB_OTG_DEV_IN_EP_REG_OFFSET, size);
	}
	else if(offset >= USB_OTG_DEV_OUT_EP_REG_OFFSET && offset < USB_OTG_DEV_OUT_EP_REG_OFFSET + sizeof(USB_OTG_OUTEPREGS) * NUM_TX_FIFOS)
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
		printf("Error : USB forbiden read acces offset %lx\n", offset);
	}

	return val;
}

static int atlantronic_usb_can_receive(void *opaque)
{
	return EVENT_DATA_SIZE;
}

static void atlantronic_usb_receive(void *opaque, const uint8_t* buf, int size)
{
	struct atlantronic_usb_state *usb = opaque;

	qemu_mutex_lock(&usb->event_mutex);
	if((usb->event_end + 1) % EVENT_NUM != usb->event_start && size < EVENT_DATA_SIZE && size > 0)
	{
		// il reste de la place
		usb->event[usb->event_end].type = ATLANTRONIC_EVENT_DATA;
		usb->event[usb->event_end].ep = 2;
		usb->event[usb->event_end].length = size;
		memcpy(usb->event[usb->event_end].data, buf, size);
		usb->event_end = (usb->event_end + 1) % EVENT_NUM;
	}
	qemu_cond_signal(&usb->event_cond);
	qemu_mutex_unlock(&usb->event_mutex);
}

static void atlantronic_usb_event(void *opaque, int event)
{

}

static const MemoryRegionOps atlantronic_usb_ops =
{
	.read = atlantronic_usb_read,
	.write = atlantronic_usb_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static int atlantronic_usb_init(SysBusDevice * dev)
{
    struct atlantronic_usb_state *s = OBJECT_CHECK(struct atlantronic_usb_state, dev, "atlantronic-usb");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_usb_ops, s, "atlantronic_usb", 0x10000);
	sysbus_init_mmio(dev, &s->iomem);
	sysbus_init_irq(dev, &s->irq);

	s->chr = qemu_chr_find("foo_usb");
	if( s->chr == NULL)
	{
		hw_error("chardev foo_usb not found");
	}
    else
	{
		qemu_chr_add_handlers(s->chr, atlantronic_usb_can_receive, atlantronic_usb_receive, atlantronic_usb_event, s);
	}

	atlantronic_usb_reset(s);

	s->event_start = 0;
	s->event_end = 0;
	qemu_mutex_init(&s->event_mutex);
	qemu_mutex_init(&s->xfer_complete_mutex);
	qemu_cond_init(&s->event_cond);
	qemu_cond_init(&s->xfer_complete_cond);
	qemu_cond_init(&s->setup_complete_cond);
	qemu_thread_create(&s->rx_thread_id, atlantronic_usb_rx_thread, s, QEMU_THREAD_JOINABLE);

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
