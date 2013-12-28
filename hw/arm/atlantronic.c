#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "exec/address-spaces.h"

#include "atlantronic_model.h"
#include "atlantronic_can.h"

#include "atlantronic_cpu.h"

#define USB_OTG_FS_BASE_ADDR      0x50000000

static void atlantronic_init(QEMUMachineInitArgs *args)
{
	configure_icount("0");
	system_clock_scale = 1;

	MemoryRegion *address_space_mem = get_system_memory();

	const int flash_size = 1024;
	const int sram_size = 192;
	qemu_irq* pic = armv7m_init(address_space_mem, flash_size, sram_size, args->kernel_filename, "cortex-m4");

	// rcc
	sysbus_create_simple("atlantronic-rcc", RCC_BASE, NULL);

	// gpio
	/*DeviceState* gpioa = */sysbus_create_simple("atlantronic-gpio", GPIOA_BASE, NULL);
	/*DeviceState* gpiob = */sysbus_create_simple("atlantronic-gpio", GPIOB_BASE, NULL);
	/*DeviceState* gpioc = */sysbus_create_simple("atlantronic-gpio", GPIOC_BASE, NULL);
	sysbus_create_simple("atlantronic-gpio", GPIOD_BASE, NULL);
	/*DeviceState* gpioe = */sysbus_create_simple("atlantronic-gpio", GPIOE_BASE, NULL);
	sysbus_create_simple("atlantronic-gpio", GPIOF_BASE, NULL);

	// tim
	/*DeviceState* tim1 = */sysbus_create_simple("atlantronic-tim", TIM1_BASE, NULL);
	/*DeviceState* tim2 = */sysbus_create_simple("atlantronic-tim", TIM2_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM3_BASE, NULL);
	/*DeviceState* tim4 = */sysbus_create_simple("atlantronic-tim", TIM4_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM5_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM6_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM7_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM8_BASE, NULL);

	// dma
	DeviceState* dma1 = sysbus_create_simple("atlantronic-dma", DMA1_BASE, NULL);
	DeviceState* dma1_stream0 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream0_BASE, NULL);
	DeviceState* dma1_stream1 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream1_BASE, NULL);
	DeviceState* dma1_stream2 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream2_BASE, NULL);
	DeviceState* dma1_stream3 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream3_BASE, NULL);
	DeviceState* dma1_stream4 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream4_BASE, NULL);
	DeviceState* dma1_stream5 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream5_BASE, NULL);
	DeviceState* dma1_stream6 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream6_BASE, NULL);
	DeviceState* dma1_stream7 = sysbus_create_simple("atlantronic-dma-stream", DMA1_Stream7_BASE, NULL);
	qdev_connect_gpio_out(dma1_stream0, 0, qdev_get_gpio_in(dma1, 0));  // dma1_stream0 -> dma1
	qdev_connect_gpio_out(dma1_stream1, 0, qdev_get_gpio_in(dma1, 1));  // dma1_stream1 -> dma1
	qdev_connect_gpio_out(dma1_stream2, 0, qdev_get_gpio_in(dma1, 2));  // dma1_stream2 -> dma1
	qdev_connect_gpio_out(dma1_stream3, 0, qdev_get_gpio_in(dma1, 3));  // dma1_stream3 -> dma1
	qdev_connect_gpio_out(dma1_stream4, 0, qdev_get_gpio_in(dma1, 4));  // dma1_stream4 -> dma1
	qdev_connect_gpio_out(dma1_stream5, 0, qdev_get_gpio_in(dma1, 5));  // dma1_stream5 -> dma1
	qdev_connect_gpio_out(dma1_stream6, 0, qdev_get_gpio_in(dma1, 6));  // dma1_stream6 -> dma1
	qdev_connect_gpio_out(dma1_stream7, 0, qdev_get_gpio_in(dma1, 7));  // dma1_stream7 -> dma1
	qdev_connect_gpio_out(dma1, 0, pic[DMA1_Stream0_IRQn]);
	qdev_connect_gpio_out(dma1, 1, pic[DMA1_Stream1_IRQn]);
	qdev_connect_gpio_out(dma1, 2, pic[DMA1_Stream2_IRQn]);
	qdev_connect_gpio_out(dma1, 3, pic[DMA1_Stream3_IRQn]);
	qdev_connect_gpio_out(dma1, 4, pic[DMA1_Stream4_IRQn]);
	qdev_connect_gpio_out(dma1, 5, pic[DMA1_Stream5_IRQn]);
	qdev_connect_gpio_out(dma1, 6, pic[DMA1_Stream6_IRQn]);
	qdev_connect_gpio_out(dma1, 7, pic[DMA1_Stream7_IRQn]);

	DeviceState* dma2 = sysbus_create_simple("atlantronic-dma", DMA2_BASE, NULL);
	DeviceState* dma2_stream0 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream0_BASE, NULL);
	DeviceState* dma2_stream1 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream1_BASE, NULL);
	DeviceState* dma2_stream2 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream2_BASE, NULL);
	DeviceState* dma2_stream3 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream3_BASE, NULL);
	DeviceState* dma2_stream4 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream4_BASE, NULL);
	DeviceState* dma2_stream5 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream5_BASE, NULL);
	DeviceState* dma2_stream6 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream6_BASE, NULL);
	DeviceState* dma2_stream7 = sysbus_create_simple("atlantronic-dma-stream", DMA2_Stream7_BASE, NULL);
	qdev_connect_gpio_out(dma2_stream0, 0, qdev_get_gpio_in(dma2, 0));  // dma2_stream0 -> dma2
	qdev_connect_gpio_out(dma2_stream1, 0, qdev_get_gpio_in(dma2, 1));  // dma2_stream1 -> dma2
	qdev_connect_gpio_out(dma2_stream2, 0, qdev_get_gpio_in(dma2, 2));  // dma2_stream2 -> dma2
	qdev_connect_gpio_out(dma2_stream3, 0, qdev_get_gpio_in(dma2, 3));  // dma2_stream3 -> dma2
	qdev_connect_gpio_out(dma2_stream4, 0, qdev_get_gpio_in(dma2, 4));  // dma2_stream4 -> dma2
	qdev_connect_gpio_out(dma2_stream5, 0, qdev_get_gpio_in(dma2, 5));  // dma2_stream5 -> dma2
	qdev_connect_gpio_out(dma2_stream6, 0, qdev_get_gpio_in(dma2, 6));  // dma2_stream6 -> dma2
	qdev_connect_gpio_out(dma2_stream7, 0, qdev_get_gpio_in(dma2, 7));  // dma2_stream7 -> dma2
	qdev_connect_gpio_out(dma2, 0, pic[DMA2_Stream0_IRQn]);
	qdev_connect_gpio_out(dma2, 1, pic[DMA2_Stream1_IRQn]);
	qdev_connect_gpio_out(dma2, 2, pic[DMA2_Stream2_IRQn]);
	qdev_connect_gpio_out(dma2, 3, pic[DMA2_Stream3_IRQn]);
	qdev_connect_gpio_out(dma2, 4, pic[DMA2_Stream4_IRQn]);
	qdev_connect_gpio_out(dma2, 5, pic[DMA2_Stream5_IRQn]);
	qdev_connect_gpio_out(dma2, 6, pic[DMA2_Stream6_IRQn]);
	qdev_connect_gpio_out(dma2, 7, pic[DMA2_Stream7_IRQn]);
#if 0
	// adc
	DeviceState* adc1 = sysbus_create_simple("atlantronic-adc", ADC1_BASE, NULL);
	sysbus_create_simple("atlantronic-adc", ADC2_BASE, NULL);
	qdev_connect_gpio_out(gpioa, 0, qdev_get_gpio_in(adc1, 0));   // AN0  = PA0
	qdev_connect_gpio_out(gpioa, 1, qdev_get_gpio_in(adc1, 1));   // AN1  = PA1
	qdev_connect_gpio_out(gpioa, 2, qdev_get_gpio_in(adc1, 2));   // AN2  = PA2
	qdev_connect_gpio_out(gpioa, 3, qdev_get_gpio_in(adc1, 3));   // AN3  = PA3
	qdev_connect_gpio_out(gpioa, 4, qdev_get_gpio_in(adc1, 4));   // AN4  = PA4
	qdev_connect_gpio_out(gpioa, 5, qdev_get_gpio_in(adc1, 5));   // AN5  = PA5
	qdev_connect_gpio_out(gpioa, 6, qdev_get_gpio_in(adc1, 6));   // AN6  = PA6
	qdev_connect_gpio_out(gpioa, 7, qdev_get_gpio_in(adc1, 7));   // AN7  = PA7
	qdev_connect_gpio_out(gpiob, 0, qdev_get_gpio_in(adc1, 8));   // AN8  = PB0
	qdev_connect_gpio_out(gpiob, 1, qdev_get_gpio_in(adc1, 9));   // AN9  = PB1
	qdev_connect_gpio_out(gpioc, 0, qdev_get_gpio_in(adc1, 10));  // AN10 = PC0
	qdev_connect_gpio_out(gpioc, 1, qdev_get_gpio_in(adc1, 11));  // AN11 = PC1
	qdev_connect_gpio_out(gpioc, 2, qdev_get_gpio_in(adc1, 12));  // AN12 = PC2
	qdev_connect_gpio_out(gpioc, 3, qdev_get_gpio_in(adc1, 13));  // AN13 = PC3
	qdev_connect_gpio_out(gpioc, 4, qdev_get_gpio_in(adc1, 14));  // AN14 = PC4
	qdev_connect_gpio_out(gpioc, 5, qdev_get_gpio_in(adc1, 15));  // AN15 = PC5
	qdev_connect_gpio_out(adc1, 0, qdev_get_gpio_in(dma1_chan1, 0));  // adc1 -> dma1_chan1
#endif
	// usart
	DeviceState* usart2 = sysbus_create_simple("atlantronic-usart", USART2_BASE, NULL);
	DeviceState* usart3 = sysbus_create_simple("atlantronic-usart", USART3_BASE, NULL);
	DeviceState* uart4 = sysbus_create_simple("atlantronic-usart", UART4_BASE, NULL);
	DeviceState* uart5 = sysbus_create_simple("atlantronic-usart", UART5_BASE, NULL);
	DeviceState* usart6 = sysbus_create_simple("atlantronic-usart", USART6_BASE, NULL);

	qdev_connect_gpio_out(usart2, 0, pic[USART2_IRQn]);  // usart2 -> it hw
	qdev_connect_gpio_out(usart2, 1, qdev_get_gpio_in(dma1_stream5, 0));  // usart2 -> dma1_stream5 (rx)
	qdev_connect_gpio_out(usart3, 0, pic[USART3_IRQn]);  // usart3 -> it hw
	qdev_connect_gpio_out(usart3, 1, qdev_get_gpio_in(dma1_stream1, 0));  // usart3 -> dma1_stream1 (rx)
	qdev_connect_gpio_out(uart4, 0, pic[UART4_IRQn]);  // uart4 -> it hw
	qdev_connect_gpio_out(uart4, 1, qdev_get_gpio_in(dma1_stream2, 0));  // uart4 -> dma1_stream2 (rx)
	qdev_connect_gpio_out(uart5, 0, pic[UART5_IRQn]);  // uart5 -> it hw
	qdev_connect_gpio_out(uart5, 1, qdev_get_gpio_in(dma1_stream0, 0));  // uart5 -> dma1_stream0 (rx)
	qdev_connect_gpio_out(usart6, 0, pic[USART6_IRQn]);  // usart6 -> it hw
	qdev_connect_gpio_out(usart6, 1, qdev_get_gpio_in(dma2_stream1, 0));  // usart6 -> dma2_stream1 (rx)

	// can
	DeviceState* can1 = sysbus_create_simple("atlantronic-can", CAN1_BASE, NULL);
	qdev_connect_gpio_out(can1, ATLANTRONIC_CAN_IRQ_OUT_TX0, pic[CAN1_TX_IRQn]);  // can1 -> it hw tx
	qdev_connect_gpio_out(can1, ATLANTRONIC_CAN_IRQ_OUT_RX0, pic[CAN1_RX0_IRQn]);  // can1 -> it hw rx0

	// usb
	DeviceState* usb = sysbus_create_simple("atlantronic-usb", USB_OTG_FS_BASE_ADDR, NULL);
	sysbus_connect_irq(SYS_BUS_DEVICE(usb), 0, pic[OTG_FS_IRQn]);

	// modele
	DeviceState* model = sysbus_create_simple("atlantronic-model", 0, NULL);
	qdev_connect_gpio_out(usart6, 3, qdev_get_gpio_in(model, MODEL_IRQ_IN_USART_AX12));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_USART_AX12, qdev_get_gpio_in(usart6, 0));
	qdev_connect_gpio_out(uart4, 3, qdev_get_gpio_in(model, MODEL_IRQ_IN_USART_HOKUYO1));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_USART_HOKUYO1, qdev_get_gpio_in(uart4, 0));
	qdev_connect_gpio_out(usart2, 3, qdev_get_gpio_in(model, MODEL_IRQ_IN_USART_HOKUYO2));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_USART_HOKUYO2, qdev_get_gpio_in(usart2, 0));
	qdev_connect_gpio_out(can1, ATLANTRONIC_CAN_IRQ_OUT_CAN1_MSG_ID, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_ID));
	qdev_connect_gpio_out(can1, ATLANTRONIC_CAN_IRQ_OUT_CAN1_MSG_SIZE, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_SIZE));
	qdev_connect_gpio_out(can1, ATLANTRONIC_CAN_IRQ_OUT_CAN1_MSG_DATA_L, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_DATA_L));
	qdev_connect_gpio_out(can1, ATLANTRONIC_CAN_IRQ_OUT_CAN1_MSG_DATA_H, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_DATA_H));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_ID, qdev_get_gpio_in(can1, ATLANTRONIC_CAN_IRQ_IN_CAN1_MSG_ID));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_SIZE, qdev_get_gpio_in(can1, ATLANTRONIC_CAN_IRQ_IN_CAN1_MSG_SIZE));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_DATA_L, qdev_get_gpio_in(can1, ATLANTRONIC_CAN_IRQ_IN_CAN1_MSG_DATA_L));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_DATA_H, qdev_get_gpio_in(can1, ATLANTRONIC_CAN_IRQ_IN_CAN1_MSG_DATA_H));

#if 0
	qdev_connect_gpio_out(tim2, 0, qdev_get_gpio_in(model, 0)); // encodeur 1 (droite)
	qdev_connect_gpio_out(tim4, 0, qdev_get_gpio_in(model, 1)); // encodeur 2 (gauche)
	qdev_connect_gpio_out(tim1, 1, qdev_get_gpio_in(model, 2)); // pwm 1 (droite)
	qdev_connect_gpio_out(tim1, 2, qdev_get_gpio_in(model, 3)); // pwm 2 (gauche)
	qdev_connect_gpio_out(tim1, 3, qdev_get_gpio_in(model, 4)); // pwm 3
	qdev_connect_gpio_out(tim1, 4, qdev_get_gpio_in(model, 5)); // pwm 4
	qdev_connect_gpio_out(gpioe, 8, qdev_get_gpio_in(model, 6)); // direction pwm 1 (droite)
	qdev_connect_gpio_out(gpioe, 10, qdev_get_gpio_in(model, 7)); // direction pwm 2 (gauche)
	qdev_connect_gpio_out(gpioe, 12, qdev_get_gpio_in(model, 8)); // direction pwm 3
	qdev_connect_gpio_out(gpioe, 15, qdev_get_gpio_in(model, 9)); // direction pwm 4

	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_ENCODER_RIGHT, qdev_get_gpio_in(tim2, 0));  // encodeur 1 (droite)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_ENCODER_LEFT, qdev_get_gpio_in(tim4, 0));   // encodeur 2 (gauche)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_RIGHT, qdev_get_gpio_in(gpioc, 5));  // intensite moteur 1 (droite)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_LEFT, qdev_get_gpio_in(gpioa, 5));  // intensite moteur 2 (gauche)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_MOT3, qdev_get_gpio_in(gpioc, 3));  // intensite moteur 3
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_MOT4, qdev_get_gpio_in(gpioc, 1));  // intensite moteur 4
#endif
}

static QEMUMachine atlantronic_discovery =
{
    .name = "atlantronic",
    .desc = "Robot d'Atlantronic - carte discovery",
    .init = atlantronic_init,
};

static void atlantronic_init_discovery(void)
{
    qemu_register_machine(&atlantronic_discovery);
}

machine_init(atlantronic_init_discovery);

