#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "exec/address-spaces.h"
#include "qemu/config-file.h"
#include "qemu/option.h"

#include "atlantronic_model.h"
#include "atlantronic_can.h"
#include "atlantronic_gpio.h"
#include "atlantronic_syscfg.h"
#include "atlantronic_exti.h"
#include "atlantronic_spi.h"
#include "atlantronic_cpu.h"

#define NUM_IRQ_LINES             128

static void atlantronic_init(MachineState *args)
{
	QemuOpts *opts = qemu_opts_create(qemu_find_opts("icount"), NULL, 0, &error_abort);
	qemu_opt_set(opts, "shift", "0", &error_abort);
	configure_icount(opts, &error_abort);
	qemu_opts_del(opts);
	system_clock_scale = 1;

	MemoryRegion* address_space_mem = get_system_memory();
	MemoryRegion* flash = g_new(MemoryRegion, 1);
	MemoryRegion* flash_alias = g_new(MemoryRegion, 1);
	MemoryRegion* sram = g_new(MemoryRegion, 1);
	MemoryRegion* ext_ram = g_new(MemoryRegion, 1);

	const int flash_size = 2048*1024;
	const int sram_size = 192*1024;
	const int ext_ram_size = 64*1024*1024;

	// ajout flash
	memory_region_init_ram(flash, NULL, "atlantronic.flash", flash_size, &error_abort);
	vmstate_register_ram_global(flash);
	memory_region_set_readonly(flash, true);
	memory_region_add_subregion(address_space_mem, 0x8000000, flash);
	memory_region_init_alias(flash_alias, NULL, "atlantronic.flash_alias", flash, 0, flash_size);
	memory_region_add_subregion(address_space_mem, 0, flash_alias);

	// ajout ram
	memory_region_init_ram(sram, NULL, "atlantronic.sram", sram_size, &error_abort);
	vmstate_register_ram_global(sram);
	memory_region_add_subregion(address_space_mem, 0x20000000, sram);

	// ajout ram externe
	memory_region_init_ram(ext_ram, NULL, "atlantronic.ext_ram", ext_ram_size, &error_abort);
	memory_region_add_subregion(address_space_mem, 0xD0000000, ext_ram);

	qemu_irq* pic = armv7m_init(address_space_mem, flash_size, NUM_IRQ_LINES, args->kernel_filename, "cortex-m4");

	// rcc
	sysbus_create_simple("atlantronic-rcc", RCC_BASE, NULL);

	// gpio
	DeviceState* gpioa = sysbus_create_simple("atlantronic-gpio", GPIOA_BASE, NULL);
	DeviceState* gpiob = sysbus_create_simple("atlantronic-gpio", GPIOB_BASE, NULL);
	DeviceState* gpioc = sysbus_create_simple("atlantronic-gpio", GPIOC_BASE, NULL);
	DeviceState* gpiod = sysbus_create_simple("atlantronic-gpio", GPIOD_BASE, NULL);
	DeviceState* gpioe = sysbus_create_simple("atlantronic-gpio", GPIOE_BASE, NULL);
	DeviceState* gpiof = sysbus_create_simple("atlantronic-gpio", GPIOF_BASE, NULL);
	DeviceState* gpiog = sysbus_create_simple("atlantronic-gpio", GPIOG_BASE, NULL);

	// syscfg
	DeviceState* syscfg = sysbus_create_simple("atlantronic-syscfg", SYSCFG_BASE, NULL);
	qdev_connect_gpio_out(gpioa, GPIO_IRQ_OUT_CHANGE_PIN, qdev_get_gpio_in(syscfg, SYSCFG_IRQ_IN_GPIOA));
	qdev_connect_gpio_out(gpiob, GPIO_IRQ_OUT_CHANGE_PIN, qdev_get_gpio_in(syscfg, SYSCFG_IRQ_IN_GPIOB));
	qdev_connect_gpio_out(gpioc, GPIO_IRQ_OUT_CHANGE_PIN, qdev_get_gpio_in(syscfg, SYSCFG_IRQ_IN_GPIOC));
	qdev_connect_gpio_out(gpiod, GPIO_IRQ_OUT_CHANGE_PIN, qdev_get_gpio_in(syscfg, SYSCFG_IRQ_IN_GPIOD));
	qdev_connect_gpio_out(gpioe, GPIO_IRQ_OUT_CHANGE_PIN, qdev_get_gpio_in(syscfg, SYSCFG_IRQ_IN_GPIOE));
	qdev_connect_gpio_out(gpiof, GPIO_IRQ_OUT_CHANGE_PIN, qdev_get_gpio_in(syscfg, SYSCFG_IRQ_IN_GPIOF));
	qdev_connect_gpio_out(gpiog, GPIO_IRQ_OUT_CHANGE_PIN, qdev_get_gpio_in(syscfg, SYSCFG_IRQ_IN_GPIOG));

	// exti
	DeviceState* exti = sysbus_create_simple("atlantronic-exti", EXTI_BASE, NULL);
	qdev_connect_gpio_out(syscfg, SYSCFG_IRQ_OUT_EXTI, qdev_get_gpio_in(exti, EXTI_IRQ_IN_IO));
	qdev_connect_gpio_out(exti, 0, pic[EXTI0_IRQn]);
	qdev_connect_gpio_out(exti, 1, pic[EXTI1_IRQn]);
	qdev_connect_gpio_out(exti, 2, pic[EXTI2_IRQn]);
	qdev_connect_gpio_out(exti, 3, pic[EXTI3_IRQn]);
	qdev_connect_gpio_out(exti, 4, pic[EXTI4_IRQn]);
	qdev_connect_gpio_out(exti, 5, pic[EXTI9_5_IRQn]);
	qdev_connect_gpio_out(exti, 6, pic[EXTI15_10_IRQn]);

	// tim
	sysbus_create_simple("atlantronic-tim", TIM1_BASE, NULL);
	DeviceState* tim2 = sysbus_create_simple("atlantronic-tim", TIM2_BASE, NULL);
	DeviceState* tim3 = sysbus_create_simple("atlantronic-tim", TIM3_BASE, NULL);
	DeviceState* tim4 = sysbus_create_simple("atlantronic-tim", TIM4_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM5_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM6_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM7_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM8_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM9_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM10_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM11_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM12_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM13_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM14_BASE, NULL);

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
	qdev_connect_gpio_out(adc1, 0, qdev_get_gpio_in(dma2_stream0, 0));  // adc1 -> dma2_stream0

	// usart
	DeviceState* usart1 = sysbus_create_simple("atlantronic-usart", USART1_BASE, NULL);
	DeviceState* usart2 = sysbus_create_simple("atlantronic-usart", USART2_BASE, NULL);
	DeviceState* usart3 = sysbus_create_simple("atlantronic-usart", USART3_BASE, NULL);
	DeviceState* uart4 = sysbus_create_simple("atlantronic-usart", UART4_BASE, NULL);
	DeviceState* uart5 = sysbus_create_simple("atlantronic-usart", UART5_BASE, NULL);
	DeviceState* usart6 = sysbus_create_simple("atlantronic-usart", USART6_BASE, NULL);

	qdev_connect_gpio_out(usart1, 0, pic[USART1_IRQn]);  // usart2 -> it hw
	qdev_connect_gpio_out(usart1, 1, qdev_get_gpio_in(dma2_stream2, 0));  // usart1 -> dma2_stream2 (rx)
	qdev_connect_gpio_out(usart2, 0, pic[USART2_IRQn]);  // usart2 -> it hw
	qdev_connect_gpio_out(usart2, 1, qdev_get_gpio_in(dma1_stream5, 0));  // usart2 -> dma1_stream5 (rx)
	qdev_connect_gpio_out(usart3, 0, pic[USART3_IRQn]);  // usart3 -> it hw
	qdev_connect_gpio_out(usart3, 1, qdev_get_gpio_in(dma1_stream1, 0));  // usart3 -> dma1_stream1 (rx)
	qdev_connect_gpio_out(uart4, 0, pic[UART4_IRQn]);  // uart4 -> it hw
	qdev_connect_gpio_out(uart4, 1, qdev_get_gpio_in(dma1_stream2, 0));  // uart4 -> dma1_stream2 (rx)
	qdev_connect_gpio_out(uart5, 0, pic[UART5_IRQn]);  // uart5 -> it hw
	qdev_connect_gpio_out(uart5, 1, qdev_get_gpio_in(dma1_stream0, 0));  // uart5 -> dma1_stream0 (rx)
	qemu_set_irq(qdev_get_gpio_in(uart4, 1), 1);
	qdev_connect_gpio_out(usart6, 0, pic[USART6_IRQn]);  // usart6 -> it hw
	qdev_connect_gpio_out(usart6, 1, qdev_get_gpio_in(dma2_stream1, 0));  // usart6 -> dma2_stream1 (rx)

	// spi
	DeviceState* spi5 = sysbus_create_simple("atlantronic-spi", SPI5_BASE, NULL);
	qdev_connect_gpio_out(spi5, SPI_IRQ_OUT_HW, pic[SPI5_IRQn]);  // spi5 -> it hw
	qdev_connect_gpio_out(spi5, SPI_IRQ_OUT_DMAR, qdev_get_gpio_in(dma2_stream2, 0));  // spi5 -> dma2_stream3 (rx)
	qdev_connect_gpio_out(gpioc, 1, qdev_get_gpio_in(spi5, SPI_IRQ_IN_CS0));
	qdev_connect_gpio_out(gpioc, 2, qdev_get_gpio_in(spi5, SPI_IRQ_IN_CS1));
	qdev_connect_gpio_out(gpiof, 10, qdev_get_gpio_in(spi5, SPI_IRQ_IN_CS2));

	// can
	DeviceState* can1 = sysbus_create_simple("atlantronic-can", CAN1_BASE, NULL);
	qdev_connect_gpio_out(can1, CAN_IRQ_OUT_TX0, pic[CAN1_TX_IRQn]);  // can1 -> it hw tx
	qdev_connect_gpio_out(can1, CAN_IRQ_OUT_RX0, pic[CAN1_RX0_IRQn]);  // can1 -> it hw rx0

	// usb
	DeviceState* usb = sysbus_create_simple("atlantronic-usb", USB_OTG_HS_PERIPH_BASE, NULL);
	sysbus_connect_irq(SYS_BUS_DEVICE(usb), 0, pic[OTG_HS_IRQn]);

	// modele
	DeviceState* model = sysbus_create_simple("atlantronic-model", 0, NULL);
	qdev_connect_gpio_out(uart5, 3, qdev_get_gpio_in(model, MODEL_IRQ_IN_USART_AX12));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_USART_AX12, qdev_get_gpio_in(uart5, 0));
	qdev_connect_gpio_out(uart4, 3, qdev_get_gpio_in(model, MODEL_IRQ_IN_USART_RX24));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_USART_RX24, qdev_get_gpio_in(uart4, 0));
	qdev_connect_gpio_out(usart3, 3, qdev_get_gpio_in(model, MODEL_IRQ_IN_USART_HOKUYO1));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_USART_HOKUYO1, qdev_get_gpio_in(usart3, 0));
	qdev_connect_gpio_out(usart1, 3, qdev_get_gpio_in(model, MODEL_IRQ_IN_USART_HOKUYO2));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_USART_HOKUYO2, qdev_get_gpio_in(usart1, 0));
	qdev_connect_gpio_out(spi5, SPI_IRQ_OUT_DEVICE1_RX, qdev_get_gpio_in(model, MODEL_IRQ_IN_LCD));
	qdev_connect_gpio_out(can1, CAN_IRQ_OUT_CAN1_MSG_ID, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_ID));
	qdev_connect_gpio_out(can1, CAN_IRQ_OUT_CAN1_MSG_SIZE, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_SIZE));
	qdev_connect_gpio_out(can1, CAN_IRQ_OUT_CAN1_MSG_DATA_L, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_DATA_L));
	qdev_connect_gpio_out(can1, CAN_IRQ_OUT_CAN1_MSG_DATA_H, qdev_get_gpio_in(model, MODEL_IRQ_IN_CAN1_MSG_DATA_H));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_ID, qdev_get_gpio_in(can1, CAN_IRQ_IN_CAN1_MSG_ID));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_SIZE, qdev_get_gpio_in(can1, CAN_IRQ_IN_CAN1_MSG_SIZE));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_DATA_L, qdev_get_gpio_in(can1, CAN_IRQ_IN_CAN1_MSG_DATA_L));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_CAN1_MSG_DATA_H, qdev_get_gpio_in(can1, CAN_IRQ_IN_CAN1_MSG_DATA_H));

	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_0, qdev_get_gpio_in(gpioc, 15));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_1, qdev_get_gpio_in(gpioc, 13));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_2, qdev_get_gpio_in(gpioe, 3));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_3, qdev_get_gpio_in(gpioe, 4));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_4, qdev_get_gpio_in(gpiog, 10));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_5, qdev_get_gpio_in(gpiog, 11));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_6, qdev_get_gpio_in(gpiod, 4));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_7, qdev_get_gpio_in(gpiod, 7));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_8, qdev_get_gpio_in(gpiod, 3));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_9, qdev_get_gpio_in(gpiod, 2));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_10, qdev_get_gpio_in(gpiog, 7));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_11, qdev_get_gpio_in(gpiog, 6));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_GPIO_GO, qdev_get_gpio_in(gpioc, 14));

	qdev_connect_gpio_out(tim2, 0, qdev_get_gpio_in(model, MODEL_IRQ_IN_ENCODER1)); // encodeur 1 (droite)
	qdev_connect_gpio_out(tim4, 0, qdev_get_gpio_in(model, MODEL_IRQ_IN_ENCODER2)); // encodeur 2 (gauche)
	qdev_connect_gpio_out(tim3, 0, qdev_get_gpio_in(model, MODEL_IRQ_IN_ENCODER3)); // encodeur 3
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_ENCODER1, qdev_get_gpio_in(tim2, 0));  // encodeur 1 (droite)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_ENCODER2, qdev_get_gpio_in(tim4, 0));  // encodeur 2 (gauche)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_ENCODER3, qdev_get_gpio_in(tim3, 0)); // encodeur 3
#if 0
	qdev_connect_gpio_out(tim1, 1, qdev_get_gpio_in(model, 2)); // pwm 1 (droite)
	qdev_connect_gpio_out(tim1, 2, qdev_get_gpio_in(model, 3)); // pwm 2 (gauche)
	qdev_connect_gpio_out(tim1, 3, qdev_get_gpio_in(model, 4)); // pwm 3
	qdev_connect_gpio_out(tim1, 4, qdev_get_gpio_in(model, 5)); // pwm 4
	qdev_connect_gpio_out(gpioe, 8, qdev_get_gpio_in(model, 6)); // direction pwm 1 (droite)
	qdev_connect_gpio_out(gpioe, 10, qdev_get_gpio_in(model, 7)); // direction pwm 2 (gauche)
	qdev_connect_gpio_out(gpioe, 12, qdev_get_gpio_in(model, 8)); // direction pwm 3
	qdev_connect_gpio_out(gpioe, 15, qdev_get_gpio_in(model, 9)); // direction pwm 4

	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_RIGHT, qdev_get_gpio_in(gpioc, 5));  // intensite moteur 1 (droite)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_LEFT, qdev_get_gpio_in(gpioa, 5));  // intensite moteur 2 (gauche)
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_MOT3, qdev_get_gpio_in(gpioc, 3));  // intensite moteur 3
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_I_MOT4, qdev_get_gpio_in(gpioc, 1));  // intensite moteur 4
#endif
}

static QEMUMachine atlantronic_discovery =
{
    .name = "atlantronic",
    .desc = "Robot d'Atlantronic - carte disco",
    .init = atlantronic_init,
};

static void atlantronic_init_discovery(void)
{
    qemu_register_machine(&atlantronic_discovery);
}

machine_init(atlantronic_init_discovery);

