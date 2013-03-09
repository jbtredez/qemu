#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/otgd_fs_regs.h"

#include "atlantronic_model.h"
#include "atlantronic_hokuyo.h"
#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"
#include "exec/address-spaces.h"

static void atlantronic_foo_init(QEMUMachineInitArgs *args)
{
	configure_icount("0");
	system_clock_scale = 1;

	MemoryRegion *address_space_mem = get_system_memory();

	const int flash_size = 256;
	const int sram_size = 64;
	qemu_irq* pic = armv7m_init(address_space_mem, flash_size, sram_size, args->kernel_filename, "cortex-m3");

	// rcc
	sysbus_create_simple("atlantronic-rcc", RCC_BASE, NULL);

	// gpio
	DeviceState* gpioa = sysbus_create_simple("atlantronic-gpio", GPIOA_BASE, NULL);
	DeviceState* gpiob = sysbus_create_simple("atlantronic-gpio", GPIOB_BASE, NULL);
	DeviceState* gpioc = sysbus_create_simple("atlantronic-gpio", GPIOC_BASE, NULL);
	sysbus_create_simple("atlantronic-gpio", GPIOD_BASE, NULL);
	DeviceState* gpioe = sysbus_create_simple("atlantronic-gpio", GPIOE_BASE, NULL);
	sysbus_create_simple("atlantronic-gpio", GPIOF_BASE, NULL);

	// mise a 1 des pin 2 et 3 de gpioa (id de foo)
	qemu_set_irq(qdev_get_gpio_in(gpioa, 2), 1);
	qemu_set_irq(qdev_get_gpio_in(gpioa, 3), 1);

	// tim
	DeviceState* tim1 = sysbus_create_simple("atlantronic-tim", TIM1_BASE, NULL);
	DeviceState* tim2 = sysbus_create_simple("atlantronic-tim", TIM2_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM3_BASE, NULL);
	DeviceState* tim4 = sysbus_create_simple("atlantronic-tim", TIM4_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM5_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM6_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM7_BASE, NULL);
	sysbus_create_simple("atlantronic-tim", TIM8_BASE, NULL);

	// dma
	DeviceState* dma1 = sysbus_create_simple("atlantronic-dma", DMA1_BASE, NULL);
	DeviceState* dma1_chan1 = sysbus_create_simple("atlantronic-dma-chan", DMA1_Channel1_BASE, NULL);
	DeviceState* dma1_chan2 = sysbus_create_simple("atlantronic-dma-chan", DMA1_Channel2_BASE, NULL);
	DeviceState* dma1_chan3 = sysbus_create_simple("atlantronic-dma-chan", DMA1_Channel3_BASE, NULL);
	DeviceState* dma1_chan4 = sysbus_create_simple("atlantronic-dma-chan", DMA1_Channel4_BASE, NULL);
	DeviceState* dma1_chan5 = sysbus_create_simple("atlantronic-dma-chan", DMA1_Channel5_BASE, NULL);
	DeviceState* dma1_chan6 = sysbus_create_simple("atlantronic-dma-chan", DMA1_Channel6_BASE, NULL);
	DeviceState* dma1_chan7 = sysbus_create_simple("atlantronic-dma-chan", DMA1_Channel7_BASE, NULL);
	qdev_connect_gpio_out(dma1_chan1, 0, qdev_get_gpio_in(dma1, 0));  // dma1_chan1 -> dma1
	qdev_connect_gpio_out(dma1_chan2, 0, qdev_get_gpio_in(dma1, 1));  // dma1_chan2 -> dma1
	qdev_connect_gpio_out(dma1_chan3, 0, qdev_get_gpio_in(dma1, 2));  // dma1_chan3 -> dma1
	qdev_connect_gpio_out(dma1_chan4, 0, qdev_get_gpio_in(dma1, 3));  // dma1_chan4 -> dma1
	qdev_connect_gpio_out(dma1_chan5, 0, qdev_get_gpio_in(dma1, 4));  // dma1_chan5 -> dma1
	qdev_connect_gpio_out(dma1_chan6, 0, qdev_get_gpio_in(dma1, 5));  // dma1_chan6 -> dma1
	qdev_connect_gpio_out(dma1_chan7, 0, qdev_get_gpio_in(dma1, 6));  // dma1_chan7 -> dma1
	qdev_connect_gpio_out(dma1, 0, pic[DMA1_Channel1_IRQn]);
	qdev_connect_gpio_out(dma1, 1, pic[DMA1_Channel2_IRQn]);
	qdev_connect_gpio_out(dma1, 2, pic[DMA1_Channel3_IRQn]);
	qdev_connect_gpio_out(dma1, 3, pic[DMA1_Channel4_IRQn]);
	qdev_connect_gpio_out(dma1, 4, pic[DMA1_Channel5_IRQn]);
	qdev_connect_gpio_out(dma1, 5, pic[DMA1_Channel6_IRQn]);
	qdev_connect_gpio_out(dma1, 6, pic[DMA1_Channel7_IRQn]);

	DeviceState* dma2 = sysbus_create_simple("atlantronic-dma", DMA2_BASE, NULL);
	DeviceState* dma2_chan1 = sysbus_create_simple("atlantronic-dma-chan", DMA2_Channel1_BASE, NULL);
	DeviceState* dma2_chan2 = sysbus_create_simple("atlantronic-dma-chan", DMA2_Channel2_BASE, NULL);
	DeviceState* dma2_chan3 = sysbus_create_simple("atlantronic-dma-chan", DMA2_Channel3_BASE, NULL);
	DeviceState* dma2_chan4 = sysbus_create_simple("atlantronic-dma-chan", DMA2_Channel4_BASE, NULL);
	DeviceState* dma2_chan5 = sysbus_create_simple("atlantronic-dma-chan", DMA2_Channel5_BASE, NULL);
	qdev_connect_gpio_out(dma2_chan1, 0, qdev_get_gpio_in(dma2, 0));  // dma2_chan1 -> dma2
	qdev_connect_gpio_out(dma2_chan2, 0, qdev_get_gpio_in(dma2, 1));  // dma2_chan2 -> dma2
	qdev_connect_gpio_out(dma2_chan3, 0, qdev_get_gpio_in(dma2, 2));  // dma2_chan3 -> dma2
	qdev_connect_gpio_out(dma2_chan4, 0, qdev_get_gpio_in(dma2, 3));  // dma2_chan4 -> dma2
	qdev_connect_gpio_out(dma2_chan5, 0, qdev_get_gpio_in(dma2, 4));  // dma2_chan5 -> dma2
	qdev_connect_gpio_out(dma2, 0, pic[DMA2_Channel1_IRQn]);
	qdev_connect_gpio_out(dma2, 1, pic[DMA2_Channel2_IRQn]);
	qdev_connect_gpio_out(dma2, 2, pic[DMA2_Channel3_IRQn]);
	qdev_connect_gpio_out(dma2, 3, pic[DMA2_Channel4_IRQn]);
	qdev_connect_gpio_out(dma2, 4, pic[DMA2_Channel5_IRQn]);

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

	// usart
	DeviceState* usart3 = sysbus_create_simple("atlantronic-usart", USART3_BASE, NULL);
	qdev_connect_gpio_out(usart3, 0, pic[USART3_IRQn]);  // usart3 -> it hw
	qdev_connect_gpio_out(usart3, 1, qdev_get_gpio_in(dma1_chan3, 0));  // usart3 -> dma1_chan3 (rx)


	// usb
	DeviceState* usb = sysbus_create_simple("atlantronic-usb", USB_OTG_FS_BASE_ADDR, NULL);
	sysbus_connect_irq(SYS_BUS_DEVICE(usb), 0, pic[OTG_FS_IRQn]);

	// modele
	DeviceState* model = sysbus_create_simple("atlantronic-model", 0, NULL);
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

	// hokuyo
	DeviceState* hokuyo1 = sysbus_create_simple("atlantronic-hokuyo", 0, NULL);
	qdev_connect_gpio_out(usart3, 3, qdev_get_gpio_in(hokuyo1, HOKUYO_IRQ_IN_USART_DATA));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_X, qdev_get_gpio_in(hokuyo1, HOKUYO_IRQ_IN_X));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_Y, qdev_get_gpio_in(hokuyo1, HOKUYO_IRQ_IN_Y));
	qdev_connect_gpio_out(model, MODEL_IRQ_OUT_ALPHA, qdev_get_gpio_in(hokuyo1, HOKUYO_IRQ_IN_ALPHA));
	qdev_connect_gpio_out(hokuyo1, 0, qdev_get_gpio_in(usart3, 0));
}

static QEMUMachine atlantronic_foo =
{
    .name = "atlantronic-foo",
    .desc = "Robot d'Atlantronic - carte foo",
    .init = atlantronic_foo_init,
};

static void atlantronic_init(void)
{
    qemu_register_machine(&atlantronic_foo);
}

machine_init(atlantronic_init);
