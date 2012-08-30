#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/otgd_fs_regs.h"

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"
#include "exec-memory.h"

static void atlantronic_foo_init(ram_addr_t ram_size,
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

	// gpio
	DeviceState* gpioa = sysbus_create_simple("atlantronic-gpio", GPIOA_BASE, NULL);
	sysbus_create_simple("atlantronic-gpio", GPIOB_BASE, NULL);
	sysbus_create_simple("atlantronic-gpio", GPIOC_BASE, NULL);
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

	// usb
	DeviceState* usbDev = sysbus_create_simple("atlantronic-usb", USB_OTG_FS_BASE_ADDR, NULL);
	sysbus_connect_irq(sysbus_from_qdev(usbDev), 0, pic[OTG_FS_IRQn]);

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

	qdev_connect_gpio_out(model, 0, qdev_get_gpio_in(tim2, 0)); // encodeur 1 (droite)
	qdev_connect_gpio_out(model, 1, qdev_get_gpio_in(tim4, 0)); // encodeur 2 (gauche)
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
