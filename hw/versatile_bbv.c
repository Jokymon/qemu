/*
 * mario.konrad@bbv.ch
 *
 * Demonstration of a simple ARM (versatile) based board.
 *
 * To define a completly new board, the kernel (BSP) must also support this
 * purely fictive board. Since this is (at the moment) out of question, this
 * simulated board will remain based on the versatile board (minus most of the
 * hardware).
 */

#include "sysbus.h"
#include "boards.h"
#include "arm-misc.h"
#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"

static struct arm_boot_info kernel_boot_info;

static void versatile_bbv_board_init(QEMUMachineInitArgs *args)
{
	ARMCPU * cpu;
	MemoryRegion * sysmem;
	MemoryRegion * ram;
	DeviceState * sysctl;
	qemu_irq * cpu_pic;
	DeviceState * dev;
	qemu_irq pic[32];
	qemu_irq sic[32];
	int i;

	/* cpu */

	if (!args->cpu_model) {
		args->cpu_model = "arm926";
	}
	cpu = cpu_arm_init(args->cpu_model);
	if (!cpu) {
		fprintf(stderr, "Unable to find CPU definition\n");
		exit(1);
	}

	/* ram */

	sysmem = get_system_memory();
	ram = g_new(MemoryRegion, 1);
	memory_region_init_ram(ram, "versatile.ram", args->ram_size);
	vmstate_register_ram_global(ram);
	memory_region_add_subregion(sysmem, 0, ram);

	/* sysctrl */

	sysctl = qdev_create(NULL, "realview_sysctl");
	qdev_prop_set_uint32(sysctl, "sys_id", 0x41007004);
	qdev_prop_set_uint32(sysctl, "proc_id", 0x02000000);
	qdev_init_nofail(sysctl);
	sysbus_mmio_map(sysbus_from_qdev(sysctl), 0, 0x10000000);

	/* interrupt control */

	cpu_pic = arm_pic_init_cpu(cpu);
	dev = sysbus_create_varargs("pl190", 0x10140000,
			cpu_pic[ARM_PIC_CPU_IRQ],
			cpu_pic[ARM_PIC_CPU_FIQ], NULL);
	for (i = 0; i < 32; i++) {
		pic[i] = qdev_get_gpio_in(dev, i);
	}

	/* secondary interrupt control */
	/* this is necessary because kernel expects a versatile compatible board, therefore
		the secondary interrupt controller is necessary (PS2 keyboard, PS2 mouse) */

	dev = sysbus_create_simple("versatilepb_sic", 0x10003000, NULL);
	for (i = 0; i < 32; i++) {
		sysbus_connect_irq(sysbus_from_qdev(dev), i, pic[i]);
		sic[i] = qdev_get_gpio_in(dev, i);
	}

	/* timer */

	sysbus_create_simple("sp804", 0x101e2000, pic[4]);
	sysbus_create_simple("sp804", 0x101e3000, pic[5]);

	/* uart */

	sysbus_create_simple("pl011", 0x101f1000, pic[12]);
	sysbus_create_simple("pl011", 0x101f2000, pic[13]);
	sysbus_create_simple("pl011", 0x101f3000, pic[14]);

	/* gpio */

	sysbus_create_simple("pl061", 0x101e4000, pic[6]);
	sysbus_create_simple("pl061", 0x101e5000, pic[7]);
	sysbus_create_simple("pl061", 0x101e6000, pic[8]);
	sysbus_create_simple("pl061bbv", 0x101e7000, pic[9]);

	/* rtc */

	sysbus_create_simple("pl031", 0x101e8000, pic[10]);

	/* default input devices */

	sysbus_create_simple("pl050_keyboard", 0x10006000, sic[3]); /* without this, no keyboard on console -> no login */
    sysbus_create_simple("pl050_mouse", 0x10007000, sic[4]);

	/* initialization of the graphical console, without this, there will be no output on framebuffer-console-oriented kernels */

	/* The versatile/PB actually has a modified Color LCD controller
	   that includes hardware cursor support from the PL111.  */
	dev = sysbus_create_simple("pl110_versatile", 0x10120000, pic[16]);
	/* Wire up the mux control signals from the SYS_CLCD register */
	/*qdev_connect_gpio_out(sysctl, 0, qdev_get_gpio_in(dev, 0));*/

	/* load kernel */

	kernel_boot_info.ram_size = args->ram_size;
	kernel_boot_info.kernel_filename = args->kernel_filename;
	kernel_boot_info.kernel_cmdline = args->kernel_cmdline;
	kernel_boot_info.initrd_filename = args->initrd_filename;
	kernel_boot_info.board_id = 0x183; /* stolen from versatilebp */
	kernel_boot_info.nb_cpus = 1;
    kernel_boot_info.loader_start = 0;
	arm_load_kernel(cpu, &kernel_boot_info);
}

static QEMUMachine versatile_bbv_board_machine = {
	.name = "versatile-bbv",
	.desc = "ARM Versatile - bbv",
	.init = versatile_bbv_board_init,
};

static void versatile_bbv_board_machine_init(void)
{
	qemu_register_machine(&versatile_bbv_board_machine);
}

machine_init(versatile_bbv_board_machine_init);

