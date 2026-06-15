/*
 * QEMU RISC-V Embedded Controller (EC) Board
 *
 * Minimal RV32 board: CLINT, PLIC, UART, RAM.
 * No device tree — firmware knows the hardware at compile time.
 * MCU-style boot: CPU starts executing directly from RAM base.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/char/serial-mm.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/ec.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/odp/i2c-controller.h"
#include "hw/odp/i2c-target.h"
#include "hw/odp/gpio.h"
#include "chardev/char.h"
#include "system/system.h"
#include "elf.h"

static const MemMapEntry ec_memmap[] = {
    [EC_CLINT] = { 0x02000000,      0x10000 },
    [EC_PLIC]  = { 0x0C000000, EC_PLIC_SIZE },
    [EC_UART0] = { 0x10000000,       0x1000 },
    [EC_I2C0]  = { 0x10001000,       0x1000 },
    [EC_I2C_TARGET] = { 0x10002000,  0x1000 },
    [EC_GPIO]  = { 0x10003000,       0x1000 },
    [EC_RAM]   = { 0x80000000,          0x0 },
};

/*
 * Load an image into EC RAM. The image is typically an ELF (the EC firmware
 * compiled by cargo), but a raw flat binary is also accepted as a fallback.
 *
 * For ELF images we parse the program headers so each segment lands at its
 * correct link address and we honour the ELF entry point. Loading an ELF as a
 * flat binary would copy the ELF header itself to RAM base and the CPU would
 * execute the ELF magic as instructions. Returns the address the CPU should
 * reset to (the entry point).
 */
static hwaddr ec_load_image(const char *filename, hwaddr ram_base,
                            uint64_t ram_size)
{
    uint64_t entry = 0;
    ssize_t size;

    /* Try ELF first: places segments correctly and reports the entry point. */
    size = load_elf(filename, NULL, NULL, NULL, &entry, NULL, NULL, NULL,
                    ELFDATA2LSB, EM_RISCV, 1, 0);
    if (size > 0) {
        return entry;
    }

    /* Fall back to a raw flat binary loaded at RAM base. */
    if (load_image_targphys(filename, ram_base, ram_size) < 0) {
        error_report("could not load image '%s'", filename);
        exit(1);
    }

    return ram_base;
}

static void ec_machine_done(Notifier *notifier, void *data)
{
    RISCVECState *s = container_of(notifier, RISCVECState, machine_done);
    MachineState *machine = MACHINE(s);
    hwaddr ram_base = s->memmap[EC_RAM].base;
    const char *filename = NULL;
    hwaddr entry;

    if (machine->firmware && strcmp(machine->firmware, "none")) {
        filename = machine->firmware;
    } else if (machine->kernel_filename) {
        filename = machine->kernel_filename;
    }

    if (!filename) {
        return;
    }

    entry = ec_load_image(filename, ram_base, machine->ram_size);

    /*
     * Point the hart's reset vector at the image entry point. For a flat
     * binary this is RAM base; for an ELF it is the real entry address.
     */
    s->soc.harts[0].env.resetvec = entry;
}

static void ec_machine_init(MachineState *machine)
{
    RISCVECState *s = RISCV_EC_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    char *plic_hart_config;

    s->memmap = ec_memmap;

    /* Initialize hart */
    object_initialize_child(OBJECT(machine), "soc", &s->soc,
                            TYPE_RISCV_HART_ARRAY);
    object_property_set_str(OBJECT(&s->soc), "cpu-type",
                            machine->cpu_type, &error_abort);
    object_property_set_int(OBJECT(&s->soc), "hartid-base",
                            0, &error_abort);
    object_property_set_int(OBJECT(&s->soc), "num-harts",
                            1, &error_abort);
    object_property_set_int(OBJECT(&s->soc), "resetvec",
                            s->memmap[EC_RAM].base, &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);

    /* Register RAM */
    memory_region_add_subregion(system_memory,
                                s->memmap[EC_RAM].base, machine->ram);

    /* CLINT (timer + software interrupts) */
    riscv_aclint_swi_create(s->memmap[EC_CLINT].base,
                            0, 1, false);
    riscv_aclint_mtimer_create(s->memmap[EC_CLINT].base +
                               RISCV_ACLINT_SWI_SIZE,
                               RISCV_ACLINT_DEFAULT_MTIMER_SIZE,
                               0, 1,
                               RISCV_ACLINT_DEFAULT_MTIMECMP,
                               RISCV_ACLINT_DEFAULT_MTIME,
                               RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ, true);

    /*
     * PLIC: M-mode only (single context per hart). The EC firmware runs
     * entirely in M-mode, so we hardcode "M" rather than deriving the
     * config from the CPU (which would add an S-mode context and require
     * EC_PLIC_SIZE to reserve a second context).
     */
    plic_hart_config = g_strdup("M");
    s->plic = sifive_plic_create(
        s->memmap[EC_PLIC].base,
        plic_hart_config,
        1, 0,
        EC_IRQCHIP_NUM_SOURCES,
        (1U << EC_IRQCHIP_NUM_PRIO_BITS) - 1,
        EC_PLIC_PRIORITY_BASE,
        EC_PLIC_PENDING_BASE,
        EC_PLIC_ENABLE_BASE,
        EC_PLIC_ENABLE_STRIDE,
        EC_PLIC_CONTEXT_BASE,
        EC_PLIC_CONTEXT_STRIDE,
        s->memmap[EC_PLIC].size);
    g_free(plic_hart_config);

    /* UART */
    serial_mm_init(system_memory, s->memmap[EC_UART0].base, 0,
                   qdev_get_gpio_in(s->plic, EC_UART0_IRQ),
                   399193, serial_hd(0), DEVICE_LITTLE_ENDIAN);

    /*
     * Socket-backed I2C controller. The chardev backend is optional: if the
     * user did not supply '-chardev socket,id=ec-i2c-controller,...' the
     * device still maps and simply has no peer until one is attached.
     */
    s->i2c0 = qdev_new(TYPE_ODP_I2C_CONTROLLER);
    {
        Chardev *i2c_chr = qemu_chr_find("ec-i2c-controller");
        if (i2c_chr) {
            qdev_prop_set_chr(s->i2c0, "chardev", i2c_chr);
        }
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->i2c0), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->i2c0), 0, s->memmap[EC_I2C0].base);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->i2c0), 0,
                       qdev_get_gpio_in(s->plic, EC_I2C0_IRQ));

    /*
     * Socket-backed I2C target. Like the controller, its chardev backend is
     * optional and looked up by id ('-chardev socket,id=ec-i2c-target,...').
     */
    s->i2c_target = qdev_new(TYPE_ODP_I2C_TARGET);
    {
        Chardev *tgt_chr = qemu_chr_find("ec-i2c-target");
        if (tgt_chr) {
            qdev_prop_set_chr(s->i2c_target, "chardev", tgt_chr);
        }
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->i2c_target), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->i2c_target), 0,
                    s->memmap[EC_I2C_TARGET].base);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->i2c_target), 0,
                       qdev_get_gpio_in(s->plic, EC_I2C_TARGET_IRQ));

    /*
     * Socket-backed bidirectional GPIO controller. Pin 0 is bridged to a peer
     * over a chardev looked up by id ('-chardev socket,id=ec-gpio0,...'); the
     * backend is optional and the device still maps without a peer attached.
     */
    s->gpio0 = qdev_new(TYPE_ODP_GPIO);
    {
        Chardev *gpio_chr = qemu_chr_find("ec-gpio0");
        if (gpio_chr) {
            qdev_prop_set_chr(s->gpio0, "gpio0", gpio_chr);
        }
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->gpio0), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->gpio0), 0, s->memmap[EC_GPIO].base);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->gpio0), 0,
                       qdev_get_gpio_in(s->plic, EC_GPIO_IRQ));

    /* Firmware loading happens in machine_done */
    s->machine_done.notify = ec_machine_done;
    qemu_add_machine_init_done_notifier(&s->machine_done);
}

static void ec_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V Embedded Controller";
    mc->init = ec_machine_init;
    mc->max_cpus = EC_CPUS_MAX;
    mc->default_cpu_type = TYPE_RISCV_CPU_BASE;
    mc->default_ram_id = "riscv.ec.ram";
}

static const TypeInfo ec_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("ec"),
    .parent     = TYPE_MACHINE,
    .class_init = ec_machine_class_init,
    .instance_size = sizeof(RISCVECState),
};

static void ec_machine_init_register_types(void)
{
    type_register_static(&ec_machine_typeinfo);
}

type_init(ec_machine_init_register_types)
