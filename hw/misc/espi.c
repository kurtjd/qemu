/*
 * QEMU eSPI Device — Fake eSPI controller/target with shadow memory + VWires
 *
 * Provides transparent shared memory between two QEMU VMs via a chardev
 * socket backend, plus virtual wire signaling with interrupts. Symmetric
 * device — same code used for both host and EC sides.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

#include "hw/misc/espi.h"

/* ========================================================================= */
/* IRQ Logic                                                                 */
/* ========================================================================= */

static void espi_update_irq(ESPIState *s)
{
    int level = (s->int_status & s->int_enable) != 0;
    qemu_set_irq(s->irq, level);
}

/* ========================================================================= */
/* Socket Send Helpers                                                       */
/* ========================================================================= */

static void espi_send_msg(ESPIState *s, uint8_t op, uint16_t offset,
                          uint32_t data, uint8_t size)
{
    uint8_t buf[ESPI_MSG_SIZE];

    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        return;
    }

    buf[0] = op;
    buf[1] = size;
    buf[2] = (offset >> 0) & 0xFF;
    buf[3] = (offset >> 8) & 0xFF;
    buf[4] = (data >>  0) & 0xFF;
    buf[5] = (data >>  8) & 0xFF;
    buf[6] = (data >> 16) & 0xFF;
    buf[7] = (data >> 24) & 0xFF;

    qemu_chr_fe_write_all(&s->chr, buf, sizeof(buf));
}

/* ========================================================================= */
/* Control Register MMIO Ops                                                 */
/* ========================================================================= */

static uint64_t espi_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    ESPIState *s = opaque;

    switch (addr) {
    case ESPI_REG_INT_STATUS:
        return s->int_status;
    case ESPI_REG_INT_ENABLE:
        return s->int_enable;
    case ESPI_REG_VWIRE_OUT:
        return s->vwire_out;
    case ESPI_REG_VWIRE_IN:
        return s->vwire_in;
    case ESPI_REG_PERIPH_ADDR:
        return s->periph_addr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "espi: bad ctrl read offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void espi_ctrl_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    ESPIState *s = opaque;

    switch (addr) {
    case ESPI_REG_INT_STATUS:
        /* W1C: write 1 to clear */
        s->int_status &= ~(uint32_t)val;
        espi_update_irq(s);
        break;
    case ESPI_REG_INT_ENABLE:
        s->int_enable = (uint32_t)val;
        espi_update_irq(s);
        break;
    case ESPI_REG_VWIRE_OUT:
        s->vwire_out = (uint32_t)val;
        espi_send_msg(s, ESPI_OP_VWIRE, 0, s->vwire_out, 4);
        break;
    case ESPI_REG_VWIRE_IN:
        /* Read-only register, ignore writes */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "espi: write to read-only VWIRE_IN\n");
        break;
    case ESPI_REG_PERIPH_ADDR:
        /* Read-only register, ignore writes */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "espi: write to read-only PERIPH_ADDR\n");
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "espi: bad ctrl write offset 0x%" HWADDR_PRIx "\n",
                      addr);
        break;
    }
}

static const MemoryRegionOps espi_ctrl_ops = {
    .read = espi_ctrl_read,
    .write = espi_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ========================================================================= */
/* Shared Memory MMIO Ops                                                    */
/* ========================================================================= */

static uint64_t espi_shmem_read(void *opaque, hwaddr addr, unsigned size)
{
    ESPIState *s = opaque;
    uint32_t val = 0;
    unsigned i;

    if (addr + size > s->shmem_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "espi: shmem read out of range at 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }

    for (i = 0; i < size; i++) {
        val |= (uint32_t)s->shmem_buf[addr + i] << (i * 8);
    }
    return val;
}

static void espi_shmem_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    ESPIState *s = opaque;
    unsigned i;

    if (addr + size > s->shmem_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "espi: shmem write out of range at 0x%" HWADDR_PRIx "\n",
                      addr);
        return;
    }

    /* Update local mirror */
    for (i = 0; i < size; i++) {
        s->shmem_buf[addr + i] = (val >> (i * 8)) & 0xFF;
    }

    /* Forward to the other side */
    espi_send_msg(s, ESPI_OP_MEM_WRITE, (uint16_t)addr, (uint32_t)val, size);
}

static const MemoryRegionOps espi_shmem_ops = {
    .read = espi_shmem_read,
    .write = espi_shmem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ========================================================================= */
/* Chardev Callbacks (receive data from the other VM)                        */
/* ========================================================================= */

static void espi_process_msg(ESPIState *s)
{
    uint8_t op = s->msg_buf[0];
    uint8_t msg_size = s->msg_buf[1];
    uint16_t offset = s->msg_buf[2] | ((uint16_t)s->msg_buf[3] << 8);
    uint32_t data = s->msg_buf[4]
                  | ((uint32_t)s->msg_buf[5] << 8)
                  | ((uint32_t)s->msg_buf[6] << 16)
                  | ((uint32_t)s->msg_buf[7] << 24);

    switch (op) {
    case ESPI_OP_MEM_WRITE:
        if (offset < s->shmem_size) {
            unsigned i;
            unsigned count = msg_size;

            if (count > 4) {
                count = 4;
            }
            if (offset + count > s->shmem_size) {
                count = s->shmem_size - offset;
            }
            for (i = 0; i < count; i++) {
                s->shmem_buf[offset + i] = (data >> (i * 8)) & 0xFF;
            }
        }
        s->periph_addr = offset;
        s->int_status |= ESPI_INT_PERIPH_PENDING;
        espi_update_irq(s);
        break;

    case ESPI_OP_VWIRE:
        s->vwire_in = data;
        s->int_status |= ESPI_INT_VWIRE_PENDING;
        espi_update_irq(s);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "espi: unknown msg op 0x%02x\n", op);
        break;
    }
}

static int espi_can_receive(void *opaque)
{
    ESPIState *s = opaque;
    return sizeof(s->msg_buf) - s->msg_buffered_bytes;
}

static void espi_receive(void *opaque, const uint8_t *buf, int size)
{
    ESPIState *s = opaque;

    assert(size >= 0 && s->msg_buffered_bytes + size <= (int)sizeof(s->msg_buf));

    memcpy(s->msg_buf + s->msg_buffered_bytes, buf, size);
    s->msg_buffered_bytes += size;

    if (s->msg_buffered_bytes >= ESPI_MSG_SIZE) {
        espi_process_msg(s);
        s->msg_buffered_bytes = 0;
    }
}

/* ========================================================================= */
/* Device Lifecycle                                                          */
/* ========================================================================= */

static void espi_reset(DeviceState *dev)
{
    ESPIState *s = ESPI(dev);

    s->int_status = 0;
    s->int_enable = 0;
    s->vwire_out = 0;
    s->vwire_in = 0;
    s->periph_addr = 0;
    s->msg_buffered_bytes = 0;

    if (s->shmem_buf) {
        memset(s->shmem_buf, 0, s->shmem_size);
    }

    qemu_set_irq(s->irq, 0);
}

static void espi_realize(DeviceState *dev, Error **errp)
{
    ESPIState *s = ESPI(dev);

    if (s->shmem_size == 0) {
        error_setg(errp, "espi: shmem-size must be > 0");
        return;
    }

    /* Allocate the shared memory mirror */
    s->shmem_buf = g_malloc0(s->shmem_size);

    /* Initialize shared memory MMIO region */
    memory_region_init_io(&s->shmem, OBJECT(s), &espi_shmem_ops, s,
                          "espi-shmem", s->shmem_size);

    /* Map shared memory at the user-configured base address */
    memory_region_add_subregion(get_system_memory(), s->shmem_base,
                                &s->shmem);

    /* Set up chardev handlers if a backend is connected */
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr,
                                 espi_can_receive,
                                 espi_receive,
                                 NULL,  /* event */
                                 NULL,  /* be_change */
                                 s,
                                 NULL,  /* context */
                                 true); /* set_open */
    }
}

static void espi_unrealize(DeviceState *dev)
{
    ESPIState *s = ESPI(dev);

    memory_region_del_subregion(get_system_memory(), &s->shmem);
    g_free(s->shmem_buf);
    s->shmem_buf = NULL;
}

static void espi_instance_init(Object *obj)
{
    ESPIState *s = ESPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Control registers MMIO region (fixed address, assigned by machine) */
    memory_region_init_io(&s->ctrl_regs, obj, &espi_ctrl_ops, s,
                          "espi-ctrl", ESPI_CTRL_REG_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl_regs);

    /* Single IRQ output */
    sysbus_init_irq(sbd, &s->irq);
}

/* ========================================================================= */
/* VMState (migration / snapshots)                                           */
/* ========================================================================= */

static const VMStateDescription vmstate_espi = {
    .name = TYPE_ESPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_status, ESPIState),
        VMSTATE_UINT32(int_enable, ESPIState),
        VMSTATE_UINT32(vwire_out, ESPIState),
        VMSTATE_UINT32(vwire_in, ESPIState),
        VMSTATE_UINT32(periph_addr, ESPIState),
        VMSTATE_UINT32(shmem_size, ESPIState),
        VMSTATE_VARRAY_UINT32(shmem_buf, ESPIState, shmem_size,
                              0, vmstate_info_uint8, uint8_t),
        VMSTATE_END_OF_LIST()
    }
};

/* ========================================================================= */
/* QOM Boilerplate                                                           */
/* ========================================================================= */

static const Property espi_properties[] = {
    DEFINE_PROP_CHR("chardev", ESPIState, chr),
    DEFINE_PROP_UINT32("shmem-size", ESPIState, shmem_size, 4 * KiB),
    DEFINE_PROP_UINT64("shmem-base", ESPIState, shmem_base, 0x10200000),
};

static void espi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = espi_realize;
    dc->unrealize = espi_unrealize;
    device_class_set_legacy_reset(dc, espi_reset);
    dc->vmsd = &vmstate_espi;
    device_class_set_props(dc, espi_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo espi_types[] = {
    {
        .name          = TYPE_ESPI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESPIState),
        .instance_init = espi_instance_init,
        .class_init    = espi_class_init,
    },
};

DEFINE_TYPES(espi_types)
