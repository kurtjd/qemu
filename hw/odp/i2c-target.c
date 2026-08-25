/*
 * ODP socket-backed I2C target (slave)
 *
 * The slave-side counterpart to odp-i2c-controller.  It does not drive an
 * in-process QEMU I2CBus; instead it speaks the same tiny framed protocol over
 * a chardev (typically "-chardev socket,...") to an external endpoint - e.g.
 * the odp-i2c-controller running in another QEMU VM acting as the host.
 *
 * The guest drives four 32-bit registers (CTRL/STATUS/ADDR/DATA) and the
 * device runs an I2C-target state machine, signalling bus events back through
 * status bits and a single interrupt line.  The register/interrupt model is
 * designed to back the OpenDevicePartnership embedded-mcu async I2C target HAL
 * (listen / respond_to_read / respond_to_write / recover).
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qom/object.h"
#include "hw/odp/i2c-target.h"

/* MMIO register map (32-bit registers). */
#define ODP_I2CT_REG_CTRL    0x0
#define ODP_I2CT_REG_STATUS  0x4
#define ODP_I2CT_REG_ADDR    0x8
#define ODP_I2CT_REG_DATA    0xc
#define ODP_I2CT_MMIO_SIZE   0x1000

/* CTRL register bits. */
#define CTRL_RX_ACK     (1u << 0) /* W1 strobe: ACK pending RX byte, continue   */
#define CTRL_RX_NAK     (1u << 1) /* W1 strobe: NAK pending RX byte (reject)    */
#define CTRL_INT_START  (1u << 2) /* int enable: addressed (START set)   (R/W)  */
#define CTRL_INT_STOP   (1u << 3) /* int enable: STOP                    (R/W)  */
#define CTRL_INT_RX_RDY (1u << 4) /* int enable: byte received           (R/W)  */
#define CTRL_INT_TX     (1u << 5) /* int enable: TX_DONE                 (R/W)  */
#define CTRL_RESET      (1u << 6) /* W1 self-clearing: recover()                */
#define CTRL_IE_MASK    (CTRL_INT_START | CTRL_INT_STOP | \
                         CTRL_INT_RX_RDY | CTRL_INT_TX)

/* STATUS register bits. */
#define STATUS_RW       (1u << 0) /* direction: 1=master read, 0=write   (R)    */
#define STATUS_START    (1u << 1) /* addressed (address matched)       (R/W1C)  */
#define STATUS_STOP     (1u << 2) /* master sent STOP                  (R/W1C)  */
#define STATUS_RESTART  (1u << 3) /* repeated START (also sets START)  (R/W1C)  */
#define STATUS_RX_RDY   (1u << 4) /* byte in DATA awaiting strobe        (R)    */
#define STATUS_TX_DONE  (1u << 5) /* TX byte clocked + master replied  (R/W1C)  */
#define STATUS_TX_NAK   (1u << 6) /* last TX byte: 1=master NAKed(done)   (R)    */
#define STATUS_W1C_MASK (STATUS_START | STATUS_STOP | STATUS_RESTART | \
                         STATUS_TX_DONE)

#define ADDR_MASK       0x7fu

/* Wire protocol opcodes (identical to odp-i2c-controller). */
#define WIRE_START      0x42
#define WIRE_STOP       0x77
#define WIRE_DATA       0xDA
#define WIRE_ACK        0x01
#define WIRE_NAK        0x00

typedef enum {
    ODP_I2CT_IDLE,             /* not addressed; awaiting START + matching addr */
    ODP_I2CT_WRITE_WAIT_DATA,  /* master write: awaiting next DATA/STOP/START   */
    ODP_I2CT_WRITE_WAIT_STROBE,/* RX byte latched; awaiting RX_ACK/RX_NAK       */
    ODP_I2CT_READ_WAIT_TXBYTE, /* master read: awaiting firmware TX byte (DATA) */
    ODP_I2CT_READ_WAIT_ACK,    /* sent a TX byte; awaiting master ACK/NAK       */
    ODP_I2CT_READ_ENDED,       /* master NAKed; awaiting STOP or repeated START */
} OdpI2CTState;

typedef enum {
    PEND_NONE,  /* next wire byte is an opcode             */
    PEND_ADDR,  /* next wire byte is the address (post-START) */
    PEND_DATA,  /* next wire byte is RX payload (post-DATA) */
} OdpI2CTPending;

struct OdpI2CTargetState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
    CharBackend chr;

    /* Register state. */
    uint32_t ctrl;       /* interrupt-enable bits only */
    uint32_t status;
    uint8_t addr_match;  /* 7-bit address we answer to */
    uint8_t data;        /* RX byte (write) / last TX byte (read) */

    /* State machine. */
    OdpI2CTState state;
    OdpI2CTPending pending;
};

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static void odp_i2ct_update_irq(OdpI2CTargetState *s)
{
    bool level =
        ((s->status & STATUS_START)  && (s->ctrl & CTRL_INT_START)) ||
        ((s->status & STATUS_STOP)   && (s->ctrl & CTRL_INT_STOP)) ||
        ((s->status & STATUS_RX_RDY) && (s->ctrl & CTRL_INT_RX_RDY)) ||
        ((s->status & STATUS_TX_DONE) && (s->ctrl & CTRL_INT_TX));

    qemu_set_irq(s->irq, level);
}

static void odp_i2ct_wire_send(OdpI2CTargetState *s, uint8_t op)
{
    /* Best effort: if the backend is gone the CLOSED path / firmware recovers. */
    qemu_chr_fe_write_all(&s->chr, &op, 1);
}

/*
 * Reset the target.  Shared by the CTRL recover() strobe and the QOM
 * cold-reset path.  recover() (keep_config=true) preserves the match address
 * and interrupt-enable bits; cold reset clears everything.
 */
static void odp_i2ct_do_reset(OdpI2CTargetState *s, bool keep_config)
{
    s->status = 0;
    s->data = 0;
    s->state = ODP_I2CT_IDLE;
    s->pending = PEND_NONE;

    if (!keep_config) {
        s->ctrl = 0;
        s->addr_match = 0;
    }

    qemu_set_irq(s->irq, 0);
    qemu_chr_fe_accept_input(&s->chr);
}

/* ------------------------------------------------------------------ */
/* Socket receive (controller -> target)                              */
/* ------------------------------------------------------------------ */

static void odp_i2ct_handle_address(OdpI2CTargetState *s, uint8_t wire_addr)
{
    bool is_restart = (s->state != ODP_I2CT_IDLE);
    bool is_recv = wire_addr & 0x1;

    if ((wire_addr >> 1) != s->addr_match) {
        /* Not for us: NAK and return to idle. */
        odp_i2ct_wire_send(s, WIRE_NAK);
        s->state = ODP_I2CT_IDLE;
        return;
    }

    /* Addressed.  A repeated start also raises the START event. */
    s->status |= STATUS_START;
    if (is_restart) {
        s->status |= STATUS_RESTART;
    }
    if (is_recv) {
        s->status |= STATUS_RW;
        s->state = ODP_I2CT_READ_WAIT_TXBYTE;
    } else {
        s->status &= ~STATUS_RW;
        s->state = ODP_I2CT_WRITE_WAIT_DATA;
    }

    odp_i2ct_wire_send(s, WIRE_ACK);
    odp_i2ct_update_irq(s);
}

static void odp_i2ct_handle_rx_byte(OdpI2CTargetState *s, uint8_t byte)
{
    if (s->state != ODP_I2CT_WRITE_WAIT_DATA) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unexpected DATA byte in state %d\n",
                      TYPE_ODP_I2C_TARGET, s->state);
        return;
    }

    /* Latch the byte and stall the bus until firmware ACKs/NAKs it. */
    s->data = byte;
    s->status |= STATUS_RX_RDY;
    s->state = ODP_I2CT_WRITE_WAIT_STROBE;
    odp_i2ct_update_irq(s);
}

static void odp_i2ct_handle_stop(OdpI2CTargetState *s)
{
    s->status |= STATUS_STOP;
    s->state = ODP_I2CT_IDLE;
    odp_i2ct_update_irq(s);
}

static void odp_i2ct_handle_master_ack(OdpI2CTargetState *s, bool ack)
{
    if (s->state != ODP_I2CT_READ_WAIT_ACK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unexpected ACK/NAK in state %d\n",
                      TYPE_ODP_I2C_TARGET, s->state);
        return;
    }

    s->status |= STATUS_TX_DONE;
    if (ack) {
        s->status &= ~STATUS_TX_NAK;
        s->state = ODP_I2CT_READ_WAIT_TXBYTE; /* firmware feeds next byte */
    } else {
        s->status |= STATUS_TX_NAK;
        s->state = ODP_I2CT_READ_ENDED;       /* master done; await STOP/START */
    }
    odp_i2ct_update_irq(s);
}

static int odp_i2ct_can_receive(void *opaque)
{
    OdpI2CTargetState *s = opaque;

    /*
     * While a received byte is awaiting the firmware ACK/NAK strobe we hold
     * the bus (emulated clock-stretch) and refuse further input.  Everywhere
     * else we can accept an opcode plus its one payload byte.
     */
    if (s->state == ODP_I2CT_WRITE_WAIT_STROBE) {
        return 0;
    }
    return 2;
}

static void odp_i2ct_receive(void *opaque, const uint8_t *buf, int size)
{
    OdpI2CTargetState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        uint8_t b = buf[i];

        switch (s->pending) {
        case PEND_ADDR:
            s->pending = PEND_NONE;
            odp_i2ct_handle_address(s, b);
            continue;
        case PEND_DATA:
            s->pending = PEND_NONE;
            odp_i2ct_handle_rx_byte(s, b);
            continue;
        case PEND_NONE:
            break;
        }

        switch (b) {
        case WIRE_START:
            s->pending = PEND_ADDR;
            break;
        case WIRE_STOP:
            odp_i2ct_handle_stop(s);
            break;
        case WIRE_DATA:
            s->pending = PEND_DATA;
            break;
        case WIRE_ACK:
            odp_i2ct_handle_master_ack(s, true);
            break;
        case WIRE_NAK:
            odp_i2ct_handle_master_ack(s, false);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: unknown wire opcode 0x%02x\n",
                          TYPE_ODP_I2C_TARGET, b);
            break;
        }
    }
}

static void odp_i2ct_event(void *opaque, QEMUChrEvent event)
{
    OdpI2CTargetState *s = opaque;

    if (event == CHR_EVENT_CLOSED && s->state != ODP_I2CT_IDLE) {
        /* Link dropped mid-transfer: abort to idle and flag a STOP. */
        s->status |= STATUS_STOP;
        s->state = ODP_I2CT_IDLE;
        s->pending = PEND_NONE;
        odp_i2ct_update_irq(s);
    }
}

static int odp_i2ct_be_change(void *opaque)
{
    OdpI2CTargetState *s = opaque;

    qemu_chr_fe_set_handlers(&s->chr, odp_i2ct_can_receive, odp_i2ct_receive,
                             odp_i2ct_event, odp_i2ct_be_change, s, NULL, true);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Firmware-driven actions (CTRL strobes, DATA writes)                */
/* ------------------------------------------------------------------ */

static void odp_i2ct_rx_strobe(OdpI2CTargetState *s, bool ack)
{
    if (s->state != ODP_I2CT_WRITE_WAIT_STROBE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: RX ACK/NAK strobe with no byte pending\n",
                      TYPE_ODP_I2C_TARGET);
        return;
    }

    odp_i2ct_wire_send(s, ack ? WIRE_ACK : WIRE_NAK);
    s->status &= ~STATUS_RX_RDY;
    s->state = ack ? ODP_I2CT_WRITE_WAIT_DATA : ODP_I2CT_IDLE;

    /* Bus released: resume accepting input. */
    qemu_chr_fe_accept_input(&s->chr);
    odp_i2ct_update_irq(s);
}

static void odp_i2ct_tx_byte(OdpI2CTargetState *s, uint8_t byte)
{
    if (s->state != ODP_I2CT_READ_WAIT_TXBYTE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: TX byte written in state %d\n",
                      TYPE_ODP_I2C_TARGET, s->state);
        return;
    }

    s->data = byte;
    qemu_chr_fe_write_all(&s->chr, (const uint8_t[]){ WIRE_DATA, byte }, 2);
    s->state = ODP_I2CT_READ_WAIT_ACK;
}

/* ------------------------------------------------------------------ */
/* MMIO                                                               */
/* ------------------------------------------------------------------ */

static uint64_t odp_i2ct_read(void *opaque, hwaddr offset, unsigned size)
{
    OdpI2CTargetState *s = opaque;

    switch (offset) {
    case ODP_I2CT_REG_CTRL:
        return s->ctrl & CTRL_IE_MASK;
    case ODP_I2CT_REG_STATUS:
        return s->status;
    case ODP_I2CT_REG_ADDR:
        return s->addr_match;
    case ODP_I2CT_REG_DATA:
        return s->data;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_ODP_I2C_TARGET, offset);
        return 0;
    }
}

static void odp_i2ct_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    OdpI2CTargetState *s = opaque;

    switch (offset) {
    case ODP_I2CT_REG_CTRL:
        if (value & CTRL_RESET) {
            /* recover(): overrides everything else in this write. */
            odp_i2ct_do_reset(s, true);
            break;
        }
        s->ctrl = value & CTRL_IE_MASK;
        if (value & CTRL_RX_ACK) {
            odp_i2ct_rx_strobe(s, true);
        } else if (value & CTRL_RX_NAK) {
            odp_i2ct_rx_strobe(s, false);
        }
        odp_i2ct_update_irq(s);
        break;

    case ODP_I2CT_REG_STATUS:
        /* Write-1-to-clear the latched status bits. */
        s->status &= ~(value & STATUS_W1C_MASK);
        odp_i2ct_update_irq(s);
        break;

    case ODP_I2CT_REG_ADDR:
        s->addr_match = value & ADDR_MASK;
        break;

    case ODP_I2CT_REG_DATA:
        odp_i2ct_tx_byte(s, value & 0xff);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_ODP_I2C_TARGET, offset);
        break;
    }
}

static const MemoryRegionOps odp_i2ct_ops = {
    .read = odp_i2ct_read,
    .write = odp_i2ct_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ------------------------------------------------------------------ */
/* QOM glue                                                           */
/* ------------------------------------------------------------------ */

static void odp_i2ct_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    OdpI2CTargetState *s = ODP_I2C_TARGET(obj);

    memory_region_init_io(&s->iomem, obj, &odp_i2ct_ops, s,
                          TYPE_ODP_I2C_TARGET, ODP_I2CT_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void odp_i2ct_realize(DeviceState *dev, Error **errp)
{
    OdpI2CTargetState *s = ODP_I2C_TARGET(dev);

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, odp_i2ct_can_receive,
                                 odp_i2ct_receive, odp_i2ct_event,
                                 odp_i2ct_be_change, s, NULL, true);
    }
}

static void odp_i2ct_reset_hold(Object *obj, ResetType type)
{
    OdpI2CTargetState *s = ODP_I2C_TARGET(obj);

    odp_i2ct_do_reset(s, false);
}

static const Property odp_i2ct_properties[] = {
    DEFINE_PROP_CHR("chardev", OdpI2CTargetState, chr),
};

static void odp_i2ct_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = odp_i2ct_realize;
    dc->desc = "ODP socket-backed I2C target";
    rc->phases.hold = odp_i2ct_reset_hold;
    device_class_set_props(dc, odp_i2ct_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo odp_i2ct_info = {
    .name          = TYPE_ODP_I2C_TARGET,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OdpI2CTargetState),
    .instance_init = odp_i2ct_init,
    .class_init    = odp_i2ct_class_init,
};

static void odp_i2ct_register_types(void)
{
    type_register_static(&odp_i2ct_info);
}

type_init(odp_i2ct_register_types)
