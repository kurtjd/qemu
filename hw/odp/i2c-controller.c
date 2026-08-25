/*
 * ODP socket-backed I2C controller (master)
 *
 * A simple memory-mapped I2C *controller* that does not drive an in-process
 * QEMU I2CBus.  Instead it speaks a tiny framed protocol over a chardev
 * (typically "-chardev socket,...") to an external endpoint - e.g. another
 * QEMU VM acting as an embedded controller (EC).
 *
 * The guest drives three 32-bit registers (CTRL/STATUS/DATA) and the device
 * runs an I2C-like state machine, signalling progress back through status
 * bits and a single interrupt line.
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
#include "qemu/timer.h"
#include "qemu/fifo8.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qom/object.h"
#include "hw/odp/i2c-controller.h"

/* MMIO register map (32-bit registers). */
#define ODP_I2C_REG_CTRL    0x0
#define ODP_I2C_REG_STATUS  0x4
#define ODP_I2C_REG_DATA    0x8
#define ODP_I2C_MMIO_SIZE   0x1000

/* CTRL register bits. */
#define CTRL_RX_RDY_IE      (1u << 0) /* RX-ready interrupt enable     (R/W) */
#define CTRL_CMD_DONE_IE    (1u << 1) /* CMD-complete interrupt enable (R/W) */
#define CTRL_CLEAR_FIFO     (1u << 2) /* clear RX FIFO                 (W1C) */
#define CTRL_RESET          (1u << 3) /* soft reset, self-clearing     (W1)  */
#define CTRL_IE_MASK        (CTRL_RX_RDY_IE | CTRL_CMD_DONE_IE)

/* STATUS register bits. */
#define STATUS_RX_RDY       (1u << 0) /* RX FIFO not empty           (R)     */
#define STATUS_NAK          (1u << 1) /* target NAKed / link dropped (R/W1C) */
#define STATUS_PROTO_ERR    (1u << 2) /* illegal command ordering    (R/W1C) */
#define STATUS_CMD_DONE     (1u << 3) /* command completed           (R/W1C) */
#define STATUS_W1C_MASK     (STATUS_NAK | STATUS_PROTO_ERR | STATUS_CMD_DONE)

/* DATA register CMD field (bits 9:8). */
#define DATA_DATA_MASK      0xffu
#define DATA_CMD_SHIFT      8
#define DATA_CMD_MASK       0x3u
#define CMD_START           0x0
#define CMD_STOP            0x1
#define CMD_RX              0x2
#define CMD_TX              0x3

/* Wire protocol opcodes exchanged with the external endpoint. */
#define WIRE_START          0x42
#define WIRE_STOP           0x77
#define WIRE_DATA           0xDA
#define WIRE_ACK            0x01
#define WIRE_NAK            0x00

#define ODP_I2C_RX_FIFO_SIZE 256

typedef enum {
    ODP_I2C_IDLE,           /* waiting for guest to start a transfer        */
    ODP_I2C_WAIT_START_ACK, /* sent START+addr, waiting ACK/NAK from EC     */
    ODP_I2C_READY,          /* addressed, waiting for next guest command    */
    ODP_I2C_WAIT_TX_ACK,    /* sent a data byte, waiting ACK/NAK from EC    */
    ODP_I2C_RX,             /* receiving a burst of bytes from the EC       */
} OdpI2CState;

struct OdpI2CControllerState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
    CharBackend chr;

    /* Persisted register state. */
    uint32_t ctrl;   /* only the interrupt-enable bits live here */
    uint32_t status;

    /* State machine. */
    OdpI2CState state;
    bool addr_is_recv;   /* R/W bit of the most recent (re)START address */
    bool rx_burst_done;  /* an RX burst finished; must STOP or repeat-START */
    uint32_t rx_remaining;
    bool awaiting_payload; /* last wire opcode was DATA, next byte is payload */

    Fifo8 rx_fifo;
    QEMUTimer *timeout_timer;
    uint32_t timeout_ms;
};

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static void odp_i2c_update_irq(OdpI2CControllerState *s)
{
    bool level =
        ((s->status & STATUS_RX_RDY) && (s->ctrl & CTRL_RX_RDY_IE)) ||
        ((s->status & STATUS_CMD_DONE) && (s->ctrl & CTRL_CMD_DONE_IE));

    qemu_set_irq(s->irq, level);
}

static void odp_i2c_set_cmd_done(OdpI2CControllerState *s)
{
    s->status |= STATUS_CMD_DONE;
    odp_i2c_update_irq(s);
}

static void odp_i2c_arm_timeout(OdpI2CControllerState *s)
{
    timer_mod(s->timeout_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + s->timeout_ms);
}

static void odp_i2c_disarm_timeout(OdpI2CControllerState *s)
{
    timer_del(s->timeout_timer);
}

static void odp_i2c_wire_send(OdpI2CControllerState *s,
                              const uint8_t *buf, int len)
{
    /* Best effort: if the backend is gone the timeout/CLOSED path recovers. */
    qemu_chr_fe_write_all(&s->chr, buf, len);
}

/*
 * Reset the controller back to the idle state.  Shared by the CTRL soft-reset
 * bit and the QOM cold-reset path.  Soft reset keeps the interrupt-enable
 * bits; cold reset clears everything.
 */
static void odp_i2c_do_reset(OdpI2CControllerState *s, bool keep_int_enables)
{
    odp_i2c_disarm_timeout(s);
    fifo8_reset(&s->rx_fifo);

    s->status = 0;
    s->state = ODP_I2C_IDLE;
    s->addr_is_recv = false;
    s->rx_burst_done = false;
    s->rx_remaining = 0;
    s->awaiting_payload = false;

    if (!keep_int_enables) {
        s->ctrl = 0;
    }

    qemu_set_irq(s->irq, 0);
}

/* ------------------------------------------------------------------ */
/* Command dispatch (guest -> EC)                                     */
/* ------------------------------------------------------------------ */

static void odp_i2c_proto_error(OdpI2CControllerState *s)
{
    s->status |= STATUS_PROTO_ERR;
    s->state = ODP_I2C_IDLE;
}

static void odp_i2c_do_start(OdpI2CControllerState *s, uint8_t addr)
{
    uint8_t frame[2] = { WIRE_START, addr };

    s->addr_is_recv = addr & 0x1;
    s->rx_burst_done = false;
    odp_i2c_wire_send(s, frame, sizeof(frame));
    s->state = ODP_I2C_WAIT_START_ACK;
    odp_i2c_arm_timeout(s);
}

static void odp_i2c_do_tx(OdpI2CControllerState *s, uint8_t byte)
{
    uint8_t frame[2] = { WIRE_DATA, byte };

    odp_i2c_wire_send(s, frame, sizeof(frame));
    s->state = ODP_I2C_WAIT_TX_ACK;
    odp_i2c_arm_timeout(s);
}

static void odp_i2c_do_rx(OdpI2CControllerState *s, uint8_t count_minus_one)
{
    s->rx_remaining = (uint32_t)count_minus_one + 1;
    s->state = ODP_I2C_RX;
    /* The chardev may have been throttled; make sure we can take bytes. */
    qemu_chr_fe_accept_input(&s->chr);
}

static void odp_i2c_do_stop(OdpI2CControllerState *s)
{
    uint8_t op = WIRE_STOP;

    odp_i2c_wire_send(s, &op, 1);
    s->rx_burst_done = false;
    s->state = ODP_I2C_IDLE;
    odp_i2c_set_cmd_done(s);
}

static void odp_i2c_dispatch_cmd(OdpI2CControllerState *s,
                                 uint8_t cmd, uint8_t data)
{
    switch (s->state) {
    case ODP_I2C_IDLE:
        if (cmd == CMD_START) {
            odp_i2c_do_start(s, data);
        } else {
            /* Must open with a START. */
            s->status |= STATUS_PROTO_ERR;
        }
        break;

    case ODP_I2C_READY:
        switch (cmd) {
        case CMD_STOP:
            odp_i2c_do_stop(s);
            break;
        case CMD_START: /* repeated start */
            odp_i2c_do_start(s, data);
            break;
        case CMD_RX:
            if (s->addr_is_recv && !s->rx_burst_done) {
                odp_i2c_do_rx(s, data);
            } else {
                odp_i2c_proto_error(s);
            }
            break;
        case CMD_TX:
            if (!s->addr_is_recv) {
                odp_i2c_do_tx(s, data);
            } else {
                odp_i2c_proto_error(s);
            }
            break;
        default:
            g_assert_not_reached();
        }
        break;

    case ODP_I2C_WAIT_START_ACK:
    case ODP_I2C_WAIT_TX_ACK:
    case ODP_I2C_RX:
        /* Guest issued a command while a transfer was still in flight. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: command written while controller busy\n",
                      TYPE_ODP_I2C_CONTROLLER);
        s->status |= STATUS_PROTO_ERR;
        break;

    default:
        g_assert_not_reached();
    }
}

/* ------------------------------------------------------------------ */
/* Socket receive (EC -> guest)                                       */
/* ------------------------------------------------------------------ */

static void odp_i2c_handle_ack(OdpI2CControllerState *s, bool ack)
{
    if (s->state != ODP_I2C_WAIT_START_ACK &&
        s->state != ODP_I2C_WAIT_TX_ACK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unexpected ACK/NAK in state %d\n",
                      TYPE_ODP_I2C_CONTROLLER, s->state);
        return;
    }

    odp_i2c_disarm_timeout(s);

    if (ack) {
        s->state = ODP_I2C_READY;
    } else {
        s->status |= STATUS_NAK;
        s->state = ODP_I2C_IDLE;
    }
    odp_i2c_set_cmd_done(s);
}

static void odp_i2c_handle_rx_byte(OdpI2CControllerState *s, uint8_t byte)
{
    uint8_t reply;

    if (s->state != ODP_I2C_RX) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unexpected DATA in state %d\n",
                      TYPE_ODP_I2C_CONTROLLER, s->state);
        return;
    }

    if (fifo8_is_full(&s->rx_fifo)) {
        /* Should not happen while can_receive throttles us, but be safe. */
        s->status |= STATUS_PROTO_ERR;
        reply = WIRE_NAK;
        odp_i2c_wire_send(s, &reply, 1);
        s->rx_burst_done = true;
        s->state = ODP_I2C_READY;
        odp_i2c_set_cmd_done(s);
        return;
    }

    if (fifo8_is_empty(&s->rx_fifo)) {
        s->status |= STATUS_RX_RDY;
    }
    fifo8_push(&s->rx_fifo, byte);
    s->rx_remaining--;

    if (s->rx_remaining > 0) {
        reply = WIRE_ACK;
        odp_i2c_wire_send(s, &reply, 1);
    } else {
        reply = WIRE_NAK;
        odp_i2c_wire_send(s, &reply, 1);
        s->rx_burst_done = true;
        s->state = ODP_I2C_READY;
        odp_i2c_set_cmd_done(s);
    }

    odp_i2c_update_irq(s);
}

static int odp_i2c_can_receive(void *opaque)
{
    OdpI2CControllerState *s = opaque;

    switch (s->state) {
    case ODP_I2C_WAIT_START_ACK:
    case ODP_I2C_WAIT_TX_ACK:
        return 1; /* expecting a single ACK/NAK opcode */
    case ODP_I2C_RX:
        /* Need room for a DATA opcode plus its payload byte. */
        return fifo8_is_full(&s->rx_fifo) ? 0 : 2;
    default:
        return 0;
    }
}

static void odp_i2c_receive(void *opaque, const uint8_t *buf, int size)
{
    OdpI2CControllerState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        uint8_t b = buf[i];

        if (s->awaiting_payload) {
            s->awaiting_payload = false;
            odp_i2c_handle_rx_byte(s, b);
            continue;
        }

        switch (b) {
        case WIRE_DATA:
            s->awaiting_payload = true;
            break;
        case WIRE_ACK:
            odp_i2c_handle_ack(s, true);
            break;
        case WIRE_NAK:
            odp_i2c_handle_ack(s, false);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: unknown wire opcode 0x%02x\n",
                          TYPE_ODP_I2C_CONTROLLER, b);
            break;
        }
    }
}

static void odp_i2c_event(void *opaque, QEMUChrEvent event)
{
    OdpI2CControllerState *s = opaque;

    if (event == CHR_EVENT_CLOSED && s->state != ODP_I2C_IDLE) {
        /* Link dropped mid-transfer: fail the outstanding command. */
        odp_i2c_disarm_timeout(s);
        s->status |= STATUS_NAK;
        s->state = ODP_I2C_IDLE;
        s->awaiting_payload = false;
        odp_i2c_set_cmd_done(s);
    }
}

static void odp_i2c_timeout(void *opaque)
{
    OdpI2CControllerState *s = opaque;

    if (s->state == ODP_I2C_WAIT_START_ACK ||
        s->state == ODP_I2C_WAIT_TX_ACK) {
        s->status |= STATUS_NAK;
        s->state = ODP_I2C_IDLE;
        odp_i2c_set_cmd_done(s);
    }
}

static int odp_i2c_be_change(void *opaque)
{
    OdpI2CControllerState *s = opaque;

    qemu_chr_fe_set_handlers(&s->chr, odp_i2c_can_receive, odp_i2c_receive,
                             odp_i2c_event, odp_i2c_be_change, s, NULL, true);
    return 0;
}

/* ------------------------------------------------------------------ */
/* MMIO                                                               */
/* ------------------------------------------------------------------ */

static uint64_t odp_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    OdpI2CControllerState *s = opaque;
    uint8_t byte;

    switch (offset) {
    case ODP_I2C_REG_CTRL:
        return s->ctrl & CTRL_IE_MASK;

    case ODP_I2C_REG_STATUS:
        return s->status;

    case ODP_I2C_REG_DATA:
        if (fifo8_is_empty(&s->rx_fifo)) {
            return 0;
        }
        byte = fifo8_pop(&s->rx_fifo);
        if (fifo8_is_empty(&s->rx_fifo)) {
            s->status &= ~STATUS_RX_RDY;
            odp_i2c_update_irq(s);
        }
        /* Room freed up: let the chardev resume delivering RX bytes. */
        qemu_chr_fe_accept_input(&s->chr);
        return byte;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_ODP_I2C_CONTROLLER, offset);
        return 0;
    }
}

static void odp_i2c_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    OdpI2CControllerState *s = opaque;

    switch (offset) {
    case ODP_I2C_REG_CTRL:
        if (value & CTRL_RESET) {
            /* Soft reset overrides everything else in this write. */
            odp_i2c_do_reset(s, true);
            break;
        }
        s->ctrl = value & CTRL_IE_MASK;
        if (value & CTRL_CLEAR_FIFO) {
            fifo8_reset(&s->rx_fifo);
            s->status &= ~STATUS_RX_RDY;
            qemu_chr_fe_accept_input(&s->chr);
        }
        odp_i2c_update_irq(s);
        break;

    case ODP_I2C_REG_STATUS:
        /* Write-1-to-clear the latched status bits. */
        s->status &= ~(value & STATUS_W1C_MASK);
        odp_i2c_update_irq(s);
        break;

    case ODP_I2C_REG_DATA: {
        uint8_t cmd = (value >> DATA_CMD_SHIFT) & DATA_CMD_MASK;
        uint8_t data = value & DATA_DATA_MASK;

        odp_i2c_dispatch_cmd(s, cmd, data);
        break;
    }

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_ODP_I2C_CONTROLLER, offset);
        break;
    }
}

static const MemoryRegionOps odp_i2c_ops = {
    .read = odp_i2c_read,
    .write = odp_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ------------------------------------------------------------------ */
/* QOM glue                                                           */
/* ------------------------------------------------------------------ */

static void odp_i2c_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    OdpI2CControllerState *s = ODP_I2C_CONTROLLER(obj);

    memory_region_init_io(&s->iomem, obj, &odp_i2c_ops, s,
                          TYPE_ODP_I2C_CONTROLLER, ODP_I2C_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void odp_i2c_realize(DeviceState *dev, Error **errp)
{
    OdpI2CControllerState *s = ODP_I2C_CONTROLLER(dev);

    fifo8_create(&s->rx_fifo, ODP_I2C_RX_FIFO_SIZE);
    s->timeout_timer = timer_new_ms(QEMU_CLOCK_REALTIME, odp_i2c_timeout, s);

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, odp_i2c_can_receive, odp_i2c_receive,
                                 odp_i2c_event, odp_i2c_be_change, s, NULL,
                                 true);
    }
}

static void odp_i2c_unrealize(DeviceState *dev)
{
    OdpI2CControllerState *s = ODP_I2C_CONTROLLER(dev);

    timer_free(s->timeout_timer);
    fifo8_destroy(&s->rx_fifo);
}

static void odp_i2c_reset_hold(Object *obj, ResetType type)
{
    OdpI2CControllerState *s = ODP_I2C_CONTROLLER(obj);

    odp_i2c_do_reset(s, false);
}

static const Property odp_i2c_properties[] = {
    DEFINE_PROP_CHR("chardev", OdpI2CControllerState, chr),
    DEFINE_PROP_UINT32("timeout-ms", OdpI2CControllerState, timeout_ms, 100),
};

static void odp_i2c_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = odp_i2c_realize;
    dc->unrealize = odp_i2c_unrealize;
    dc->desc = "ODP socket-backed I2C controller";
    rc->phases.hold = odp_i2c_reset_hold;
    device_class_set_props(dc, odp_i2c_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo odp_i2c_info = {
    .name          = TYPE_ODP_I2C_CONTROLLER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OdpI2CControllerState),
    .instance_init = odp_i2c_init,
    .class_init    = odp_i2c_class_init,
};

static void odp_i2c_register_types(void)
{
    type_register_static(&odp_i2c_info);
}

type_init(odp_i2c_register_types)
