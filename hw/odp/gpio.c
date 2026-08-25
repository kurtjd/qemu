/*
 * ODP socket-backed virtual GPIO controller
 *
 * A simple memory-mapped, bidirectional GPIO controller.  Each of the 32 pins
 * is bridged to a peer (typically another QEMU VM) over its own chardev
 * (usually "-chardev socket,...").  A pin is bidirectional: the guest drives an
 * output level through the OUT register and observes the peer's level through
 * the IN register, exactly like a real wire between two devices.
 *
 * Whenever the guest changes an OUT bit, the new level (a single byte, 0 or 1)
 * is pushed over that pin's socket.  Whenever a byte arrives on a pin's socket,
 * the corresponding IN bit is updated and, if the change matches the pin's
 * configured trigger/polarity and the pin's interrupt is enabled, the shared
 * interrupt line is asserted.  Software reads IRQ_PEND to discover which pins
 * tripped.
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
#include "hw/odp/gpio.h"

/* MMIO register map (32-bit registers). */
#define ODP_GPIO_REG_IN        0x00 /* input level                 (R)     */
#define ODP_GPIO_REG_OUT       0x04 /* output level                (R/W)   */
#define ODP_GPIO_REG_IRQ_TRIG  0x08 /* 0 = level, 1 = edge         (R/W)   */
#define ODP_GPIO_REG_IRQ_POL   0x0c /* 0 = low/falling, 1 = high/rising (R/W) */
#define ODP_GPIO_REG_IRQ_EN    0x10 /* per-pin interrupt enable    (R/W)   */
#define ODP_GPIO_REG_IRQ_PEND  0x14 /* per-pin interrupt pending   (R/W1C) */
#define ODP_GPIO_MMIO_SIZE     0x1000

#define ODP_GPIO_NUM_PINS      32

typedef struct OdpGpioPin {
    OdpGpioState *s;
    int index;
} OdpGpioPin;

struct OdpGpioState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;                          /* shared output line       */
    CharBackend chr[ODP_GPIO_NUM_PINS];   /* one peer socket per pin  */

    /* Register state (bit per pin). */
    uint32_t in;
    uint32_t out;
    uint32_t irq_trig;
    uint32_t irq_pol;
    uint32_t irq_en;
    uint32_t irq_pend;

    /* Per-pin chardev callback context (carries the pin index). */
    OdpGpioPin pin[ODP_GPIO_NUM_PINS];
};

/* ------------------------------------------------------------------ */
/* IRQ evaluation                                                     */
/* ------------------------------------------------------------------ */

/*
 * Recompute the pending bits for the level-triggered pins and refresh the
 * shared interrupt line.
 *
 * Edge-triggered pending bits are latched at the moment the matching edge is
 * seen (see odp_gpio_set_input) and stay set until the guest clears them with a
 * write-1-to-clear to IRQ_PEND.  Level-triggered pending bits, in contrast,
 * always reflect the current input level versus the configured polarity, so a
 * level interrupt re-asserts immediately after a W1C if the level is still
 * active - matching real level-sensitive hardware.
 */
static void odp_gpio_update_irq(OdpGpioState *s)
{
    uint32_t level_pins = ~s->irq_trig;
    /* A level pin is active when its input matches its polarity (XNOR). */
    uint32_t level_active = ~(s->in ^ s->irq_pol) & level_pins;

    s->irq_pend = (s->irq_pend & ~level_pins) | level_active;

    qemu_set_irq(s->irq, (s->irq_pend & s->irq_en) != 0);
}

/*
 * Apply a new input level for a single pin (driven by the pin's peer socket),
 * latching an edge-triggered pending bit if the transition matches the pin's
 * configured edge, then refreshing the interrupt state.
 */
static void odp_gpio_set_input(OdpGpioState *s, int pin, bool level)
{
    uint32_t mask = 1u << pin;
    bool old = (s->in & mask) != 0;

    if (old == level) {
        return;
    }

    if (level) {
        s->in |= mask;
    } else {
        s->in &= ~mask;
    }

    /* Edge-triggered: latch on the matching edge (POL=1 rising, POL=0 falling). */
    if (s->irq_trig & mask) {
        bool want_rising = (s->irq_pol & mask) != 0;
        if (level == want_rising) {
            s->irq_pend |= mask;
        }
    }

    odp_gpio_update_irq(s);
}

/* ------------------------------------------------------------------ */
/* Output (guest -> peer sockets)                                     */
/* ------------------------------------------------------------------ */

static void odp_gpio_out_write(OdpGpioState *s, uint32_t value)
{
    uint32_t changed = s->out ^ value;
    int pin;

    s->out = value;

    for (pin = 0; pin < ODP_GPIO_NUM_PINS; pin++) {
        if (changed & (1u << pin)) {
            uint8_t b = (value >> pin) & 1u;
            /* Best effort: if no peer is attached the write is simply dropped. */
            qemu_chr_fe_write_all(&s->chr[pin], &b, 1);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Socket receive (peer -> guest)                                     */
/* ------------------------------------------------------------------ */

static int odp_gpio_can_receive(void *opaque)
{
    /* Levels are applied immediately, so we can always accept input. */
    return ODP_GPIO_NUM_PINS;
}

static void odp_gpio_receive(void *opaque, const uint8_t *buf, int size)
{
    OdpGpioPin *p = opaque;
    int i;

    /* Each byte is a fresh level for this pin; only the latest one matters. */
    for (i = 0; i < size; i++) {
        odp_gpio_set_input(p->s, p->index, buf[i] != 0);
    }
}

static int odp_gpio_be_change(void *opaque)
{
    OdpGpioPin *p = opaque;

    qemu_chr_fe_set_handlers(&p->s->chr[p->index], odp_gpio_can_receive,
                             odp_gpio_receive, NULL, odp_gpio_be_change,
                             opaque, NULL, true);
    return 0;
}

/* ------------------------------------------------------------------ */
/* MMIO                                                               */
/* ------------------------------------------------------------------ */

static uint64_t odp_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    OdpGpioState *s = opaque;

    switch (offset) {
    case ODP_GPIO_REG_IN:
        return s->in;
    case ODP_GPIO_REG_OUT:
        return s->out;
    case ODP_GPIO_REG_IRQ_TRIG:
        return s->irq_trig;
    case ODP_GPIO_REG_IRQ_POL:
        return s->irq_pol;
    case ODP_GPIO_REG_IRQ_EN:
        return s->irq_en;
    case ODP_GPIO_REG_IRQ_PEND:
        return s->irq_pend;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_ODP_GPIO, offset);
        return 0;
    }
}

static void odp_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    OdpGpioState *s = opaque;

    switch (offset) {
    case ODP_GPIO_REG_IN:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only IN register\n",
                      TYPE_ODP_GPIO);
        break;

    case ODP_GPIO_REG_OUT:
        odp_gpio_out_write(s, value);
        break;

    case ODP_GPIO_REG_IRQ_TRIG:
        s->irq_trig = value;
        odp_gpio_update_irq(s);
        break;

    case ODP_GPIO_REG_IRQ_POL:
        s->irq_pol = value;
        odp_gpio_update_irq(s);
        break;

    case ODP_GPIO_REG_IRQ_EN:
        s->irq_en = value;
        odp_gpio_update_irq(s);
        break;

    case ODP_GPIO_REG_IRQ_PEND:
        /* Write-1-to-clear; level pins re-assert in update if still active. */
        s->irq_pend &= ~(uint32_t)value;
        odp_gpio_update_irq(s);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_ODP_GPIO, offset);
        break;
    }
}

static const MemoryRegionOps odp_gpio_ops = {
    .read = odp_gpio_read,
    .write = odp_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ------------------------------------------------------------------ */
/* QOM glue                                                           */
/* ------------------------------------------------------------------ */

static void odp_gpio_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    OdpGpioState *s = ODP_GPIO(obj);

    memory_region_init_io(&s->iomem, obj, &odp_gpio_ops, s,
                          TYPE_ODP_GPIO, ODP_GPIO_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void odp_gpio_realize(DeviceState *dev, Error **errp)
{
    OdpGpioState *s = ODP_GPIO(dev);
    int i;

    for (i = 0; i < ODP_GPIO_NUM_PINS; i++) {
        s->pin[i].s = s;
        s->pin[i].index = i;

        if (qemu_chr_fe_backend_connected(&s->chr[i])) {
            qemu_chr_fe_set_handlers(&s->chr[i], odp_gpio_can_receive,
                                     odp_gpio_receive, NULL,
                                     odp_gpio_be_change, &s->pin[i], NULL,
                                     true);
        }
    }
}

static void odp_gpio_reset_hold(Object *obj, ResetType type)
{
    OdpGpioState *s = ODP_GPIO(obj);

    s->in = 0;
    s->out = 0;
    s->irq_trig = 0;
    s->irq_pol = 0;
    s->irq_en = 0;
    s->irq_pend = 0;

    qemu_set_irq(s->irq, 0);
}

static const Property odp_gpio_properties[] = {
    DEFINE_PROP_CHR("gpio0",  OdpGpioState, chr[0]),
    DEFINE_PROP_CHR("gpio1",  OdpGpioState, chr[1]),
    DEFINE_PROP_CHR("gpio2",  OdpGpioState, chr[2]),
    DEFINE_PROP_CHR("gpio3",  OdpGpioState, chr[3]),
    DEFINE_PROP_CHR("gpio4",  OdpGpioState, chr[4]),
    DEFINE_PROP_CHR("gpio5",  OdpGpioState, chr[5]),
    DEFINE_PROP_CHR("gpio6",  OdpGpioState, chr[6]),
    DEFINE_PROP_CHR("gpio7",  OdpGpioState, chr[7]),
    DEFINE_PROP_CHR("gpio8",  OdpGpioState, chr[8]),
    DEFINE_PROP_CHR("gpio9",  OdpGpioState, chr[9]),
    DEFINE_PROP_CHR("gpio10", OdpGpioState, chr[10]),
    DEFINE_PROP_CHR("gpio11", OdpGpioState, chr[11]),
    DEFINE_PROP_CHR("gpio12", OdpGpioState, chr[12]),
    DEFINE_PROP_CHR("gpio13", OdpGpioState, chr[13]),
    DEFINE_PROP_CHR("gpio14", OdpGpioState, chr[14]),
    DEFINE_PROP_CHR("gpio15", OdpGpioState, chr[15]),
    DEFINE_PROP_CHR("gpio16", OdpGpioState, chr[16]),
    DEFINE_PROP_CHR("gpio17", OdpGpioState, chr[17]),
    DEFINE_PROP_CHR("gpio18", OdpGpioState, chr[18]),
    DEFINE_PROP_CHR("gpio19", OdpGpioState, chr[19]),
    DEFINE_PROP_CHR("gpio20", OdpGpioState, chr[20]),
    DEFINE_PROP_CHR("gpio21", OdpGpioState, chr[21]),
    DEFINE_PROP_CHR("gpio22", OdpGpioState, chr[22]),
    DEFINE_PROP_CHR("gpio23", OdpGpioState, chr[23]),
    DEFINE_PROP_CHR("gpio24", OdpGpioState, chr[24]),
    DEFINE_PROP_CHR("gpio25", OdpGpioState, chr[25]),
    DEFINE_PROP_CHR("gpio26", OdpGpioState, chr[26]),
    DEFINE_PROP_CHR("gpio27", OdpGpioState, chr[27]),
    DEFINE_PROP_CHR("gpio28", OdpGpioState, chr[28]),
    DEFINE_PROP_CHR("gpio29", OdpGpioState, chr[29]),
    DEFINE_PROP_CHR("gpio30", OdpGpioState, chr[30]),
    DEFINE_PROP_CHR("gpio31", OdpGpioState, chr[31]),
};

static void odp_gpio_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = odp_gpio_realize;
    dc->desc = "ODP socket-backed virtual GPIO controller";
    rc->phases.hold = odp_gpio_reset_hold;
    device_class_set_props(dc, odp_gpio_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo odp_gpio_info = {
    .name          = TYPE_ODP_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OdpGpioState),
    .instance_init = odp_gpio_init,
    .class_init    = odp_gpio_class_init,
};

static void odp_gpio_register_types(void)
{
    type_register_static(&odp_gpio_info);
}

type_init(odp_gpio_register_types)
