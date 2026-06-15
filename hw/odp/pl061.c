/*
 * Socket-backed Arm PrimeCell PL061 General Purpose IO
 *
 * Based on hw/gpio/pl061.c (Copyright (c) 2007 CodeSourcery, written by
 * Paul Brook).  The MMIO register interface and interrupt semantics are
 * identical to the original PL061 so that an unmodified guest driver (e.g.
 * the Windows inbox ARMH0061 driver) binds and operates exactly as it would
 * against the in-tree pl061 device.
 *
 * The difference is purely in how the eight GPIO lines are bridged: instead
 * of internal qemu_irq wires, each line is connected to its own chardev
 * (typically "-chardev socket,...") so that a pin can be wired to a peer in a
 * separate QEMU instance, modelled on hw/odp/gpio.c.  A line configured as an
 * output pushes its level (a single byte, 0 or 1) over that pin's socket; a
 * byte arriving on a pin's socket updates that line's input level and drives
 * the interrupt logic exactly like a real wire toggling at the PL061's pin.
 *
 * This code is licensed under the GPL.
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
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "hw/odp/pl061.h"

static const uint8_t pl061_id[12] =
  { 0x00, 0x00, 0x00, 0x00, 0x61, 0x10, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1 };

#define N_GPIOS 8

typedef struct OdpPl061Pin {
    OdpPl061State *s;
    int index;
} OdpPl061Pin;

struct OdpPl061State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t locked;
    uint32_t data;
    uint32_t old_out_data;
    uint32_t old_in_data;
    uint32_t dir;
    uint32_t isense;
    uint32_t ibe;
    uint32_t iev;
    uint32_t im;
    uint32_t istate;
    uint32_t afsel;
    uint32_t dr2r;
    uint32_t dr4r;
    uint32_t dr8r;
    uint32_t odr;
    uint32_t pur;
    uint32_t pdr;
    uint32_t slr;
    uint32_t den;
    uint32_t cr;
    uint32_t amsel;
    qemu_irq irq;
    CharFrontend chr[N_GPIOS];          /* one peer socket per GPIO line */
    const unsigned char *id;
    /* Properties */
    uint8_t pullups;
    uint8_t pulldowns;

    /* Per-pin chardev callback context (carries the pin index). */
    OdpPl061Pin pin[N_GPIOS];
};

static const VMStateDescription vmstate_odp_pl061 = {
    .name = "odp-pl061",
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(locked, OdpPl061State),
        VMSTATE_UINT32(data, OdpPl061State),
        VMSTATE_UINT32(old_out_data, OdpPl061State),
        VMSTATE_UINT32(old_in_data, OdpPl061State),
        VMSTATE_UINT32(dir, OdpPl061State),
        VMSTATE_UINT32(isense, OdpPl061State),
        VMSTATE_UINT32(ibe, OdpPl061State),
        VMSTATE_UINT32(iev, OdpPl061State),
        VMSTATE_UINT32(im, OdpPl061State),
        VMSTATE_UINT32(istate, OdpPl061State),
        VMSTATE_UINT32(afsel, OdpPl061State),
        VMSTATE_UINT32(dr2r, OdpPl061State),
        VMSTATE_UINT32(dr4r, OdpPl061State),
        VMSTATE_UINT32(dr8r, OdpPl061State),
        VMSTATE_UINT32(odr, OdpPl061State),
        VMSTATE_UINT32(pur, OdpPl061State),
        VMSTATE_UINT32(pdr, OdpPl061State),
        VMSTATE_UINT32(slr, OdpPl061State),
        VMSTATE_UINT32(den, OdpPl061State),
        VMSTATE_UINT32(cr, OdpPl061State),
        VMSTATE_UINT32_V(amsel, OdpPl061State, 2),
        VMSTATE_END_OF_LIST()
    }
};

static uint8_t pl061_floating(OdpPl061State *s)
{
    /*
     * Return mask of bits which correspond to pins configured as inputs
     * and which are floating (neither pulled up to 1 nor down to 0).
     */
    uint8_t floating = ~(s->pullups | s->pulldowns);

    return floating & ~s->dir;
}

static uint8_t pl061_pullups(OdpPl061State *s)
{
    /*
     * Return mask of bits which correspond to pins configured as inputs
     * and which are pulled up to 1.
     */
    return s->pullups & ~s->dir;
}

static void pl061_update(OdpPl061State *s)
{
    uint8_t changed;
    uint8_t mask;
    uint8_t out;
    int i;
    uint8_t pullups = pl061_pullups(s);
    uint8_t floating = pl061_floating(s);

    /*
     * Pins configured as output are driven from the data register;
     * otherwise if they're pulled up they're 1, and if they're floating
     * then we give them the same value they had previously, so we don't
     * report any change to the other end.
     */
    out = (s->data & s->dir) | pullups | (s->old_out_data & floating);
    changed = s->old_out_data ^ out;
    if (changed) {
        s->old_out_data = out;
        for (i = 0; i < N_GPIOS; i++) {
            mask = 1 << i;
            if (changed & mask) {
                uint8_t level = (out & mask) != 0;
                /*
                 * Best effort: if no peer is attached the write is simply
                 * dropped.
                 */
                qemu_chr_fe_write_all(&s->chr[i], &level, 1);
            }
        }
    }

    /* Inputs */
    changed = (s->old_in_data ^ s->data) & ~s->dir;
    if (changed) {
        s->old_in_data = s->data;
        for (i = 0; i < N_GPIOS; i++) {
            mask = 1 << i;
            if (changed & mask) {
                if (!(s->isense & mask)) {
                    /* Edge interrupt */
                    if (s->ibe & mask) {
                        /* Any edge triggers the interrupt */
                        s->istate |= mask;
                    } else {
                        /* Edge is selected by IEV */
                        s->istate |= ~(s->data ^ s->iev) & mask;
                    }
                }
            }
        }
    }

    /* Level interrupt */
    s->istate |= ~(s->data ^ s->iev) & s->isense;

    qemu_set_irq(s->irq, (s->istate & s->im) != 0);
}

/*
 * Apply a new input level for a single line (driven by the line's peer
 * socket).  Mirrors the original pl061_set_irq() body: a line only registers
 * an input when it is configured as an input (dir bit clear).
 */
static void odp_pl061_set_input(OdpPl061State *s, int pin, bool level)
{
    uint8_t mask = 1 << pin;

    if ((s->dir & mask) == 0) {
        s->data &= ~mask;
        if (level) {
            s->data |= mask;
        }
        pl061_update(s);
    }
}

static uint64_t pl061_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    OdpPl061State *s = (OdpPl061State *)opaque;

    switch (offset) {
    case 0x0 ... 0x3ff: /* Data */
        return s->data & (offset >> 2);
    case 0x400: /* Direction */
        return s->dir;
    case 0x404: /* Interrupt sense */
        return s->isense;
    case 0x408: /* Interrupt both edges */
        return s->ibe;
    case 0x40c: /* Interrupt event */
        return s->iev;
    case 0x410: /* Interrupt mask */
        return s->im;
    case 0x414: /* Raw interrupt status */
        return s->istate;
    case 0x418: /* Masked interrupt status */
        return s->istate & s->im;
    case 0x420: /* Alternate function select */
        return s->afsel;
    case 0xfd0 ... 0xfff: /* ID registers */
        return s->id[(offset - 0xfd0) >> 2];
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "odp_pl061_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void pl061_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    OdpPl061State *s = (OdpPl061State *)opaque;
    uint8_t mask;

    switch (offset) {
    case 0 ... 0x3ff:
        mask = (offset >> 2) & s->dir;
        s->data = (s->data & ~mask) | (value & mask);
        pl061_update(s);
        return;
    case 0x400: /* Direction */
        s->dir = value & 0xff;
        break;
    case 0x404: /* Interrupt sense */
        s->isense = value & 0xff;
        break;
    case 0x408: /* Interrupt both edges */
        s->ibe = value & 0xff;
        break;
    case 0x40c: /* Interrupt event */
        s->iev = value & 0xff;
        break;
    case 0x410: /* Interrupt mask */
        s->im = value & 0xff;
        break;
    case 0x41c: /* Interrupt clear */
        s->istate &= ~value;
        break;
    case 0x420: /* Alternate function select */
        mask = s->cr;
        s->afsel = (s->afsel & ~mask) | (value & mask);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "odp_pl061_write: Bad offset %x\n", (int)offset);
        return;
    }
    pl061_update(s);
}

static void pl061_enter_reset(Object *obj, ResetType type)
{
    OdpPl061State *s = ODP_PL061(obj);

    /* reset values from PL061 TRM */
    s->data = 0;
    s->old_in_data = 0;
    s->dir = 0;
    s->isense = 0;
    s->ibe = 0;
    s->iev = 0;
    s->im = 0;
    s->istate = 0;
    s->afsel = 0;
    s->dr2r = 0xff;
    s->dr4r = 0;
    s->dr8r = 0;
    s->odr = 0;
    s->pur = 0;
    s->pdr = 0;
    s->slr = 0;
    s->den = 0;
    s->locked = 1;
    s->cr = 0xff;
    s->amsel = 0;
}

static void pl061_hold_reset(Object *obj, ResetType type)
{
    OdpPl061State *s = ODP_PL061(obj);
    int i;
    uint8_t floating = pl061_floating(s);
    uint8_t pullups = pl061_pullups(s);

    for (i = 0; i < N_GPIOS; i++) {
        uint8_t level;

        if (extract32(floating, i, 1)) {
            continue;
        }
        level = extract32(pullups, i, 1);
        qemu_chr_fe_write_all(&s->chr[i], &level, 1);
    }
    s->old_out_data = pullups;

    /*
     * Seed the input data register from the pull-ups so an undriven input
     * line reads its resting level until a peer drives it.  Without this an
     * active-low, pulled-up interrupt line (e.g. the HID-over-I2C attention
     * line on pin 0) would read 0 and appear permanently asserted the moment
     * the guest unmasks it, before the peer EC ever connects and drives it
     * high.  Keep old_in_data in sync so this resting level does not look
     * like an input edge.
     */
    s->data |= pullups;
    s->old_in_data = s->data;
}

/* ------------------------------------------------------------------ */
/* Socket receive (peer -> guest)                                     */
/* ------------------------------------------------------------------ */

static int odp_pl061_can_receive(void *opaque)
{
    /* Levels are applied immediately, so we can always accept input. */
    return N_GPIOS;
}

static void odp_pl061_receive(void *opaque, const uint8_t *buf, int size)
{
    OdpPl061Pin *p = opaque;
    int i;

    /* Each byte is a fresh level for this line; only the latest one matters. */
    for (i = 0; i < size; i++) {
        odp_pl061_set_input(p->s, p->index, buf[i] != 0);
    }
}

static int odp_pl061_be_change(void *opaque)
{
    OdpPl061Pin *p = opaque;

    qemu_chr_fe_set_handlers(&p->s->chr[p->index], odp_pl061_can_receive,
                             odp_pl061_receive, NULL, odp_pl061_be_change,
                             opaque, NULL, true);
    return 0;
}

static const MemoryRegionOps pl061_ops = {
    .read = pl061_read,
    .write = pl061_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void odp_pl061_init(Object *obj)
{
    OdpPl061State *s = ODP_PL061(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->id = pl061_id;

    memory_region_init_io(&s->iomem, obj, &pl061_ops, s, "odp-pl061", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void odp_pl061_realize(DeviceState *dev, Error **errp)
{
    OdpPl061State *s = ODP_PL061(dev);
    int i;

    if (s->pullups & s->pulldowns) {
        error_setg(errp, "no bit may be set both in pullups and pulldowns");
        return;
    }

    for (i = 0; i < N_GPIOS; i++) {
        s->pin[i].s = s;
        s->pin[i].index = i;

        if (qemu_chr_fe_backend_connected(&s->chr[i])) {
            qemu_chr_fe_set_handlers(&s->chr[i], odp_pl061_can_receive,
                                     odp_pl061_receive, NULL,
                                     odp_pl061_be_change, &s->pin[i], NULL,
                                     true);
        }
    }
}

static const Property pl061_props[] = {
    DEFINE_PROP_UINT8("pullups", OdpPl061State, pullups, 0xff),
    DEFINE_PROP_UINT8("pulldowns", OdpPl061State, pulldowns, 0x0),
    DEFINE_PROP_CHR("gpio0", OdpPl061State, chr[0]),
    DEFINE_PROP_CHR("gpio1", OdpPl061State, chr[1]),
    DEFINE_PROP_CHR("gpio2", OdpPl061State, chr[2]),
    DEFINE_PROP_CHR("gpio3", OdpPl061State, chr[3]),
    DEFINE_PROP_CHR("gpio4", OdpPl061State, chr[4]),
    DEFINE_PROP_CHR("gpio5", OdpPl061State, chr[5]),
    DEFINE_PROP_CHR("gpio6", OdpPl061State, chr[6]),
    DEFINE_PROP_CHR("gpio7", OdpPl061State, chr[7]),
};

static void odp_pl061_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_odp_pl061;
    dc->realize = odp_pl061_realize;
    dc->desc = "Socket-backed PL061 GPIO controller";
    device_class_set_props(dc, pl061_props);
    rc->phases.enter = pl061_enter_reset;
    rc->phases.hold = pl061_hold_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo odp_pl061_info = {
    .name          = TYPE_ODP_PL061,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OdpPl061State),
    .instance_init = odp_pl061_init,
    .class_init    = odp_pl061_class_init,
};

static void odp_pl061_register_types(void)
{
    type_register_static(&odp_pl061_info);
}

type_init(odp_pl061_register_types)
