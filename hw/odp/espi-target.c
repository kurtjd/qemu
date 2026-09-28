#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"
#include "hw/irq.h"
#include "hw/odp/espi-proto.h"
#include "hw/odp/espi-target.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"

#define ESPI_REG_CONTROL         0x00
#define ESPI_REG_MAILBOX0_ADDR   0x04
#define ESPI_REG_MAILBOX1_ADDR   0x08
#define ESPI_REG_CONTROL_ADDR    0x0c
#define ESPI_REG_MAILBOX_CONTROL 0x10
#define ESPI_REG_MAILBOX_STRIDE  0x0c
#define ESPI_REG_SIZE            0x1000
#define ESPI_MAILBOX_COUNT       2
#define ESPI_MAILBOX_SIZE        4096
#define ESPI_MAX_PAYLOAD         64
#define ESPI_MAILBOX_CONTROL_SIZE 0x1000
#define ESPI_MAILBOX_CONTROL_STRIDE 0x20
#define ESPI_MAILBOX_SHARED_STATUS 0x00
#define ESPI_MAILBOX_DOORBELL      0x04
#define ESPI_MAILBOX_IRQ_SET       0x08
#define ESPI_MAILBOX_IRQ_ACK       0x0c

#define ESPI_CONTROL_ENABLE     (1U << 0)
#define ESPI_ERROR_CRC          (1U << 0)
#define ESPI_ERROR_PROTOCOL     (1U << 1)
#define ESPI_ERROR_ADDRESS      (1U << 2)
#define ESPI_ERROR_PERMISSION   (1U << 3)
#define ESPI_ERROR_LINK         (1U << 4)
#define ESPI_ERROR_CONFIG       (1U << 5)
#define ESPI_ERROR_ABORT        (1U << 6)
#define ESPI_ERROR_LENGTH       (1U << 7)
#define ESPI_ERROR_UNSUPPORTED  (1U << 8)
#define ESPI_ERROR_BUSY         (1U << 9)
#define ESPI_ERROR_TIMEOUT      (1U << 10)
#define ESPI_CH0_RESET          (ODP_ESPI_CH0_ENABLE | (1U << 4) | \
                                (1U << 8) | (1U << 12))
#define ESPI_VWIRE_COUNT        ODP_ESPI_VWIRE_MIN_COUNT
#define ESPI_VWIRE_QUEUE_SIZE   64
#define ESPI_HOST_IRQ_MASK      ((1U << ESPI_MAILBOX_COUNT) - 1)
#define ESPI_CH1_RESET          ((ESPI_VWIRE_COUNT - 1) << \
                                ODP_ESPI_CH1_MAX_COUNT_SUPPORTED_SHIFT)
#define ESPI_STATUS_FIXED_FREE  (ODP_ESPI_STATUS_VWIRE_FREE | (1U << 8))
#define ESPI_GENERAL_SUPPORTED  (ODP_ESPI_GENERAL_CHANNEL0_SUPPORTED | \
                                ODP_ESPI_GENERAL_CHANNEL1_SUPPORTED)
#define ESPI_GENERAL_WRITABLE   (ODP_ESPI_GENERAL_CRC_ENABLE | \
                                (1U << 28) | (0xfU << 12))
#define ESPI_GENERAL_UNSUPPORTED ((1U << 30) | (3U << 26) | \
                                  (1U << 23) | (7U << 20))

typedef enum OdpESPITargetRxState {
    ESPI_RX_KIND,
    ESPI_RX_DATA,
    ESPI_RX_RESET,
} OdpESPITargetRxState;

typedef struct OdpESPIMailbox {
    MemoryRegion iomem;
    uint8_t data[ESPI_MAILBOX_SIZE];
    uint32_t base;
    uint32_t shared_status;
    bool doorbell;
} OdpESPIMailbox;

struct OdpESPITargetState {
    SysBusDevice parent_obj;
    CharBackend chr;
    MemoryRegion iomem;
    qemu_irq irq;
    OdpESPIMailbox mailboxes[ESPI_MAILBOX_COUNT];
    uint32_t control;
    uint32_t errors;
    uint32_t general;
    uint32_t channel;
    uint32_t vwire_channel;
    uint32_t host_irq;
    uint32_t mailbox_control_base;
    uint8_t vwire_queue[ESPI_VWIRE_QUEUE_SIZE][ODP_ESPI_VWIRE_GROUP_SIZE];
    unsigned vwire_head;
    unsigned vwire_count;
    unsigned tx_vwire_count;
    bool connected;
    bool external_reset;
    OdpESPITargetRxState rx_state;
    uint8_t rx[ODP_ESPI_PACKET_LENGTH_MAX + 9];
    unsigned rx_size;
    unsigned rx_needed;
    uint8_t tx[ESPI_MAX_PAYLOAD + 8];
    unsigned tx_size;
    unsigned tx_pos;
    uint16_t config_address;
    uint32_t config_value;
    uint16_t reported_status;
    uint16_t alerted_status;
    guint write_watch;
    QEMUTimer *timer;
    uint32_t timeout_ms;
};

static void odp_espit_alert(OdpESPITargetState *s);

static uint32_t odp_espit_irq_status(OdpESPITargetState *s)
{
    uint32_t status = 0;
    unsigned index;

    for (index = 0; index < ESPI_MAILBOX_COUNT; index++) {
        if (s->mailboxes[index].doorbell) {
            status |= 1U << index;
        }
    }
    return status;
}

static void odp_espit_update_irq(OdpESPITargetState *s)
{
    qemu_set_irq(s->irq, odp_espit_irq_status(s) != 0);
}

static void odp_espit_set_error(OdpESPITargetState *s, uint32_t error)
{
    s->errors |= error;
}

static uint8_t odp_espit_crc8(const uint8_t *data, unsigned length)
{
    uint8_t crc = ODP_ESPI_CRC8_INIT;
    unsigned index, bit;

    for (index = 0; index < length; index++) {
        crc ^= data[index];
        for (bit = 0; bit < 8; bit++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? ODP_ESPI_CRC8_POLYNOMIAL : 0);
        }
    }
    return crc;
}

static bool odp_espit_pending(OdpESPITargetState *s)
{
    return s->rx_state != ESPI_RX_KIND || s->tx_size;
}

static void odp_espit_clear_io(OdpESPITargetState *s)
{
    if (s->write_watch) {
        g_source_remove(s->write_watch);
        s->write_watch = 0;
    }
    timer_del(s->timer);
    s->rx_state = ESPI_RX_KIND;
    s->rx_size = 0;
    s->rx_needed = 0;
    s->tx_size = 0;
    s->tx_pos = 0;
    s->vwire_head = 0;
    s->vwire_count = 0;
    s->tx_vwire_count = 0;
    s->config_address = 0;
    s->reported_status = ESPI_STATUS_FIXED_FREE;
    s->alerted_status = ESPI_STATUS_FIXED_FREE;
}

static void odp_espit_disconnect(OdpESPITargetState *s, uint32_t error)
{
    odp_espit_set_error(s, error |
                       (odp_espit_pending(s) ? ESPI_ERROR_ABORT : 0));
    odp_espit_clear_io(s);
    qemu_chr_fe_disconnect(&s->chr);
}

static void odp_espit_timeout(void *opaque)
{
    odp_espit_disconnect(opaque, ESPI_ERROR_TIMEOUT);
}

static void odp_espit_arm_timer(OdpESPITargetState *s)
{
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + s->timeout_ms);
}

static bool odp_espit_ready(OdpESPITargetState *s)
{
    return s->connected && !s->external_reset &&
           (s->control & ESPI_CONTROL_ENABLE) &&
           (s->channel & ODP_ESPI_CH0_ENABLE);
}

static uint32_t odp_espit_channel(OdpESPITargetState *s)
{
    return s->channel | (odp_espit_ready(s) ? ODP_ESPI_CH0_READY : 0);
}

static bool odp_espit_vwire_ready(OdpESPITargetState *state)
{
    return state->connected && !state->external_reset &&
           (state->vwire_channel & ODP_ESPI_CH1_ENABLE);
}

static uint32_t odp_espit_vwire_channel(OdpESPITargetState *state)
{
    return state->vwire_channel |
           (odp_espit_vwire_ready(state) ? ODP_ESPI_CH1_READY : 0);
}

static void odp_espit_queue_vwire(OdpESPITargetState *state, uint8_t index,
                                  uint8_t value)
{
    unsigned tail;

    if (!odp_espit_vwire_ready(state)) {
        return;
    }
    if (state->vwire_count == ESPI_VWIRE_QUEUE_SIZE) {
        odp_espit_disconnect(state, ESPI_ERROR_BUSY);
        return;
    }
    tail = (state->vwire_head + state->vwire_count) % ESPI_VWIRE_QUEUE_SIZE;
    state->vwire_queue[tail][0] = index;
    state->vwire_queue[tail][1] = value;
    state->vwire_count++;
}

static void odp_espit_host_irq(OdpESPITargetState *state, uint32_t pending)
{
    uint32_t previous = state->host_irq;
    unsigned index;

    state->host_irq = pending & ESPI_HOST_IRQ_MASK;
    for (index = 0; index < ESPI_MAILBOX_COUNT; index++) {
        if ((previous ^ state->host_irq) & (1U << index)) {
            odp_espit_queue_vwire(state, ODP_ESPI_VWIRE_IRQ_LOW,
                                  index | ((state->host_irq & (1U << index)) ?
                                           ODP_ESPI_VWIRE_IRQ_LEVEL : 0));
        }
    }
}

static void odp_espit_sync_vwires(OdpESPITargetState *state)
{
    unsigned index;

    state->vwire_head = 0;
    state->vwire_count = 0;
    for (index = 0; index < ESPI_MAILBOX_COUNT; index++) {
        odp_espit_queue_vwire(state, ODP_ESPI_VWIRE_IRQ_LOW,
                              index | ((state->host_irq &
                                        (1U << index)) ?
                                       ODP_ESPI_VWIRE_IRQ_LEVEL : 0));
    }
}

static uint16_t odp_espit_bus_status(OdpESPITargetState *s)
{
    return ESPI_STATUS_FIXED_FREE |
           (odp_espit_ready(s) ? ODP_ESPI_STATUS_PC_FREE |
                                 ODP_ESPI_STATUS_NP_FREE : 0) |
           (odp_espit_vwire_ready(s) && s->vwire_count > s->tx_vwire_count ?
            ODP_ESPI_STATUS_VWIRE_AVAIL : 0);
}

static void odp_espit_reset_hold(Object *obj, ResetType type)
{
    OdpESPITargetState *s = ODP_ESPI_TARGET(obj);
    bool aborted = odp_espit_pending(s);
    unsigned index;

    odp_espit_clear_io(s);
    if (s->connected) {
        qemu_chr_fe_disconnect(&s->chr);
    }
    s->control = 0;
    s->errors = aborted ? ESPI_ERROR_ABORT : 0;
    s->general = ESPI_GENERAL_SUPPORTED;
    s->channel = ESPI_CH0_RESET;
    s->vwire_channel = ESPI_CH1_RESET;
    s->host_irq = 0;
    s->mailbox_control_base = 0;
    for (index = 0; index < ESPI_MAILBOX_COUNT; index++) {
        OdpESPIMailbox *mailbox = &s->mailboxes[index];

        mailbox->base = 0;
        mailbox->shared_status = 0;
        mailbox->doorbell = false;
        memset(mailbox->data, 0, sizeof(mailbox->data));
    }
    odp_espit_update_irq(s);
}

static gboolean odp_espit_writable(void *unused, GIOCondition condition,
                                    void *opaque);

static void odp_espit_flush(OdpESPITargetState *s)
{
    int written;

    written = qemu_chr_fe_write(&s->chr, s->tx + s->tx_pos,
                                s->tx_size - s->tx_pos);
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        odp_espit_disconnect(s, ESPI_ERROR_LINK);
        return;
    }
    if (written > 0) {
        s->tx_pos += written;
    }
    if (s->tx_pos < s->tx_size) {
        s->write_watch = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                               odp_espit_writable, s);
        if (!s->write_watch) {
            odp_espit_disconnect(s, ESPI_ERROR_LINK);
        }
        return;
    }
    s->tx_size = 0;
    s->tx_pos = 0;
    timer_del(s->timer);
    s->vwire_head = (s->vwire_head + s->tx_vwire_count) %
                    ESPI_VWIRE_QUEUE_SIZE;
    s->vwire_count -= s->tx_vwire_count;
    s->tx_vwire_count = 0;
    if (s->config_address == ODP_ESPI_CONFIG_GENERAL) {
        s->general = s->config_value;
    } else if (s->config_address == ODP_ESPI_CONFIG_CH0) {
        s->channel = s->config_value;
    } else if (s->config_address == ODP_ESPI_CONFIG_CH1) {
        bool enabled = !(s->vwire_channel & ODP_ESPI_CH1_ENABLE) &&
                       (s->config_value & ODP_ESPI_CH1_ENABLE);

        s->vwire_channel = s->config_value;
        if (enabled) {
            odp_espit_sync_vwires(s);
        }
    }
    s->config_address = 0;
    odp_espit_update_irq(s);
    odp_espit_alert(s);
    qemu_chr_fe_accept_input(&s->chr);
}

static gboolean odp_espit_writable(void *unused, GIOCondition condition,
                                    void *opaque)
{
    OdpESPITargetState *s = opaque;

    s->write_watch = 0;
    if (condition & G_IO_HUP) {
        odp_espit_disconnect(s, ESPI_ERROR_LINK);
    } else {
        odp_espit_flush(s);
    }
    return G_SOURCE_REMOVE;
}

static void odp_espit_alert(OdpESPITargetState *s)
{
    uint16_t status = odp_espit_bus_status(s);

    if (!s->connected || s->external_reset || s->tx_size ||
        s->rx_state != ESPI_RX_KIND ||
        status == s->reported_status || status == s->alerted_status) {
        return;
    }
    s->alerted_status = status;
    s->tx[0] = ODP_ESPI_WIRE_ALERT;
    s->tx_size = 1;
    s->tx_pos = 0;
    odp_espit_arm_timer(s);
    odp_espit_flush(s);
}

static void odp_espit_reply(OdpESPITargetState *s, uint8_t response,
                             const uint8_t *payload, unsigned length)
{
    uint16_t status = odp_espit_bus_status(s);

    s->tx[0] = ODP_ESPI_WIRE_DATA;
    s->tx[1] = response;
    if (length) {
        memcpy(s->tx + 2, payload, length);
    }
    stw_le_p(s->tx + 2 + length, status);
    s->tx[4 + length] = odp_espit_crc8(s->tx + 1, length + 3);
    s->tx_size = length + 5;
    s->tx_pos = 0;
    s->reported_status = status;
    s->alerted_status = status;
    odp_espit_arm_timer(s);
    odp_espit_flush(s);
}

static void odp_espit_reject(OdpESPITargetState *s, uint32_t error)
{
    odp_espit_set_error(s, error);
    odp_espit_reply(s, ODP_ESPI_RSP_FATAL_ERROR, NULL, 0);
}

static void odp_espit_configuration(OdpESPITargetState *s)
{
    uint16_t address = lduw_be_p(s->rx + 1) & 0x0ffc;
    uint32_t value = 0;
    uint8_t data[4];

    if (s->rx[0] == ODP_ESPI_CMD_GET_CONFIGURATION) {
        switch (address) {
        case ODP_ESPI_CONFIG_DEVICE_ID:
            value = ODP_ESPI_DEVICE_ID_VERSION;
            break;
        case ODP_ESPI_CONFIG_GENERAL:
            value = s->general;
            break;
        case ODP_ESPI_CONFIG_CH0:
            value = odp_espit_channel(s);
            break;
        case ODP_ESPI_CONFIG_CH1:
            value = odp_espit_vwire_channel(s);
            break;
        }
        stl_le_p(data, value);
        odp_espit_reply(s, ODP_ESPI_RSP_ACCEPT, data, sizeof(data));
        return;
    }
    value = ldl_le_p(s->rx + 3);
    switch (address) {
    case ODP_ESPI_CONFIG_GENERAL:
        if (value & ESPI_GENERAL_UNSUPPORTED) {
            odp_espit_reject(s, ESPI_ERROR_CONFIG);
            return;
        }
        s->config_address = address;
        s->config_value = ESPI_GENERAL_SUPPORTED |
                          (value & ESPI_GENERAL_WRITABLE);
        break;
    case ODP_ESPI_CONFIG_CH0:
        if ((value & ODP_ESPI_CH0_BUS_MASTER_ENABLE) ||
            ((value & ODP_ESPI_CH0_MAX_PAYLOAD_SELECTED_MASK) >>
             ODP_ESPI_CH0_MAX_PAYLOAD_SELECTED_SHIFT) !=
             ODP_ESPI_SIZE_64B_CODE ||
            ((value & ODP_ESPI_CH0_MAX_READ_REQUEST_MASK) >>
             ODP_ESPI_CH0_MAX_READ_REQUEST_SHIFT) != ODP_ESPI_SIZE_64B_CODE) {
            odp_espit_reject(s, ESPI_ERROR_CONFIG);
            return;
        }
        s->config_address = address;
        s->config_value = (ESPI_CH0_RESET & ~ODP_ESPI_CH0_ENABLE) |
                          (value & ODP_ESPI_CH0_ENABLE);
        break;
    case ODP_ESPI_CONFIG_CH1:
        if (((value & ODP_ESPI_CH1_MAX_COUNT_SELECTED_MASK) >>
             ODP_ESPI_CH1_MAX_COUNT_SELECTED_SHIFT) >= ESPI_VWIRE_COUNT) {
            odp_espit_reject(s, ESPI_ERROR_CONFIG);
            return;
        }
        s->config_address = address;
        s->config_value = ESPI_CH1_RESET |
                          (value & (ODP_ESPI_CH1_ENABLE |
                                    ODP_ESPI_CH1_MAX_COUNT_SELECTED_MASK));
        break;
    }
    odp_espit_reply(s, ODP_ESPI_RSP_ACCEPT, NULL, 0);
}

static uint32_t odp_espit_control_read(OdpESPITargetState *state,
                                      hwaddr offset)
{
    unsigned index = offset / ESPI_MAILBOX_CONTROL_STRIDE;
    OdpESPIMailbox *mailbox;

    if (index >= ESPI_MAILBOX_COUNT) {
        return 0;
    }
    mailbox = &state->mailboxes[index];
    switch (offset % ESPI_MAILBOX_CONTROL_STRIDE) {
    case ESPI_MAILBOX_SHARED_STATUS:
        return mailbox->shared_status;
    case ESPI_MAILBOX_DOORBELL:
        return mailbox->doorbell;
    case ESPI_MAILBOX_IRQ_SET:
    case ESPI_MAILBOX_IRQ_ACK:
        return (state->host_irq >> index) & 1;
    }
    return 0;
}

static void odp_espit_control_write(OdpESPITargetState *state, hwaddr offset,
                                    uint32_t value, bool host)
{
    unsigned index = offset / ESPI_MAILBOX_CONTROL_STRIDE;
    uint32_t mask;
    OdpESPIMailbox *mailbox;

    if (index >= ESPI_MAILBOX_COUNT) {
        return;
    }
    mask = 1U << index;
    mailbox = &state->mailboxes[index];
    switch (offset % ESPI_MAILBOX_CONTROL_STRIDE) {
    case ESPI_MAILBOX_SHARED_STATUS:
        mailbox->shared_status = value;
        break;
    case ESPI_MAILBOX_DOORBELL:
        if (value & 1) {
            mailbox->doorbell = host;
        }
        break;
    case ESPI_MAILBOX_IRQ_SET:
        if (!host && (value & 1)) {
            odp_espit_host_irq(state, state->host_irq | mask);
        }
        break;
    case ESPI_MAILBOX_IRQ_ACK:
        if (value & 1) {
            odp_espit_host_irq(state, state->host_irq & ~mask);
        }
        break;
    }
    odp_espit_update_irq(state);
}

static void odp_espit_memory_request(OdpESPITargetState *s)
{
    bool write = s->rx[1] == ODP_ESPI_CYCLE_MEM_WRITE32;
    uint16_t tag_length = lduw_be_p(s->rx + 2);
    unsigned length = tag_length & ODP_ESPI_PACKET_LENGTH_MASK;
    uint32_t address = ldl_be_p(s->rx + 4);
    OdpESPIMailbox *mailbox = NULL;
    uint8_t completion[3 + ESPI_MAX_PAYLOAD];
    unsigned index;
    uint32_t offset, reg;

    if (write != (s->rx[0] == ODP_ESPI_CMD_PUT_PC)) {
        odp_espit_reject(s, ESPI_ERROR_PERMISSION | ESPI_ERROR_PROTOCOL);
        return;
    }
    if (!odp_espit_ready(s)) {
        odp_espit_reject(s, ESPI_ERROR_BUSY);
        return;
    }
    if (!length || length > ESPI_MAX_PAYLOAD ||
        (address & (ESPI_MAX_PAYLOAD - 1)) + length > ESPI_MAX_PAYLOAD) {
        odp_espit_reject(s, ESPI_ERROR_LENGTH);
        return;
    }
    if (address >= s->mailbox_control_base &&
        (uint64_t)address < (uint64_t)s->mailbox_control_base +
                            ESPI_MAILBOX_CONTROL_SIZE) {
        offset = address - s->mailbox_control_base;
        reg = offset % ESPI_MAILBOX_CONTROL_STRIDE;
        if (length != 4 || (address & 3)) {
            odp_espit_set_error(s, ESPI_ERROR_LENGTH);
            goto unsuccessful;
        }
        if (offset >= ESPI_MAILBOX_COUNT * ESPI_MAILBOX_CONTROL_STRIDE ||
            (reg != ESPI_MAILBOX_SHARED_STATUS &&
             reg != ESPI_MAILBOX_DOORBELL && reg != ESPI_MAILBOX_IRQ_ACK)) {
            odp_espit_set_error(s, ESPI_ERROR_ADDRESS);
            goto unsuccessful;
        }
        if (write) {
            odp_espit_control_write(s, offset, ldl_le_p(s->rx + 8), true);
        } else {
            completion[0] = ODP_ESPI_CYCLE_SUCCESS_DATA_ONLY;
            stw_be_p(completion + 1, tag_length);
            stl_le_p(completion + 3, odp_espit_control_read(s, offset));
        }
        odp_espit_reply(s, ODP_ESPI_RSP_ACCEPT, completion,
                        write ? 0 : length + 3);
        return;
    }
    for (index = 0; index < ESPI_MAILBOX_COUNT; index++) {
        OdpESPIMailbox *candidate = &s->mailboxes[index];

        if (address >= candidate->base &&
            (uint64_t)address < (uint64_t)candidate->base + ESPI_MAILBOX_SIZE) {
            mailbox = candidate;
            break;
        }
    }
    if (!mailbox || (uint64_t)address + length >
                    (uint64_t)mailbox->base + ESPI_MAILBOX_SIZE) {
        odp_espit_set_error(s, ESPI_ERROR_ADDRESS);
        goto unsuccessful;
    }
    offset = address - mailbox->base;
    if (write) {
        memcpy(mailbox->data + offset, s->rx + 8, length);
    } else {
        completion[0] = ODP_ESPI_CYCLE_SUCCESS_DATA_ONLY;
        stw_be_p(completion + 1, tag_length);
        memcpy(completion + 3, mailbox->data + offset, length);
    }
    odp_espit_reply(s, ODP_ESPI_RSP_ACCEPT, completion,
                    write ? 0 : length + 3);
    return;

unsuccessful:
    if (write) {
        odp_espit_reply(s, ODP_ESPI_RSP_FATAL_ERROR, NULL, 0);
    } else {
        completion[0] = ODP_ESPI_CYCLE_UNSUCCESSFUL_ONLY;
        stw_be_p(completion + 1, tag_length & 0xf000);
        odp_espit_reply(s, ODP_ESPI_RSP_ACCEPT, completion, 3);
    }
}

static void odp_espit_vwire_request(OdpESPITargetState *state)
{
    unsigned limit = 1 + ((state->vwire_channel &
                          ODP_ESPI_CH1_MAX_COUNT_SELECTED_MASK) >>
                         ODP_ESPI_CH1_MAX_COUNT_SELECTED_SHIFT);
    unsigned count, index;
    uint8_t payload[1 + ESPI_VWIRE_COUNT * ODP_ESPI_VWIRE_GROUP_SIZE];

    if (!odp_espit_vwire_ready(state)) {
        odp_espit_reject(state, ESPI_ERROR_BUSY);
        return;
    }
    if (state->rx[0] == ODP_ESPI_CMD_PUT_VWIRE) {
        count = 1 + (state->rx[1] & ODP_ESPI_VWIRE_COUNT_MASK);
        if ((state->rx[1] & ~ODP_ESPI_VWIRE_COUNT_MASK) || count > limit) {
            odp_espit_reject(state, ESPI_ERROR_LENGTH);
            return;
        }
        odp_espit_reply(state, ODP_ESPI_RSP_ACCEPT, NULL, 0);
        return;
    }
    if (!state->vwire_count) {
        odp_espit_reply(state, ODP_ESPI_RSP_NON_FATAL_ERROR, NULL, 0);
        return;
    }
    count = MIN(limit, state->vwire_count);
    for (index = 0; index < count; index++) {
        unsigned slot = (state->vwire_head + index) % ESPI_VWIRE_QUEUE_SIZE;
        unsigned previous, transitions = 0;

        for (previous = 0; previous < index; previous++) {
            if (payload[1 + previous * 2] == state->vwire_queue[slot][0] &&
                (state->vwire_queue[slot][0] != ODP_ESPI_VWIRE_IRQ_LOW ||
                 (payload[2 + previous * 2] & ODP_ESPI_VWIRE_IRQ_MASK) ==
                 (state->vwire_queue[slot][1] & ODP_ESPI_VWIRE_IRQ_MASK))) {
                transitions++;
            }
        }
        if (transitions == 2) {
            break;
        }
        memcpy(payload + 1 + index * 2, state->vwire_queue[slot], 2);
    }
    state->tx_vwire_count = index;
    payload[0] = index - 1;
    odp_espit_reply(state, ODP_ESPI_RSP_ACCEPT, payload, 1 + index * 2);
}

static void odp_espit_command(OdpESPITargetState *s)
{
    if ((s->general & ODP_ESPI_GENERAL_CRC_ENABLE) &&
        odp_espit_crc8(s->rx, s->rx_size)) {
        odp_espit_disconnect(s, ESPI_ERROR_CRC);
        return;
    }
    switch (s->rx[0]) {
    case ODP_ESPI_CMD_GET_CONFIGURATION:
    case ODP_ESPI_CMD_SET_CONFIGURATION:
        odp_espit_configuration(s);
        break;
    case ODP_ESPI_CMD_GET_STATUS:
        odp_espit_reply(s, ODP_ESPI_RSP_ACCEPT, NULL, 0);
        break;
    case ODP_ESPI_CMD_PUT_PC:
    case ODP_ESPI_CMD_PUT_NP:
        odp_espit_memory_request(s);
        break;
    case ODP_ESPI_CMD_GET_PC:
        odp_espit_reject(s, ESPI_ERROR_PROTOCOL);
        break;
    case ODP_ESPI_CMD_PUT_VWIRE:
    case ODP_ESPI_CMD_GET_VWIRE:
        odp_espit_vwire_request(s);
        break;
    }
}

static void odp_espit_external_reset(OdpESPITargetState *s, bool asserted)
{
    bool abort_tx = s->tx_size != 0;
    unsigned index;

    if (asserted) {
        odp_espit_set_error(s, abort_tx ? ESPI_ERROR_ABORT : 0);
        odp_espit_clear_io(s);
        s->general = ESPI_GENERAL_SUPPORTED;
        s->channel = ESPI_CH0_RESET;
        s->vwire_channel = ESPI_CH1_RESET;
        for (index = 0; index < ESPI_MAILBOX_COUNT; index++) {
            s->mailboxes[index].doorbell = false;
        }
    }
    s->external_reset = asserted;
    odp_espit_update_irq(s);
    if (abort_tx) {
        odp_espit_disconnect(s, ESPI_ERROR_ABORT);
    } else {
        odp_espit_alert(s);
    }
}

static int odp_espit_can_receive(void *opaque)
{
    OdpESPITargetState *s = opaque;

    return s->tx_size ? 0 : sizeof(s->rx) - s->rx_size;
}

static void odp_espit_receive(void *opaque, const uint8_t *data, int size)
{
    OdpESPITargetState *s = opaque;
    int index;

    for (index = 0; index < size; index++) {
        uint8_t byte = data[index];

        if (!s->connected) {
            return;
        }
        if (s->rx_state == ESPI_RX_KIND) {
            if (byte == ODP_ESPI_WIRE_RESET) {
                s->rx_state = ESPI_RX_RESET;
            } else if (byte == ODP_ESPI_WIRE_DATA && !s->external_reset &&
                       !s->tx_size) {
                s->rx_state = ESPI_RX_DATA;
                s->rx_size = 0;
                s->rx_needed = 0;
            } else {
                odp_espit_disconnect(s, ESPI_ERROR_PROTOCOL);
                return;
            }
            odp_espit_arm_timer(s);
            continue;
        }
        if (s->rx_state == ESPI_RX_RESET) {
            if (byte > ODP_ESPI_WIRE_RESET_ASSERTED) {
                odp_espit_disconnect(s, ESPI_ERROR_PROTOCOL);
                return;
            }
            s->rx_state = ESPI_RX_KIND;
            timer_del(s->timer);
            odp_espit_external_reset(s, byte == ODP_ESPI_WIRE_RESET_ASSERTED);
            continue;
        }
        if (s->rx_size == sizeof(s->rx)) {
            odp_espit_disconnect(s, ESPI_ERROR_LENGTH);
            return;
        }
        s->rx[s->rx_size++] = byte;
        if (s->rx_size == 1) {
            switch (byte) {
            case ODP_ESPI_CMD_GET_CONFIGURATION:
                s->rx_needed = 4;
                break;
            case ODP_ESPI_CMD_SET_CONFIGURATION:
                s->rx_needed = 8;
                break;
            case ODP_ESPI_CMD_GET_STATUS:
            case ODP_ESPI_CMD_GET_PC:
            case ODP_ESPI_CMD_GET_VWIRE:
                s->rx_needed = 2;
                break;
            case ODP_ESPI_CMD_PUT_PC:
            case ODP_ESPI_CMD_PUT_NP:
            case ODP_ESPI_CMD_PUT_VWIRE:
                break;
            default:
                odp_espit_disconnect(s, ESPI_ERROR_UNSUPPORTED);
                return;
            }
        }
        if (s->rx[0] == ODP_ESPI_CMD_PUT_VWIRE && s->rx_size == 2) {
            s->rx_needed = 3 + 2 *
                (1 + (s->rx[1] & ODP_ESPI_VWIRE_COUNT_MASK));
        }
        if (!s->rx_needed && s->rx_size == 4) {
            unsigned length = lduw_be_p(s->rx + 2) &
                              ODP_ESPI_PACKET_LENGTH_MASK;

            if (s->rx[1] == ODP_ESPI_CYCLE_MEM_WRITE32) {
                s->rx_needed = 9 + (length ? length :
                                    ODP_ESPI_PACKET_LENGTH_MAX);
            } else if (s->rx[1] == ODP_ESPI_CYCLE_MEM_READ32) {
                s->rx_needed = 9;
            } else {
                odp_espit_disconnect(s, ESPI_ERROR_UNSUPPORTED);
                return;
            }
        }
        if (s->rx_needed && s->rx_size == s->rx_needed) {
            s->rx_state = ESPI_RX_KIND;
            timer_del(s->timer);
            odp_espit_command(s);
            s->rx_size = 0;
            s->rx_needed = 0;
        }
    }
}

static void odp_espit_event(void *opaque, QEMUChrEvent event)
{
    OdpESPITargetState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        odp_espit_clear_io(s);
        s->connected = true;
        s->external_reset = false;
        s->general = ESPI_GENERAL_SUPPORTED;
        s->channel = ESPI_CH0_RESET;
        s->vwire_channel = ESPI_CH1_RESET;
        odp_espit_alert(s);
        break;
    case CHR_EVENT_CLOSED:
        odp_espit_set_error(s, ESPI_ERROR_LINK |
                           (odp_espit_pending(s) ? ESPI_ERROR_ABORT : 0));
        s->connected = false;
        s->external_reset = false;
        s->general = ESPI_GENERAL_SUPPORTED;
        s->channel = ESPI_CH0_RESET;
        s->vwire_channel = ESPI_CH1_RESET;
        odp_espit_clear_io(s);
        break;
    default:
        break;
    }
    odp_espit_update_irq(s);
}

static bool odp_espit_decode_valid(OdpESPITargetState *s)
{
    uint32_t bases[] = {
        s->mailboxes[0].base,
        s->mailboxes[1].base,
        s->mailbox_control_base,
    };
    unsigned index, previous;

    for (index = 0; index < ARRAY_SIZE(bases); index++) {
        if (bases[index] & (ESPI_MAILBOX_SIZE - 1)) {
            return false;
        }
        for (previous = 0; previous < index; previous++) {
            if (bases[index] == bases[previous]) {
                return false;
            }
        }
    }
    return true;
}

static uint64_t odp_espit_reg_read(void *opaque, hwaddr offset, unsigned size)
{
    OdpESPITargetState *s = opaque;
    unsigned index;

    switch (offset) {
    case ESPI_REG_CONTROL:
        return s->control;
    case ESPI_REG_MAILBOX0_ADDR:
    case ESPI_REG_MAILBOX1_ADDR:
        index = (offset - ESPI_REG_MAILBOX0_ADDR) / 4;
        return s->mailboxes[index].base;
    case ESPI_REG_CONTROL_ADDR:
        return s->mailbox_control_base;
    }
    if (offset >= ESPI_REG_MAILBOX_CONTROL &&
        offset < ESPI_REG_MAILBOX_CONTROL +
                 ESPI_MAILBOX_COUNT * ESPI_REG_MAILBOX_STRIDE) {
        offset -= ESPI_REG_MAILBOX_CONTROL;
        index = offset / ESPI_REG_MAILBOX_STRIDE;
        return odp_espit_control_read(s,
                   index * ESPI_MAILBOX_CONTROL_STRIDE +
                   offset % ESPI_REG_MAILBOX_STRIDE);
    }
    return 0;
}

static void odp_espit_reg_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    OdpESPITargetState *s = opaque;
    unsigned index;

    switch (offset) {
    case ESPI_REG_CONTROL:
        value &= ESPI_CONTROL_ENABLE;
        if (value && !odp_espit_decode_valid(s)) {
            odp_espit_set_error(s, ESPI_ERROR_CONFIG);
            return;
        }
        s->control = value;
        odp_espit_alert(s);
        return;
    case ESPI_REG_MAILBOX0_ADDR:
    case ESPI_REG_MAILBOX1_ADDR:
    case ESPI_REG_CONTROL_ADDR:
        if (s->control & ESPI_CONTROL_ENABLE) {
            odp_espit_set_error(s, ESPI_ERROR_CONFIG);
        } else if (offset == ESPI_REG_CONTROL_ADDR) {
            s->mailbox_control_base = value;
        } else {
            index = (offset - ESPI_REG_MAILBOX0_ADDR) / 4;
            s->mailboxes[index].base = value;
        }
        return;
    }
    if (offset >= ESPI_REG_MAILBOX_CONTROL &&
        offset < ESPI_REG_MAILBOX_CONTROL +
                 ESPI_MAILBOX_COUNT * ESPI_REG_MAILBOX_STRIDE) {
        offset -= ESPI_REG_MAILBOX_CONTROL;
        index = offset / ESPI_REG_MAILBOX_STRIDE;
        odp_espit_control_write(s, index * ESPI_MAILBOX_CONTROL_STRIDE +
                                offset % ESPI_REG_MAILBOX_STRIDE,
                               value, false);
        odp_espit_alert(s);
    }
}

static bool odp_espit_reg_accepts(void *opaque, hwaddr offset, unsigned size,
                                  bool write, MemTxAttrs attrs)
{
    if (size != 4 || (offset & 3)) {
        return false;
    }
    return true;
}

static const MemoryRegionOps odp_espit_reg_ops = {
    .read = odp_espit_reg_read,
    .write = odp_espit_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4,
               .accepts = odp_espit_reg_accepts },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t odp_espit_memory_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    OdpESPIMailbox *mailbox = opaque;
    uint64_t value = 0;
    unsigned index;

    for (index = 0; index < size; index++) {
        value |= (uint64_t)mailbox->data[offset + index] << (index * 8);
    }
    return value;
}

static void odp_espit_memory_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
    OdpESPIMailbox *mailbox = opaque;
    unsigned index;

    for (index = 0; index < size; index++) {
        mailbox->data[offset + index] = value >> (index * 8);
    }
}

static bool odp_espit_memory_accepts(void *opaque, hwaddr offset,
                                     unsigned size, bool write,
                                     MemTxAttrs attrs)
{
    if (size < 1 || size > 4 || (offset & (size - 1))) {
        return false;
    }
    return true;
}

static const MemoryRegionOps odp_espit_memory_ops = {
    .read = odp_espit_memory_read,
    .write = odp_espit_memory_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4,
               .accepts = odp_espit_memory_accepts },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void odp_espi_target_init(Object *obj)
{
    OdpESPITargetState *s = ODP_ESPI_TARGET(obj);
    SysBusDevice *busdev = SYS_BUS_DEVICE(obj);
    unsigned index;

    memory_region_init_io(&s->iomem, obj, &odp_espit_reg_ops, s,
                          TYPE_ODP_ESPI_TARGET, ESPI_REG_SIZE);
    sysbus_init_mmio(busdev, &s->iomem);
    sysbus_init_irq(busdev, &s->irq);
    for (index = 0; index < ESPI_MAILBOX_COUNT; index++) {
        OdpESPIMailbox *mailbox = &s->mailboxes[index];

        memory_region_init_io(&mailbox->iomem, obj, &odp_espit_memory_ops,
                              mailbox, "odp-espi-mailbox", ESPI_MAILBOX_SIZE);
        sysbus_init_mmio(busdev, &mailbox->iomem);
    }
}

static void odp_espi_target_realize(DeviceState *dev, Error **errp)
{
    OdpESPITargetState *s = ODP_ESPI_TARGET(dev);

    if (!s->timeout_ms || s->timeout_ms > INT_MAX) {
        error_setg(errp, "eSPI timeout-ms must be between 1 and %d", INT_MAX);
        return;
    }
    s->timer = timer_new_ms(QEMU_CLOCK_REALTIME, odp_espit_timeout, s);
    odp_espit_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
    qemu_chr_fe_set_handlers(&s->chr, odp_espit_can_receive, odp_espit_receive,
                             odp_espit_event, NULL, s, NULL, true);
}

static void odp_espi_target_unrealize(DeviceState *dev)
{
    OdpESPITargetState *s = ODP_ESPI_TARGET(dev);

    qemu_chr_fe_set_handlers(&s->chr, NULL, NULL, NULL, NULL, NULL,
                             NULL, false);
    odp_espit_clear_io(s);
    qemu_set_irq(s->irq, 0);
    timer_free(s->timer);
}

static const Property odp_espi_target_properties[] = {
    DEFINE_PROP_CHR("chardev", OdpESPITargetState, chr),
    DEFINE_PROP_UINT32("timeout-ms", OdpESPITargetState, timeout_ms, 1000),
};

static const VMStateDescription odp_espit_vmstate = {
    .name = TYPE_ODP_ESPI_TARGET,
    .unmigratable = true,
};

static void odp_espi_target_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "ODP socket-backed eSPI target";
    dc->realize = odp_espi_target_realize;
    dc->unrealize = odp_espi_target_unrealize;
    dc->vmsd = &odp_espit_vmstate;
    dc->hotpluggable = false;
    rc->phases.hold = odp_espit_reset_hold;
    device_class_set_props(dc, odp_espi_target_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo odp_espi_target_info = {
    .name          = TYPE_ODP_ESPI_TARGET,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OdpESPITargetState),
    .instance_init = odp_espi_target_init,
    .class_init    = odp_espi_target_class_init,
};

static void odp_espi_target_register_types(void)
{
    type_register_static(&odp_espi_target_info);
}

type_init(odp_espi_target_register_types)
