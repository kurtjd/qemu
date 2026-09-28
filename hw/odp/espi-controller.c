#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"
#include "exec/address-spaces.h"
#include "hw/core/cpu.h"
#include "hw/irq.h"
#include "hw/odp/espi-controller.h"
#include "hw/odp/espi-proto.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "system/cpus.h"

#define ESPI_DATA_WINDOW_COUNT 2
#define ESPI_WINDOW_COUNT      (ESPI_DATA_WINDOW_COUNT + 1)
#define ESPI_WINDOW_SIZE       0x1000
#define ESPI_CONTROL_STRIDE    0x20
#define ESPI_CONTROL_STATUS    0x00
#define ESPI_CONTROL_DOORBELL  0x04
#define ESPI_CONTROL_IRQ_ACK   0x0c
#define ESPI_MAX_PAYLOAD       64
#define ESPI_RESPONSE_MODIFIER (1U << 30)

#define ESPI_ERROR_TIMEOUT    (1U << 0)
#define ESPI_ERROR_CRC        (1U << 1)
#define ESPI_ERROR_PROTOCOL   (1U << 2)
#define ESPI_ERROR_LINK       (1U << 3)
#define ESPI_ERROR_COMPLETION (1U << 4)
#define ESPI_ERROR_WINDOW     (1U << 5)

typedef struct OdpESPIWindow {
    MemoryRegion iomem;
    OdpESPIControllerState *controller;
    uint32_t base;
    uint32_t size;
    bool enabled;
    bool mapped;
} OdpESPIWindow;

struct OdpESPIControllerState {
    SysBusDevice parent_obj;
    CharBackend chr;
    qemu_irq irq[ODP_ESPI_IRQ_COUNT];
    OdpESPIWindow windows[ESPI_WINDOW_COUNT];
    uint32_t errors;
    uint32_t irq_level;
    unsigned vwire_limit;
    bool vwire_supported;
    bool vwire_ready;
    bool connected;
    bool ready;
    bool configured;
    bool reset_needed;
    bool crc_enabled;
    bool busy;
    bool failed;
    bool pending;
    bool expect_response;
    bool receiving;
    bool alert;
    uint8_t command;
    uint8_t tag;
    uint16_t target_status;
    uint8_t tx[ESPI_MAX_PAYLOAD + 16];
    unsigned tx_size;
    unsigned tx_pos;
    uint8_t rx[ODP_ESPI_VWIRE_MAX_PACKET_SIZE + 4];
    unsigned rx_size;
    unsigned rx_needed;
    uint32_t timeout_ms;
    guint write_watch;
    guint service_source;
    QemuCond cond;
};

static void odp_espi_schedule_service(OdpESPIControllerState *s);
static bool odp_espi_drain_vwires(OdpESPIControllerState *state,
                                  int64_t deadline);

static void odp_espi_update_irq(OdpESPIControllerState *state)
{
    unsigned index;

    for (index = 0; index < ODP_ESPI_IRQ_COUNT; index++) {
        bool level = (state->irq_level & (1U << index)) != 0;

        qemu_set_irq(state->irq[index], level);
    }
}

static void odp_espi_clear_vwires(OdpESPIControllerState *state)
{
    state->irq_level = 0;
    state->vwire_ready = false;
    state->target_status = 0;
    state->alert = false;
    odp_espi_update_irq(state);
}

static int64_t odp_espi_now_ms(void)
{
    return g_get_monotonic_time() / 1000;
}

static uint8_t odp_espi_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = ODP_ESPI_CRC8_INIT;
    size_t index;
    unsigned bit;

    for (index = 0; index < length; index++) {
        crc ^= data[index];
        for (bit = 0; bit < 8; bit++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? ODP_ESPI_CRC8_POLYNOMIAL : 0);
        }
    }
    return crc;
}

static void odp_espi_set_error(OdpESPIControllerState *state, uint32_t error)
{
    state->errors |= error;
}

static void odp_espi_cancel(OdpESPIControllerState *s, uint32_t error)
{
    odp_espi_set_error(s, error);
    s->failed = true;
    s->pending = false;
    s->ready = false;
    s->configured = false;
    s->receiving = false;
    odp_espi_clear_vwires(s);
    if (s->write_watch) {
        g_source_remove(s->write_watch);
        s->write_watch = 0;
    }
    qemu_cond_broadcast(&s->cond);
}

static void odp_espi_disconnect(OdpESPIControllerState *s, uint32_t error)
{
    odp_espi_cancel(s, error);
    qemu_chr_fe_disconnect(&s->chr);
}

static gboolean odp_espi_wakeup(void *opaque)
{
    return G_SOURCE_REMOVE;
}

static bool odp_espi_wait(OdpESPIControllerState *s, int64_t deadline)
{
    int64_t remaining = deadline - odp_espi_now_ms();

    if (remaining <= 0) {
        odp_espi_disconnect(s, ESPI_ERROR_TIMEOUT);
        return false;
    }
    if (qemu_in_vcpu_thread()) {
        CPUState *cpu = current_cpu;
        bool running = cpu->running;

        if (running) {
            cpu_exec_end(cpu);
        }
        process_queued_cpu_work(cpu);
        qemu_cond_timedwait_bql(&s->cond, MIN(remaining, 10));
        if (running) {
            bql_unlock();
            cpu_exec_start(cpu);
            bql_lock();
        }
    } else {
        GSource *source = g_timeout_source_new(MIN(remaining, 10));

        g_source_set_callback(source, odp_espi_wakeup, NULL, NULL);
        g_source_attach(source, NULL);
        g_main_context_iteration(NULL, true);
        g_source_destroy(source);
        g_source_unref(source);
    }
    return !s->failed;
}

static gboolean odp_espi_writable(void *unused, GIOCondition condition,
                                   void *opaque);

static void odp_espi_flush(OdpESPIControllerState *s)
{
    int written;

    written = qemu_chr_fe_write(&s->chr, s->tx + s->tx_pos,
                                s->tx_size - s->tx_pos);
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        odp_espi_disconnect(s, ESPI_ERROR_LINK);
        return;
    }
    if (written > 0) {
        s->tx_pos += written;
    }
    if (s->tx_pos < s->tx_size) {
        s->write_watch = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                               odp_espi_writable, s);
        if (!s->write_watch) {
            odp_espi_disconnect(s, ESPI_ERROR_LINK);
        }
    } else if (!s->expect_response) {
        s->pending = false;
        qemu_cond_broadcast(&s->cond);
    }
}

static gboolean odp_espi_writable(void *unused, GIOCondition condition,
                                   void *opaque)
{
    OdpESPIControllerState *s = opaque;

    s->write_watch = 0;
    if (condition & G_IO_HUP) {
        odp_espi_disconnect(s, ESPI_ERROR_LINK);
    } else {
        odp_espi_flush(s);
    }
    return G_SOURCE_REMOVE;
}

static bool odp_espi_transmit(OdpESPIControllerState *s, unsigned size,
                              bool response, int64_t deadline)
{
    if (!s->connected || s->failed) {
        return false;
    }
    s->tx_size = size;
    s->tx_pos = 0;
    s->rx_size = 0;
    s->rx_needed = 0;
    s->receiving = false;
    s->expect_response = response;
    s->alert = false;
    s->pending = true;
    odp_espi_flush(s);
    while (s->pending) {
        if (!odp_espi_wait(s, deadline)) {
            return false;
        }
    }
    return !s->failed;
}

static bool odp_espi_exchange(OdpESPIControllerState *s, uint8_t command,
                              const uint8_t *data, unsigned length,
                              int64_t deadline)
{
    s->command = command;
    s->tx[0] = ODP_ESPI_WIRE_DATA;
    s->tx[1] = command;
    if (length) {
        memcpy(s->tx + 2, data, length);
    }
    s->tx[length + 2] = odp_espi_crc8(s->tx + 1, length + 1);
    return odp_espi_transmit(s, length + 3, true, deadline);
}

static bool odp_espi_configuration(OdpESPIControllerState *s, uint16_t address,
                                   uint32_t *value, bool write,
                                   int64_t deadline)
{
    uint8_t data[6];

    stw_be_p(data, address);
    if (write) {
        stl_le_p(data + 2, *value);
    }
    if (!odp_espi_exchange(s, write ? ODP_ESPI_CMD_SET_CONFIGURATION :
                                     ODP_ESPI_CMD_GET_CONFIGURATION,
                           data, write ? 6 : 2, deadline)) {
        return false;
    }
    if (!write) {
        *value = ldl_le_p(s->rx + 1);
    }
    return true;
}

static bool odp_espi_prepare(OdpESPIControllerState *s, int64_t deadline)
{
    uint32_t value;
    unsigned supported;
    bool ready = s->ready;

    if (s->reset_needed) {
        odp_espi_clear_vwires(s);
        s->crc_enabled = false;
        s->tx[0] = ODP_ESPI_WIRE_RESET;
        s->tx[1] = ODP_ESPI_WIRE_RESET_ASSERTED;
        s->tx[2] = ODP_ESPI_WIRE_RESET;
        s->tx[3] = ODP_ESPI_WIRE_RESET_DEASSERTED;
        if (!odp_espi_transmit(s, 4, false, deadline)) {
            return false;
        }
        s->reset_needed = false;
    }
    if (!s->configured) {
        if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_DEVICE_ID,
                                    &value, false, deadline)) {
            return false;
        }
        if ((value & ODP_ESPI_DEVICE_ID_VERSION_MASK) !=
            ODP_ESPI_DEVICE_ID_VERSION) {
            odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
            return false;
        }
        if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_GENERAL,
                                    &value, false, deadline)) {
            return false;
        }
        if (!(value & ODP_ESPI_GENERAL_CHANNEL0_SUPPORTED)) {
            odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
            return false;
        }
        s->vwire_supported = value & ODP_ESPI_GENERAL_CHANNEL1_SUPPORTED;
        value = (value | ODP_ESPI_GENERAL_CRC_ENABLE) &
                ~ESPI_RESPONSE_MODIFIER;
        if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_GENERAL,
                                    &value, true, deadline)) {
            return false;
        }
        s->crc_enabled = true;
        if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_CH0,
                                    &value, false, deadline)) {
            return false;
        }
        supported = (value & ODP_ESPI_CH0_MAX_PAYLOAD_SUPPORTED_MASK) >>
                    ODP_ESPI_CH0_MAX_PAYLOAD_SUPPORTED_SHIFT;

        if (supported < ODP_ESPI_SIZE_64B_CODE ||
            supported > ODP_ESPI_SIZE_256B_CODE) {
            odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
            return false;
        }
        value &= ~(ODP_ESPI_CH0_MAX_PAYLOAD_SELECTED_MASK |
                   ODP_ESPI_CH0_MAX_READ_REQUEST_MASK |
                   ODP_ESPI_CH0_BUS_MASTER_ENABLE);
        value |= ODP_ESPI_CH0_ENABLE |
            (ODP_ESPI_SIZE_64B_CODE <<
             ODP_ESPI_CH0_MAX_PAYLOAD_SELECTED_SHIFT) |
            (ODP_ESPI_SIZE_64B_CODE << ODP_ESPI_CH0_MAX_READ_REQUEST_SHIFT);
        if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_CH0,
                                    &value, true, deadline)) {
            return false;
        }
        if (s->vwire_supported) {
            if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_CH1,
                                        &value, false, deadline)) {
                return false;
            }
            supported = 1 + ((value &
                              ODP_ESPI_CH1_MAX_COUNT_SUPPORTED_MASK) >>
                             ODP_ESPI_CH1_MAX_COUNT_SUPPORTED_SHIFT);
            if (supported < ODP_ESPI_VWIRE_MIN_COUNT) {
                odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
                return false;
            }
            s->vwire_limit = ODP_ESPI_VWIRE_MIN_COUNT;
            value &= ~ODP_ESPI_CH1_MAX_COUNT_SELECTED_MASK;
            value |= ODP_ESPI_CH1_ENABLE |
                     ((s->vwire_limit - 1) <<
                      ODP_ESPI_CH1_MAX_COUNT_SELECTED_SHIFT);
            if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_CH1,
                                        &value, true, deadline)) {
                return false;
            }
        }
        s->configured = true;
    }
    if (!s->ready) {
        if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_CH0,
                                    &value, false, deadline)) {
            return false;
        }
        ready = (value & (ODP_ESPI_CH0_ENABLE | ODP_ESPI_CH0_READY)) ==
            (ODP_ESPI_CH0_ENABLE | ODP_ESPI_CH0_READY);
        if (s->vwire_supported) {
            if (!odp_espi_configuration(s, ODP_ESPI_CONFIG_CH1,
                                        &value, false, deadline)) {
                return false;
            }
            s->vwire_ready =
                (value & (ODP_ESPI_CH1_ENABLE | ODP_ESPI_CH1_READY)) ==
                (ODP_ESPI_CH1_ENABLE | ODP_ESPI_CH1_READY);
            ready &= s->vwire_ready;
        }
    }
    if (ready && !s->ready) {
        if (s->alert) {
            s->alert = false;
            if (!odp_espi_exchange(s, ODP_ESPI_CMD_GET_STATUS,
                                   NULL, 0, deadline)) {
                return false;
            }
        }
        if (!odp_espi_drain_vwires(s, deadline)) {
            return false;
        }
    }
    s->ready = ready;
    return s->ready;
}

static int odp_espi_can_receive(void *opaque)
{
    OdpESPIControllerState *state = opaque;

    return sizeof(state->rx);
}

static void odp_espi_receive_vwires(OdpESPIControllerState *state)
{
    unsigned count = state->rx[1] + 1;
    unsigned index;

    for (index = 0; index < count; index++) {
        uint8_t group = state->rx[2 + index * 2];
        uint8_t value = state->rx[3 + index * 2];

        if (group <= ODP_ESPI_VWIRE_IRQ_HIGH) {
            unsigned irq = group * 128 + (value & ODP_ESPI_VWIRE_IRQ_MASK);

            if (irq < ODP_ESPI_IRQ_COUNT) {
                if (value & ODP_ESPI_VWIRE_IRQ_LEVEL) {
                    state->irq_level |= 1U << irq;
                } else {
                    state->irq_level &= ~(1U << irq);
                }
                odp_espi_update_irq(state);
            }
        }
    }
}

static void odp_espi_receive(void *opaque, const uint8_t *data, int length)
{
    OdpESPIControllerState *s = opaque;
    int index;

    for (index = 0; index < length; index++) {
        uint8_t byte = data[index];

        if (!s->receiving) {
            if (byte == ODP_ESPI_WIRE_ALERT) {
                s->alert = true;
                qemu_cond_broadcast(&s->cond);
                odp_espi_schedule_service(s);
                continue;
            }
            if (byte != ODP_ESPI_WIRE_DATA || !s->pending ||
                !s->expect_response || s->tx_pos != s->tx_size) {
                goto malformed;
            }
            s->receiving = true;
            continue;
        }
        if (!s->rx_size && byte == ODP_ESPI_RSP_WAIT_STATE) {
            continue;
        }
        if (s->rx_size == sizeof(s->rx)) {
            goto malformed;
        }
        s->rx[s->rx_size++] = byte;
        if (s->rx_size == 1) {
            if (byte == ODP_ESPI_RSP_NO_RESPONSE || (byte & 0xc0)) {
                goto malformed;
            }
            switch (byte & 0x0f) {
            case ODP_ESPI_RSP_ACCEPT:
                if (s->command == ODP_ESPI_CMD_GET_CONFIGURATION) {
                    s->rx_needed = 8;
                } else if (s->command == ODP_ESPI_CMD_PUT_NP ||
                           s->command == ODP_ESPI_CMD_GET_PC ||
                           s->command == ODP_ESPI_CMD_GET_VWIRE) {
                    s->rx_needed = 0;
                } else {
                    s->rx_needed = 4;
                }
                break;
            case ODP_ESPI_RSP_DEFER:
                if (s->command != ODP_ESPI_CMD_PUT_NP) {
                    goto malformed;
                }
                s->rx_needed = 4;
                break;
            case ODP_ESPI_RSP_NON_FATAL_ERROR:
            case ODP_ESPI_RSP_FATAL_ERROR:
                s->rx_needed = 4;
                break;
            default:
                goto malformed;
            }
        }
        if (!s->rx_needed && s->command == ODP_ESPI_CMD_GET_VWIRE &&
            s->rx_size == 2) {
            if ((byte & ~ODP_ESPI_VWIRE_COUNT_MASK) ||
                byte + 1 > s->vwire_limit) {
                goto malformed;
            }
            s->rx_needed = 5 + 2 * (byte + 1);
        }
        if (!s->rx_needed && s->rx_size == 4) {
            unsigned payload = lduw_be_p(s->rx + 2) &
                               ODP_ESPI_PACKET_LENGTH_MASK;

            if (s->rx[1] == ODP_ESPI_CYCLE_SUCCESS_DATA_ONLY) {
                if (!payload || payload > ESPI_MAX_PAYLOAD) {
                    goto malformed;
                }
            } else if (s->rx[1] == ODP_ESPI_CYCLE_UNSUCCESSFUL_ONLY) {
                payload = 0;
            } else {
                goto malformed;
            }
            s->rx_needed = 7 + payload;
        }
        if (s->rx_size != s->rx_needed) {
            continue;
        }
        if (s->crc_enabled && odp_espi_crc8(s->rx, s->rx_size)) {
            odp_espi_disconnect(s, ESPI_ERROR_CRC);
            return;
        }
        s->target_status = lduw_le_p(s->rx + s->rx_size - 3);
        if ((s->rx[0] & 0x0f) == ODP_ESPI_RSP_FATAL_ERROR ||
            (s->rx[0] & 0x0f) == ODP_ESPI_RSP_NON_FATAL_ERROR) {
            goto malformed;
        }
        s->pending = false;
        s->receiving = false;
        if (s->command == ODP_ESPI_CMD_GET_VWIRE) {
            odp_espi_receive_vwires(s);
        }
        if (s->target_status & ODP_ESPI_STATUS_VWIRE_AVAIL) {
            odp_espi_schedule_service(s);
        }
        qemu_cond_broadcast(&s->cond);
    }
    return;

malformed:
    odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
}

static bool odp_espi_drain_vwires(OdpESPIControllerState *state,
                                  int64_t deadline)
{
    while (state->vwire_ready &&
           (state->target_status & ODP_ESPI_STATUS_VWIRE_AVAIL)) {
        if (odp_espi_now_ms() >= deadline) {
            odp_espi_disconnect(state, ESPI_ERROR_TIMEOUT);
            return false;
        }
        if (!odp_espi_exchange(state, ODP_ESPI_CMD_GET_VWIRE,
                               NULL, 0, deadline)) {
            return false;
        }
    }
    return !state->failed;
}

static gboolean odp_espi_service(void *opaque)
{
    OdpESPIControllerState *s = opaque;
    int64_t deadline = odp_espi_now_ms() + s->timeout_ms;

    s->service_source = 0;
    if (!s->connected) {
        return G_SOURCE_REMOVE;
    }
    if (!s->busy) {
        s->busy = true;
        s->failed = false;
        if (odp_espi_prepare(s, deadline) || s->vwire_ready) {
            if (s->alert) {
                s->alert = false;
                odp_espi_exchange(s, ODP_ESPI_CMD_GET_STATUS,
                                   NULL, 0, deadline);
            }
            if (!s->failed) {
                odp_espi_drain_vwires(s, deadline);
            }
        }
        s->busy = false;
        qemu_cond_broadcast(&s->cond);
    }
    if (!s->busy && s->connected && !s->ready && !s->service_source) {
        s->service_source = g_timeout_add(10, odp_espi_service, s);
    }
    return G_SOURCE_REMOVE;
}

static void odp_espi_schedule_service(OdpESPIControllerState *s)
{
    if (!s->service_source && s->connected) {
        s->service_source = g_idle_add(odp_espi_service, s);
    }
}

static void odp_espi_event(void *opaque, QEMUChrEvent event)
{
    OdpESPIControllerState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        s->connected = true;
        s->ready = false;
        s->configured = false;
        s->reset_needed = true;
        odp_espi_clear_vwires(s);
        odp_espi_schedule_service(s);
        break;
    case CHR_EVENT_CLOSED:
        s->connected = false;
        odp_espi_cancel(s, ESPI_ERROR_LINK);
        if (s->service_source) {
            g_source_remove(s->service_source);
            s->service_source = 0;
        }
        break;
    default:
        break;
    }
}

static void odp_espi_unmap_window(OdpESPIWindow *window)
{
    if (window->mapped) {
        memory_region_del_subregion(get_system_memory(), &window->iomem);
        window->mapped = false;
    }
}

static bool odp_espi_window_valid(OdpESPIWindow *window,
                                  uint32_t base, uint32_t size)
{
    MemoryRegionSection section;

    if (!size || (uint64_t)base + size > (1ULL << 32)) {
        return false;
    }

    section = memory_region_find(get_system_memory(), base, size);
    if (section.mr) {
        memory_region_unref(section.mr);
        return false;
    }
    return true;
}

static void odp_espi_map_window(OdpESPIWindow *window)
{
    memory_region_set_size(&window->iomem, window->size);
    memory_region_add_subregion(get_system_memory(), window->base,
                               &window->iomem);
    window->mapped = true;
}

static MemTxResult odp_espi_access(OdpESPIWindow *window, hwaddr offset,
                                   uint64_t *value, unsigned size, bool write)
{
    OdpESPIControllerState *s = window->controller;
    int64_t deadline = odp_espi_now_ms() + s->timeout_ms;
    uint8_t request[7 + 4];
    unsigned index;
    uint16_t free_bit = write ? ODP_ESPI_STATUS_PC_FREE :
                               ODP_ESPI_STATUS_NP_FREE;
    MemTxResult result = MEMTX_ERROR;

    while (s->busy) {
        if (!qemu_in_vcpu_thread() || !odp_espi_wait(s, deadline)) {
            *value = UINT64_MAX;
            return MEMTX_ERROR;
        }
    }
    s->busy = true;
    s->failed = false;
    if (!s->connected) {
        odp_espi_set_error(s, ESPI_ERROR_LINK);
        goto out;
    }
    if (!window->enabled || offset + size > window->size ||
        !odp_espi_prepare(s, deadline) ||
        !odp_espi_drain_vwires(s, deadline)) {
        goto out;
    }
    while (!(s->target_status & free_bit)) {
        if (!odp_espi_exchange(s, ODP_ESPI_CMD_GET_STATUS, NULL, 0, deadline) ||
            !odp_espi_drain_vwires(s, deadline) ||
            (!(s->target_status & free_bit) && !odp_espi_wait(s, deadline))) {
            goto out;
        }
    }
    if (!write) {
        s->tag = (s->tag + 1) & 0x0f;
    }
    request[0] = write ? ODP_ESPI_CYCLE_MEM_WRITE32 :
                         ODP_ESPI_CYCLE_MEM_READ32;
    stw_be_p(request + 1, (s->tag << 12) | size);
    stl_be_p(request + 3, window->base + offset);
    for (index = 0; write && index < size; index++) {
        request[7 + index] = *value >> (index * 8);
    }
    if (!odp_espi_exchange(s, write ? ODP_ESPI_CMD_PUT_PC :
                                     ODP_ESPI_CMD_PUT_NP,
                           request, 7 + (write ? size : 0), deadline)) {
        goto out;
    }
    if (write) {
        result = MEMTX_OK;
        goto out;
    }
    if ((s->rx[0] & 0x0f) == ODP_ESPI_RSP_DEFER) {
        while (!(s->target_status & ODP_ESPI_STATUS_PC_AVAIL)) {
            if (!odp_espi_exchange(s, ODP_ESPI_CMD_GET_STATUS,
                                   NULL, 0, deadline) ||
                !odp_espi_drain_vwires(s, deadline) ||
                (!(s->target_status & ODP_ESPI_STATUS_PC_AVAIL) &&
                 !odp_espi_wait(s, deadline))) {
                goto out;
            }
        }
        if (!odp_espi_exchange(s, ODP_ESPI_CMD_GET_PC, NULL, 0, deadline)) {
            goto out;
        }
    }
    if ((s->rx[2] >> ODP_ESPI_PACKET_TAG_SHIFT) != s->tag) {
        odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
        goto out;
    }
    if (s->rx[1] == ODP_ESPI_CYCLE_UNSUCCESSFUL_ONLY) {
        odp_espi_set_error(s, ESPI_ERROR_COMPLETION);
        goto out;
    }
    if (s->rx_size != size + 7) {
        odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
        goto out;
    }
    *value = 0;
    for (index = 0; index < size; index++) {
        *value |= (uint64_t)s->rx[4 + index] << (index * 8);
    }
    result = MEMTX_OK;

out:
    if (result == MEMTX_OK && !odp_espi_drain_vwires(s, deadline)) {
        result = MEMTX_ERROR;
    }
    s->busy = false;
    if (result != MEMTX_OK) {
        *value = UINT64_MAX;
    }
    qemu_cond_broadcast(&s->cond);
    if (s->alert || (s->target_status & ODP_ESPI_STATUS_VWIRE_AVAIL)) {
        odp_espi_schedule_service(s);
    }
    return result;
}

static MemTxResult odp_espi_window_read(void *opaque, hwaddr offset,
                                       uint64_t *value, unsigned size,
                                       MemTxAttrs attrs)
{
    return odp_espi_access(opaque, offset, value, size, false);
}

static MemTxResult odp_espi_window_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size,
                                        MemTxAttrs attrs)
{
    return odp_espi_access(opaque, offset, &value, size, true);
}

static bool odp_espi_window_accepts(void *opaque, hwaddr offset,
                                    unsigned size, bool write, MemTxAttrs attrs)
{
    OdpESPIWindow *window = opaque;
    bool valid = size >= 1 && size <= 8 && !(offset & (size - 1));

    if (window == &window->controller->windows[ESPI_DATA_WINDOW_COUNT]) {
        hwaddr reg = offset % ESPI_CONTROL_STRIDE;

        valid = valid && size == 4 &&
                offset < ESPI_DATA_WINDOW_COUNT * ESPI_CONTROL_STRIDE &&
                (reg == ESPI_CONTROL_STATUS ||
                 reg == ESPI_CONTROL_DOORBELL ||
                 reg == ESPI_CONTROL_IRQ_ACK);
    }
    return valid;
}

static const MemoryRegionOps odp_espi_window_ops = {
    .read_with_attrs = odp_espi_window_read,
    .write_with_attrs = odp_espi_window_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8,
               .accepts = odp_espi_window_accepts },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void odp_espi_reset_hold(Object *obj, ResetType type)
{
    OdpESPIControllerState *s = ODP_ESPI_CONTROLLER(obj);
    unsigned index;

    if (s->busy) {
        odp_espi_disconnect(s, ESPI_ERROR_PROTOCOL);
    }
    for (index = 0; index < ESPI_WINDOW_COUNT; index++) {
        odp_espi_unmap_window(&s->windows[index]);
    }
    s->errors = 0;
    odp_espi_clear_vwires(s);
    s->ready = false;
    s->configured = false;
    s->reset_needed = true;
    s->tag = 0;
    s->target_status = 0;
    s->alert = false;
    for (index = 0; index < ESPI_WINDOW_COUNT; index++) {
        OdpESPIWindow *window = &s->windows[index];

        window->base = 0x090f0000 + index * 0x10000;
        window->size = ESPI_WINDOW_SIZE;
        window->enabled = odp_espi_window_valid(window, window->base,
                                                window->size);
        if (window->enabled) {
            odp_espi_map_window(window);
        } else {
            odp_espi_set_error(s, ESPI_ERROR_WINDOW);
        }
    }
    odp_espi_schedule_service(s);
}

static void odp_espi_controller_init(Object *obj)
{
    OdpESPIControllerState *s = ODP_ESPI_CONTROLLER(obj);
    unsigned index;

    for (index = 0; index < ODP_ESPI_IRQ_COUNT; index++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[index]);
    }
    for (index = 0; index < ESPI_WINDOW_COUNT; index++) {
        OdpESPIWindow *window = &s->windows[index];

        window->controller = s;
        memory_region_init_io(&window->iomem, obj, &odp_espi_window_ops,
                              window, "odp-espi-window", ESPI_WINDOW_SIZE);
        window->iomem.disable_reentrancy_guard = true;
    }
}

static void odp_espi_controller_realize(DeviceState *dev, Error **errp)
{
    OdpESPIControllerState *s = ODP_ESPI_CONTROLLER(dev);

    if (!s->timeout_ms || s->timeout_ms > INT_MAX) {
        error_setg(errp, "eSPI timeout-ms must be between 1 and %d", INT_MAX);
        return;
    }
    qemu_cond_init(&s->cond);
    odp_espi_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
    qemu_chr_fe_set_handlers(&s->chr, odp_espi_can_receive, odp_espi_receive,
                             odp_espi_event, NULL, s, NULL, true);
}

static void odp_espi_controller_unrealize(DeviceState *dev)
{
    OdpESPIControllerState *s = ODP_ESPI_CONTROLLER(dev);
    unsigned index;

    qemu_chr_fe_set_handlers(&s->chr, NULL, NULL, NULL, NULL, NULL,
                             NULL, false);
    if (s->service_source) {
        g_source_remove(s->service_source);
    }
    if (s->write_watch) {
        g_source_remove(s->write_watch);
    }
    for (index = 0; index < ESPI_WINDOW_COUNT; index++) {
        odp_espi_unmap_window(&s->windows[index]);
    }
    qemu_cond_destroy(&s->cond);
}

static const Property odp_espi_controller_properties[] = {
    DEFINE_PROP_CHR("chardev", OdpESPIControllerState, chr),
    DEFINE_PROP_UINT32("timeout-ms", OdpESPIControllerState, timeout_ms, 1000),
};

static const VMStateDescription odp_espi_vmstate = {
    .name = TYPE_ODP_ESPI_CONTROLLER,
    .unmigratable = true,
};

static void odp_espi_controller_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "ODP socket-backed eSPI controller";
    dc->realize = odp_espi_controller_realize;
    dc->unrealize = odp_espi_controller_unrealize;
    dc->vmsd = &odp_espi_vmstate;
    dc->hotpluggable = false;
    rc->phases.hold = odp_espi_reset_hold;
    device_class_set_props(dc, odp_espi_controller_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo odp_espi_controller_info = {
    .name          = TYPE_ODP_ESPI_CONTROLLER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OdpESPIControllerState),
    .instance_init = odp_espi_controller_init,
    .class_init    = odp_espi_controller_class_init,
};

static void odp_espi_controller_register_types(void)
{
    type_register_static(&odp_espi_controller_info);
}

type_init(odp_espi_controller_register_types)
