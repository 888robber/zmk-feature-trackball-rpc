/*
 * LisM trackball live-settings RPC service.
 *
 * Exposes a custom BLE GATT service (separate UUIDs from ZMK Studio's own
 * service/characteristic — this does not touch the Studio RPC protocol at
 * all) that lets a companion app read and live-adjust:
 *   - cursor sensitivity (CPI)
 *   - "precision mode" secondary CPI + on/off toggle
 *   - scroll direction invert              (hardware effect wired in a
 *   - automatic mouse layer idle threshold  later phase; stored/echoed here)
 *
 * All values persist across reboot via the Zephyr settings subsystem.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <paw3222.h>

LOG_MODULE_REGISTER(trackball_rpc, CONFIG_ZMK_LOG_LEVEL);

#define TB_RPC_NODE DT_NODELABEL(trackball_rpc)
#if !DT_NODE_HAS_STATUS(TB_RPC_NODE, okay)
#error "trackball_rpc devicetree node not found or not okay; add it to trackball.overlay"
#endif

/* Randomly generated 128-bit UUIDs, distinct from ZMK Studio's own service
 * (00000000-0196-6107-c967-c5cfb1c2482a / ...c967-c5cfb1c2482a char). */
#define BT_UUID_TRACKBALL_RPC_SERVICE_VAL                                                        \
    BT_UUID_128_ENCODE(0x7a2e2f10, 0x9c41, 0x4d0a, 0x8b0f, 0x1a2b3c4d5e6f)
#define BT_UUID_TRACKBALL_RPC_CHAR_VAL                                                           \
    BT_UUID_128_ENCODE(0x7a2e2f11, 0x9c41, 0x4d0a, 0x8b0f, 0x1a2b3c4d5e6f)

static const struct bt_uuid_128 trackball_rpc_service_uuid =
    BT_UUID_INIT_128(BT_UUID_TRACKBALL_RPC_SERVICE_VAL);
static const struct bt_uuid_128 trackball_rpc_char_uuid =
    BT_UUID_INIT_128(BT_UUID_TRACKBALL_RPC_CHAR_VAL);

/* Write opcodes (app -> keyboard) */
#define OP_SET_CURSOR_CPI    0x01 /* payload: u16 LE */
#define OP_SET_PRECISION_CPI 0x02 /* payload: u16 LE */
#define OP_SET_PRECISION_ON  0x03 /* payload: u8 (0/1) */
#define OP_SET_SCROLL_INVERT 0x04 /* payload: u8 (0/1) */
#define OP_SET_AML_IDLE_MS   0x05 /* payload: u16 LE */
#define OP_GET_ALL           0x10 /* no payload */

/* Notify opcode (keyboard -> app), sent after every write and on subscribe */
#define OP_STATE_NOTIFY 0x80

/* PAW3222 register step is 38, valid range is 16*38..127*38 (see
 * zmk-driver-paw3222's RES_MIN/RES_MAX). */
#define CPI_MIN 608
#define CPI_MAX 4826

struct trackball_rpc_state {
    uint16_t cursor_cpi;
    uint16_t precision_cpi;
    uint8_t precision_active;
    uint8_t scroll_invert;
    uint16_t aml_idle_ms;
} __packed;

static struct trackball_rpc_state state = {
    .cursor_cpi = 1200,
    .precision_cpi = 800,
    .precision_active = 0,
    .scroll_invert = 0,
    .aml_idle_ms = 1000,
};

static const struct device *trackball_dev;
static bool notify_enabled;

/* Defined in runtime_input_processors.c, added in a later phase. Weak
 * no-op stubs here so this file links standalone until that lands. */
__weak void trackball_rpc_apply_scroll_invert(bool invert) {
    ARG_UNUSED(invert);
}
__weak void trackball_rpc_apply_aml_idle_ms(uint16_t idle_ms) {
    ARG_UNUSED(idle_ms);
}

static void apply_cursor_sensitivity(void) {
    if (!trackball_dev || !device_is_ready(trackball_dev)) {
        return;
    }

    uint16_t cpi = state.precision_active ? state.precision_cpi : state.cursor_cpi;
    int ret = paw32xx_set_resolution(trackball_dev, cpi);
    if (ret < 0) {
        LOG_ERR("paw32xx_set_resolution(%u) failed: %d", cpi, ret);
    }
}

static void notify_state(struct bt_conn *conn) {
    ARG_UNUSED(conn);
    if (!notify_enabled) {
        return;
    }

    uint8_t packet[9];
    packet[0] = OP_STATE_NOTIFY;
    sys_put_le16(state.cursor_cpi, &packet[1]);
    sys_put_le16(state.precision_cpi, &packet[3]);
    packet[5] = state.precision_active;
    packet[6] = state.scroll_invert;
    sys_put_le16(state.aml_idle_ms, &packet[7]);

    int ret = bt_gatt_notify_uuid(NULL, &trackball_rpc_char_uuid.uuid, trackball_rpc_svc.attrs,
                                    packet, sizeof(packet));
    if (ret < 0) {
        LOG_WRN("trackball-rpc notify failed: %d", ret);
    }
}

static void save_state(void) {
    settings_save_one("trackball_rpc/state", &state, sizeof(state));
}

static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                          uint16_t len, uint16_t offset, uint8_t flags) {
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = buf;
    uint8_t opcode = data[0];
    bool changed = false;

    switch (opcode) {
    case OP_SET_CURSOR_CPI:
        if (len >= 3) {
            state.cursor_cpi = CLAMP(sys_get_le16(&data[1]), CPI_MIN, CPI_MAX);
            if (!state.precision_active) {
                apply_cursor_sensitivity();
            }
            changed = true;
        }
        break;
    case OP_SET_PRECISION_CPI:
        if (len >= 3) {
            state.precision_cpi = CLAMP(sys_get_le16(&data[1]), CPI_MIN, CPI_MAX);
            if (state.precision_active) {
                apply_cursor_sensitivity();
            }
            changed = true;
        }
        break;
    case OP_SET_PRECISION_ON:
        if (len >= 2) {
            state.precision_active = data[1] ? 1 : 0;
            apply_cursor_sensitivity();
            changed = true;
        }
        break;
    case OP_SET_SCROLL_INVERT:
        if (len >= 2) {
            state.scroll_invert = data[1] ? 1 : 0;
            trackball_rpc_apply_scroll_invert(state.scroll_invert);
            changed = true;
        }
        break;
    case OP_SET_AML_IDLE_MS:
        if (len >= 3) {
            state.aml_idle_ms = sys_get_le16(&data[1]);
            trackball_rpc_apply_aml_idle_ms(state.aml_idle_ms);
            changed = true;
        }
        break;
    case OP_GET_ALL:
        break;
    default:
        LOG_WRN("Unknown trackball-rpc opcode: 0x%02x", opcode);
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    if (changed) {
        save_state();
    }
    notify_state(conn);

    return len;
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (notify_enabled) {
        notify_state(NULL);
    }
}

BT_GATT_SERVICE_DEFINE(trackball_rpc_svc, BT_GATT_PRIMARY_SERVICE(&trackball_rpc_service_uuid),
                        BT_GATT_CHARACTERISTIC(&trackball_rpc_char_uuid.uuid,
                                                 BT_GATT_CHRC_WRITE |
                                                     BT_GATT_CHRC_WRITE_WITHOUT_RESP |
                                                     BT_GATT_CHRC_NOTIFY,
                                                 BT_GATT_PERM_WRITE, NULL, on_write, NULL),
                        BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static int trackball_rpc_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                        void *cb_arg) {
    ARG_UNUSED(name);
    if (len != sizeof(state)) {
        return -EINVAL;
    }
    ssize_t ret = read_cb(cb_arg, &state, sizeof(state));
    return ret < 0 ? -EIO : 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(trackball_rpc, "trackball_rpc", NULL, trackball_rpc_settings_set,
                                 NULL, NULL);

static int trackball_rpc_init(void) {
    trackball_dev = DEVICE_DT_GET(DT_PHANDLE(TB_RPC_NODE, device));

    settings_subsys_init();
    settings_load_subtree("trackball_rpc");

    apply_cursor_sensitivity();
    trackball_rpc_apply_scroll_invert(state.scroll_invert);
    trackball_rpc_apply_aml_idle_ms(state.aml_idle_ms);

    LOG_INF("trackball-rpc init: cursor_cpi=%u precision_cpi=%u precision=%u scroll_invert=%u "
            "aml_idle_ms=%u",
            state.cursor_cpi, state.precision_cpi, state.precision_active, state.scroll_invert,
            state.aml_idle_ms);

    return 0;
}

SYS_INIT(trackball_rpc_init, APPLICATION, 90);
