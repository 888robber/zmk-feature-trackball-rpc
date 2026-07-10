/*
 * LisM combo live-editing BLE RPC service.
 *
 * Exposes a custom GATT service (its own UUIDs, independent from both ZMK
 * Studio's protocol and the trackball RPC service) that lets the companion
 * app add/edit/delete runtime combos (lism_runtime_combo.c) live.
 *
 * Wire protocol (all multi-byte fields little-endian):
 *   Writes (app -> keyboard):
 *     0x01 SET_COMBO    [slot:u8][record:21B]
 *     0x02 DELETE_COMBO [slot:u8]
 *     0x03 CLEAR_ALL
 *     0x10 GET_ALL
 *   Notifies (keyboard -> app), always sent as a full dump — after every
 *   processed command and on notification subscribe:
 *     0x82 DUMP_BEGIN [active_count:u8][max_slots:u8][max_keys:u8]
 *     0x80 COMBO_RECORD [slot:u8][record:21B]   (one per active slot)
 *     0x81 DUMP_END [last_op:u8][result:u8]     (result 0 = ok, else +errno)
 *
 *   record (21B): flags(1) key_count(1) key_positions(4, 0xFF=unused)
 *                 timeout_ms(2) require_prior_idle_ms(2) layer_mask(1)
 *                 behavior_local_id(2) param1(4) param2(4)
 *
 * GATT write callbacks run on the Bluetooth RX thread, but the combo engine
 * lives on the system workqueue (same context as key events). Commands are
 * therefore queued and processed from a work item — the write callback never
 * touches engine state directly.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "lism_combo.h"

LOG_MODULE_REGISTER(lism_combo_rpc, CONFIG_ZMK_LOG_LEVEL);

#define BT_UUID_COMBO_RPC_SERVICE_VAL                                                              \
    BT_UUID_128_ENCODE(0x7a2e2f20, 0x9c41, 0x4d0a, 0x8b0f, 0x1a2b3c4d5e6f)
#define BT_UUID_COMBO_RPC_CHAR_VAL                                                                 \
    BT_UUID_128_ENCODE(0x7a2e2f21, 0x9c41, 0x4d0a, 0x8b0f, 0x1a2b3c4d5e6f)

static const struct bt_uuid_128 combo_rpc_service_uuid =
    BT_UUID_INIT_128(BT_UUID_COMBO_RPC_SERVICE_VAL);
static const struct bt_uuid_128 combo_rpc_char_uuid = BT_UUID_INIT_128(BT_UUID_COMBO_RPC_CHAR_VAL);

#define OP_SET_COMBO    0x01
#define OP_DELETE_COMBO 0x02
#define OP_CLEAR_ALL    0x03
#define OP_GET_ALL      0x10

#define OP_COMBO_RECORD 0x80
#define OP_DUMP_END     0x81
#define OP_DUMP_BEGIN   0x82

#define CMD_MAX_LEN (2 + LISM_COMBO_RECORD_WIRE_LEN)

struct combo_cmd {
    uint8_t len;
    uint8_t data[CMD_MAX_LEN];
};

K_MSGQ_DEFINE(combo_cmd_q, sizeof(struct combo_cmd), 4, 4);

static bool notify_enabled;
static uint8_t last_op;
static uint8_t last_result;

/* Dump progress: -2 = BEGIN pending, 0..SLOTS-1 = next slot to try,
 * SLOTS = END pending, -1 = idle. Only touched on the system workqueue. */
static int dump_cursor = -1;

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags);

BT_GATT_SERVICE_DEFINE(combo_rpc_svc, BT_GATT_PRIMARY_SERVICE(&combo_rpc_service_uuid),
                       BT_GATT_CHARACTERISTIC(&combo_rpc_char_uuid.uuid,
                                              BT_GATT_CHRC_WRITE |
                                                  BT_GATT_CHRC_WRITE_WITHOUT_RESP |
                                                  BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_WRITE, NULL, on_write, NULL),
                       BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static int notify_packet(const uint8_t *packet, size_t len) {
    return bt_gatt_notify_uuid(NULL, &combo_rpc_char_uuid.uuid, combo_rpc_svc.attrs, packet, len);
}

static void dump_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dump_work, dump_work_handler);

static void dump_work_handler(struct k_work *work) {
    if (!notify_enabled) {
        dump_cursor = -1;
        return;
    }

    while (dump_cursor != -1) {
        uint8_t packet[2 + LISM_COMBO_RECORD_WIRE_LEN];
        size_t len;
        int next;

        if (dump_cursor == -2) {
            packet[0] = OP_DUMP_BEGIN;
            packet[1] = (uint8_t)lism_combo_active_count();
            packet[2] = LISM_COMBO_SLOTS;
            packet[3] = LISM_COMBO_MAX_KEYS;
            len = 4;
            next = 0;
        } else if (dump_cursor >= LISM_COMBO_SLOTS) {
            packet[0] = OP_DUMP_END;
            packet[1] = last_op;
            packet[2] = last_result;
            len = 3;
            next = -1;
        } else {
            const struct lism_combo_record *rec = lism_combo_get((uint8_t)dump_cursor);
            if (rec == NULL) {
                dump_cursor++;
                continue;
            }
            packet[0] = OP_COMBO_RECORD;
            packet[1] = (uint8_t)dump_cursor;
            memcpy(&packet[2], rec, LISM_COMBO_RECORD_WIRE_LEN);
            len = 2 + LISM_COMBO_RECORD_WIRE_LEN;
            next = dump_cursor + 1;
        }

        int ret = notify_packet(packet, len);
        if (ret == -ENOMEM || ret == -EAGAIN || ret == -ENOBUFS) {
            /* TX buffers exhausted — retry shortly without advancing. */
            k_work_schedule(&dump_work, K_MSEC(20));
            return;
        }
        if (ret < 0) {
            LOG_WRN("combo-rpc dump notify failed: %d, aborting dump", ret);
            dump_cursor = -1;
            return;
        }

        dump_cursor = next;
    }
}

static void start_dump(void) {
    if (!notify_enabled) {
        return;
    }
    dump_cursor = -2;
    k_work_schedule(&dump_work, K_NO_WAIT);
}

static void process_cmd(const struct combo_cmd *cmd) {
    const uint8_t *data = cmd->data;
    uint8_t op = data[0];
    int result = 0;

    switch (op) {
    case OP_SET_COMBO: {
        if (cmd->len < 2 + LISM_COMBO_RECORD_WIRE_LEN) {
            result = EINVAL;
            break;
        }
        struct lism_combo_record rec;
        memcpy(&rec, &data[2], LISM_COMBO_RECORD_WIRE_LEN);
        int ret = lism_combo_set(data[1], &rec);
        result = ret < 0 ? -ret : 0;
        break;
    }
    case OP_DELETE_COMBO: {
        if (cmd->len < 2) {
            result = EINVAL;
            break;
        }
        int ret = lism_combo_delete(data[1]);
        result = ret < 0 ? -ret : 0;
        break;
    }
    case OP_CLEAR_ALL:
        lism_combo_clear_all();
        break;
    case OP_GET_ALL:
        break;
    default:
        result = ENOTSUP;
        break;
    }

    last_op = op;
    last_result = (uint8_t)MIN(result, 255);
}

static void cmd_work_handler(struct k_work *work) {
    struct combo_cmd cmd;
    bool processed_any = false;

    while (k_msgq_get(&combo_cmd_q, &cmd, K_NO_WAIT) == 0) {
        process_cmd(&cmd);
        processed_any = true;
    }

    /* Also reached with an empty queue from the CCC-subscribe path, where
     * the dump itself is the point. */
    ARG_UNUSED(processed_any);
    start_dump();
}

static K_WORK_DEFINE(cmd_work, cmd_work_handler);

static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags) {
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len < 1 || len > CMD_MAX_LEN) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct combo_cmd cmd = {.len = (uint8_t)len};
    memcpy(cmd.data, buf, len);

    if (k_msgq_put(&combo_cmd_q, &cmd, K_NO_WAIT) != 0) {
        LOG_WRN("combo-rpc command queue full, dropping opcode 0x%02x", cmd.data[0]);
        return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
    }
    k_work_submit(&cmd_work);

    return len;
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (notify_enabled) {
        /* Push the current combo table to the freshly subscribed app —
         * routed through the workqueue so dump state stays single-context. */
        k_work_submit(&cmd_work);
    }
}
