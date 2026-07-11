/*
 * LisM runtime-editable combo matcher.
 *
 * ZMK's own combo engine (app/src/combo.c) bakes every combo into a
 * compile-time `static const` array, so combos cannot be changed at
 * runtime. This module is a close port of that engine's capture/release
 * design, with the combo table replaced by a runtime-mutable slot array
 * that persists via the Zephyr settings subsystem and is edited over BLE
 * by lism_combo_rpc.c.
 *
 * Safety design (mirrors combo.c's position_state_down): a key position
 * that is not part of any *active* combo slot is never captured — its
 * events bubble through completely untouched, so keys outside the user's
 * combos behave exactly as before. The companion app additionally refuses
 * to register positions whose binding is a hold-tap (Mod-Tap/Layer-Tap),
 * so this engine never competes with hold-tap's own event capturing.
 *
 * Coexists with ZMK's real combo engine (used for the I+O gesture combo):
 * events this module releases continue down the listener chain into it.
 */

#include <limits.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/matrix.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

#include "lism_combo.h"

LOG_MODULE_REGISTER(lism_combo, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(LISM_COMBO_SLOTS <= 32, "candidate bitmask is a single uint32_t");

/* Kept clear of the virtual key positions used by ZMK's real combos
 * (indexes 0..ZMK_COMBOS_LEN-1) and by input-processor behaviors (a small
 * region starting right at ZMK_COMBOS_LEN). */
#define LISM_COMBO_VIRTUAL_POS(slot) (ZMK_VIRTUAL_KEY_POSITION_COMBO(ZMK_COMBOS_LEN) + 32 + (slot))

static struct lism_combo_record slots[LISM_COMBO_SLOTS];
static bool settings_found;

struct rt_active_combo {
    int8_t slot; /* -1 = entry unused */
    uint8_t keys_pressed_count;
    struct zmk_position_state_changed_event key_positions_pressed[LISM_COMBO_MAX_KEYS];
};

static struct rt_active_combo active_combos[CONFIG_LISM_COMBO_RPC_MAX_PRESSED];
static uint8_t active_combo_count;

static uint8_t pressed_keys_count;
static struct zmk_position_state_changed_event pressed_keys[LISM_COMBO_MAX_KEYS];
static uint32_t candidates;
static int8_t fully_pressed_combo = -1;

static struct k_work_delayable timeout_task;
static int64_t timeout_task_timeout_at;

static int64_t last_tapped_timestamp = INT32_MIN;
static int64_t last_combo_timestamp = INT32_MIN;

static bool slot_is_active(int slot) {
    return (slots[slot].flags & LISM_COMBO_FLAG_ACTIVE) && slots[slot].key_count >= 2;
}

static bool slot_uses_position(int slot, uint32_t position) {
    for (int k = 0; k < slots[slot].key_count && k < LISM_COMBO_MAX_KEYS; k++) {
        if (slots[slot].key_positions[k] == position) {
            return true;
        }
    }
    return false;
}

static bool slot_active_on_layer(int slot, uint8_t layer) {
    if (slots[slot].layer_mask == 0) {
        return true;
    }
    return layer < 8 && (slots[slot].layer_mask & BIT(layer));
}

static bool is_quick_tap(int slot, int64_t timestamp) {
    return (last_tapped_timestamp + slots[slot].require_prior_idle_ms) > timestamp;
}

static int invoke_slot_behavior(int slot, int64_t timestamp, bool pressed) {
    const struct lism_combo_record *c = &slots[slot];
    const char *name = zmk_behavior_find_behavior_name_from_local_id(c->behavior_local_id);
    if (!name) {
        LOG_ERR("combo slot %d: behavior local id %u not found, skipping", slot,
                c->behavior_local_id);
        return -ENODEV;
    }

    struct zmk_behavior_binding binding = {
        .behavior_dev = name,
        .param1 = c->param1,
        .param2 = c->param2,
    };
    struct zmk_behavior_binding_event event = {
        .position = LISM_COMBO_VIRTUAL_POS(slot),
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    if (pressed) {
        last_combo_timestamp = timestamp;
    }

    return zmk_behavior_invoke_binding(&binding, event, pressed);
}

static int setup_candidates_for_first_keypress(uint32_t position, int64_t timestamp) {
    int number_of_candidates = 0;
    uint8_t highest_active_layer = zmk_keymap_highest_layer_active();

    for (int i = 0; i < LISM_COMBO_SLOTS; i++) {
        if (slot_is_active(i) && slot_uses_position(i, position) &&
            slot_active_on_layer(i, highest_active_layer) && !is_quick_tap(i, timestamp)) {
            candidates |= BIT(i);
            number_of_candidates++;
        }
    }

    return number_of_candidates;
}

static int filter_candidates(uint32_t position) {
    for (int i = 0; i < LISM_COMBO_SLOTS; i++) {
        if ((candidates & BIT(i)) && !slot_uses_position(i, position)) {
            candidates &= ~BIT(i);
        }
    }
    return POPCOUNT(candidates);
}

static int64_t first_candidate_timeout(void) {
    if (pressed_keys_count == 0) {
        return LLONG_MAX;
    }

    int64_t first_timeout = LLONG_MAX;
    for (int i = 0; i < LISM_COMBO_SLOTS; i++) {
        if (candidates & BIT(i)) {
            first_timeout = MIN(first_timeout, (int64_t)slots[i].timeout_ms);
        }
    }
    if (first_timeout == LLONG_MAX) {
        return LLONG_MAX;
    }

    return pressed_keys[0].data.timestamp + first_timeout;
}

static int filter_timed_out_candidates(int64_t timestamp) {
    int remaining = 0;
    for (int i = 0; i < LISM_COMBO_SLOTS; i++) {
        if (!(candidates & BIT(i))) {
            continue;
        }
        if (pressed_keys[0].data.timestamp + slots[i].timeout_ms > timestamp) {
            remaining++;
        } else {
            candidates &= ~BIT(i);
        }
    }
    return remaining;
}

static int capture_pressed_key(const struct zmk_position_state_changed *ev) {
    if (pressed_keys_count == LISM_COMBO_MAX_KEYS) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    pressed_keys[pressed_keys_count++] = copy_raised_zmk_position_state_changed(ev);
    return ZMK_EV_EVENT_CAPTURED;
}

const struct zmk_listener zmk_listener_lism_combo;

static int release_pressed_keys(void) {
    uint8_t count = pressed_keys_count;
    pressed_keys_count = 0;
    for (int i = 0; i < count; i++) {
        struct zmk_position_state_changed_event *ev = &pressed_keys[i];
        if (i == 0) {
            LOG_DBG("lism-combo: releasing position event %d", ev->data.position);
            ZMK_EVENT_RELEASE(*ev);
        } else {
            /* Same as combo.c: later keys are re-raised from the start of
             * the chain so overlapping combos can begin a fresh match. */
            LOG_DBG("lism-combo: reraising position event %d", ev->data.position);
            ZMK_EVENT_RAISE(*ev);
        }
    }

    return count;
}

static void move_pressed_keys_to_active_combo(struct rt_active_combo *active) {
    int combo_length = MIN(pressed_keys_count, slots[active->slot].key_count);
    for (int i = 0; i < combo_length; i++) {
        active->key_positions_pressed[i] = pressed_keys[i];
    }
    active->keys_pressed_count = combo_length;

    for (int i = 0; i + combo_length < pressed_keys_count; i++) {
        pressed_keys[i] = pressed_keys[i + combo_length];
    }
    pressed_keys_count -= combo_length;
}

static struct rt_active_combo *store_active_combo(int8_t slot) {
    for (int i = 0; i < CONFIG_LISM_COMBO_RPC_MAX_PRESSED; i++) {
        if (active_combos[i].slot == -1) {
            active_combos[i].slot = slot;
            active_combo_count++;
            return &active_combos[i];
        }
    }
    LOG_ERR("Unable to store combo; already %d active", CONFIG_LISM_COMBO_RPC_MAX_PRESSED);
    return NULL;
}

static void activate_combo(int8_t slot) {
    struct rt_active_combo *active = store_active_combo(slot);
    if (active == NULL) {
        release_pressed_keys();
        return;
    }
    move_pressed_keys_to_active_combo(active);
    invoke_slot_behavior(slot, active->key_positions_pressed[0].data.timestamp, true);
}

static void deactivate_combo(int index) {
    active_combo_count--;
    if (index != active_combo_count) {
        memcpy(&active_combos[index], &active_combos[active_combo_count],
               sizeof(struct rt_active_combo));
    }
    active_combos[active_combo_count] = (struct rt_active_combo){0};
    active_combos[active_combo_count].slot = -1;
}

/* Returns true if the released key belonged to an active combo. */
static bool release_combo_key(uint32_t position, int64_t timestamp) {
    for (int idx = 0; idx < active_combo_count; idx++) {
        struct rt_active_combo *active = &active_combos[idx];

        bool key_released = false;
        bool all_keys_pressed = active->keys_pressed_count == slots[active->slot].key_count;
        bool all_keys_released = true;
        for (int i = 0; i < active->keys_pressed_count; i++) {
            if (key_released) {
                active->key_positions_pressed[i - 1] = active->key_positions_pressed[i];
                all_keys_released = false;
            } else if (active->key_positions_pressed[i].data.position != position) {
                all_keys_released = false;
            } else {
                key_released = true;
            }
        }

        if (key_released) {
            active->keys_pressed_count--;
            bool slow_release = slots[active->slot].flags & LISM_COMBO_FLAG_SLOW_RELEASE;
            if ((slow_release && all_keys_released) || (!slow_release && all_keys_pressed)) {
                invoke_slot_behavior(active->slot, timestamp, false);
            }
            if (all_keys_released) {
                deactivate_combo(idx);
            }
            return true;
        }
    }
    return false;
}

static void update_timeout_task(void);

static int cleanup(void) {
    k_work_cancel_delayable(&timeout_task);
    candidates = 0;
    if (fully_pressed_combo != -1) {
        int8_t slot = fully_pressed_combo;
        fully_pressed_combo = -1;
        activate_combo(slot);
    }
    return release_pressed_keys();
}

static void update_timeout_task(void) {
    int64_t first_timeout = first_candidate_timeout();
    if (timeout_task_timeout_at == first_timeout) {
        return;
    }
    if (first_timeout == LLONG_MAX) {
        timeout_task_timeout_at = 0;
        k_work_cancel_delayable(&timeout_task);
        return;
    }
    if (k_work_schedule(&timeout_task, K_MSEC(first_timeout - k_uptime_get())) >= 0) {
        timeout_task_timeout_at = first_timeout;
    }
}

static int position_state_down(const zmk_event_t *ev, struct zmk_position_state_changed *data) {
    int num_candidates;
    if (!pressed_keys_count) {
        num_candidates = setup_candidates_for_first_keypress(data->position, data->timestamp);
        if (num_candidates == 0) {
            return ZMK_EV_EVENT_BUBBLE;
        }
    } else {
        filter_timed_out_candidates(data->timestamp);
        num_candidates = filter_candidates(data->position);
    }

    LOG_DBG("lism-combo: capturing position event %d", data->position);
    int ret = capture_pressed_key(data);
    update_timeout_task();

    if (num_candidates == 0) {
        cleanup();
        return ret;
    }

    /* Unlike combo.c we don't keep the table sorted shortest-first, so
     * scan every remaining candidate for an exact-length match. */
    for (int i = 0; i < LISM_COMBO_SLOTS; i++) {
        if (!(candidates & BIT(i))) {
            continue;
        }
        if (slots[i].key_count == pressed_keys_count) {
            fully_pressed_combo = i;
            if (num_candidates == 1) {
                cleanup();
            }
            break;
        }
    }

    return ret;
}

static int position_state_up(const zmk_event_t *ev, struct zmk_position_state_changed *data) {
    int released_keys = cleanup();
    if (release_combo_key(data->position, data->timestamp)) {
        return ZMK_EV_EVENT_HANDLED;
    }
    if (released_keys > 1) {
        /* Down events beyond the first were re-raised; re-raise the up
         * event too so e.g. hold-taps see them in the right order. */
        struct zmk_position_state_changed_event dupe_ev =
            copy_raised_zmk_position_state_changed(data);
        ZMK_EVENT_RAISE(dupe_ev);
        return ZMK_EV_EVENT_CAPTURED;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

static void combo_timeout_handler(struct k_work *item) {
    if (timeout_task_timeout_at == 0 || k_uptime_get() < timeout_task_timeout_at) {
        return;
    }
    if (filter_timed_out_candidates(timeout_task_timeout_at) == 0) {
        cleanup();
    }
    update_timeout_task();
}

static int position_state_changed_listener(const zmk_event_t *ev) {
    struct zmk_position_state_changed *data = as_zmk_position_state_changed(ev);
    if (data == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (data->state) {
        return position_state_down(ev, data);
    } else {
        return position_state_up(ev, data);
    }
}

static int keycode_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev->state && !is_mod(ev->usage_page, ev->keycode) &&
        ev->timestamp > last_combo_timestamp) {
        last_tapped_timestamp = ev->timestamp;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

static int lism_combo_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        return position_state_changed_listener(eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        return keycode_state_changed_listener(eh);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(lism_combo, lism_combo_listener);
ZMK_SUBSCRIPTION(lism_combo, zmk_position_state_changed);
ZMK_SUBSCRIPTION(lism_combo, zmk_keycode_state_changed);

/* ---- runtime configuration (called from the system workqueue only) ---- */

static void save_slots(void) {
    settings_save_one("lism_combo/slots", slots, sizeof(slots));
}

/* Any in-flight match is aborted before touching the table: captured keys
 * are released back into the normal chain, and if the edited slot's combo
 * is physically held right now, its behavior is force-released so nothing
 * ends up stuck. */
static void quiesce_slot(uint8_t slot) {
    if (pressed_keys_count > 0 || candidates != 0) {
        fully_pressed_combo = -1;
        cleanup();
    }

    for (int idx = 0; idx < active_combo_count; idx++) {
        if (active_combos[idx].slot == slot) {
            invoke_slot_behavior(slot, k_uptime_get(), false);
            deactivate_combo(idx);
            break;
        }
    }
}

static int validate_record(const struct lism_combo_record *rec) {
    if (rec->key_count < 2 || rec->key_count > LISM_COMBO_MAX_KEYS) {
        return -EINVAL;
    }
    for (int i = 0; i < rec->key_count; i++) {
        if (rec->key_positions[i] >= ZMK_KEYMAP_LEN) {
            return -EINVAL;
        }
        for (int j = i + 1; j < rec->key_count; j++) {
            if (rec->key_positions[i] == rec->key_positions[j]) {
                return -EINVAL;
            }
        }
    }
    if (rec->timeout_ms < 10 || rec->timeout_ms > 2000) {
        return -EINVAL;
    }
    if (zmk_behavior_find_behavior_name_from_local_id(rec->behavior_local_id) == NULL) {
        return -ENODEV;
    }
    return 0;
}

int lism_combo_set(uint8_t slot, const struct lism_combo_record *rec) {
    if (slot >= LISM_COMBO_SLOTS) {
        return -EINVAL;
    }
    int ret = validate_record(rec);
    if (ret < 0) {
        return ret;
    }

    quiesce_slot(slot);

    slots[slot] = *rec;
    slots[slot].flags |= LISM_COMBO_FLAG_ACTIVE;
    for (int i = slots[slot].key_count; i < LISM_COMBO_MAX_KEYS; i++) {
        slots[slot].key_positions[i] = 0xFF;
    }

    save_slots();
    LOG_INF("combo slot %u set: %u keys, timeout %ums, behavior id %u", slot, rec->key_count,
            rec->timeout_ms, rec->behavior_local_id);
    return 0;
}

int lism_combo_delete(uint8_t slot) {
    if (slot >= LISM_COMBO_SLOTS) {
        return -EINVAL;
    }

    quiesce_slot(slot);
    memset(&slots[slot], 0, sizeof(slots[slot]));
    save_slots();
    LOG_INF("combo slot %u deleted", slot);
    return 0;
}

void lism_combo_clear_all(void) {
    for (uint8_t slot = 0; slot < LISM_COMBO_SLOTS; slot++) {
        if (slot_is_active(slot)) {
            quiesce_slot(slot);
        }
        memset(&slots[slot], 0, sizeof(slots[slot]));
    }
    save_slots();
    LOG_INF("all combo slots cleared");
}

const struct lism_combo_record *lism_combo_get(uint8_t slot) {
    if (slot >= LISM_COMBO_SLOTS || !slot_is_active(slot)) {
        return NULL;
    }
    return &slots[slot];
}

int lism_combo_active_count(void) {
    int count = 0;
    for (int i = 0; i < LISM_COMBO_SLOTS; i++) {
        if (slot_is_active(i)) {
            count++;
        }
    }
    return count;
}

/* ---- persistence + init ---- */

static int lism_combo_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    ARG_UNUSED(name);
    if (len != sizeof(slots)) {
        /* Slot table layout changed between firmwares — start fresh. */
        LOG_WRN("stored combo table size %u != %u, ignoring", (unsigned)len,
                (unsigned)sizeof(slots));
        return -EINVAL;
    }
    ssize_t ret = read_cb(cb_arg, slots, sizeof(slots));
    if (ret < 0) {
        return -EIO;
    }
    settings_found = true;
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(lism_combo, "lism_combo", NULL, lism_combo_settings_set, NULL,
                               NULL);

#if IS_ENABLED(CONFIG_LISM_COMBO_RPC_TEST_COMBO)

/* Behavior local IDs (settings-table type) are only final after the global
 * settings_load() in ZMK's main(), which runs after every SYS_INIT — so the
 * first-boot test combo is seeded from a delayed work item instead of init. */
static void seed_test_combo_work_handler(struct k_work *work) {
    if (settings_found || lism_combo_active_count() > 0) {
        return;
    }

    /* Device name for the &kp behavior varies by build; try the usual spellings. */
    static const char *const kp_names[] = {"key_press", "KEY_PRESS", "kp"};
    zmk_behavior_local_id_t kp_id = UINT16_MAX;
    for (int i = 0; i < ARRAY_SIZE(kp_names); i++) {
        kp_id = zmk_behavior_get_local_id(kp_names[i]);
        if (kp_id != UINT16_MAX) {
            break;
        }
    }
    if (kp_id == UINT16_MAX) {
        LOG_ERR("test combo: key_press behavior not found, skipping seed");
        return;
    }

    struct lism_combo_record rec = {
        .flags = LISM_COMBO_FLAG_ACTIVE,
        .key_count = 2,
        .key_positions = {0, 1, 0xFF, 0xFF}, /* Q + W */
        .timeout_ms = 50,
        .require_prior_idle_ms = 0,
        .layer_mask = 0,
        .behavior_local_id = kp_id,
        .param1 = 0x00070029, /* HID usage: Keyboard Escape */
        .param2 = 0,
    };
    /* Straight into the table without saving: it stays a pure default until
     * the app writes real state, and disappears if the option is disabled. */
    slots[0] = rec;
    LOG_INF("seeded first-boot test combo: Q+W -> Esc (behavior id %u)", kp_id);
}

static K_WORK_DELAYABLE_DEFINE(seed_test_combo_work, seed_test_combo_work_handler);

#endif /* CONFIG_LISM_COMBO_RPC_TEST_COMBO */

static int lism_combo_init(void) {
    for (int i = 0; i < CONFIG_LISM_COMBO_RPC_MAX_PRESSED; i++) {
        active_combos[i].slot = -1;
    }
    k_work_init_delayable(&timeout_task, combo_timeout_handler);

    settings_subsys_init();
    settings_load_subtree("lism_combo");

#if IS_ENABLED(CONFIG_LISM_COMBO_RPC_TEST_COMBO)
    k_work_schedule(&seed_test_combo_work, K_SECONDS(3));
#endif

    LOG_INF("lism-combo init: %d active combos (settings %s)", lism_combo_active_count(),
            settings_found ? "loaded" : "empty");
    return 0;
}

SYS_INIT(lism_combo_init, APPLICATION, 91);
