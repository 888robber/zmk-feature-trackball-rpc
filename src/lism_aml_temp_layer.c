/*
 * LisM automatic mouse layer (AML) input processor.
 *
 * Adapted from zmkfirmware/zmk's own input_processor_temp_layer.c
 * (app/src/pointing/input_processor_temp_layer.c, v0.3.0, Copyright (c)
 * 2024 The ZMK Contributors, MIT licensed) — same layer-activation state
 * machine, but the "how long to wait since the last motion event before
 * auto-deactivating" value (upstream: the param2 devicetree cell, fixed
 * at build time) is read from a runtime-mutable global instead, settable
 * live via trackball_rpc_apply_aml_idle_ms() (called from trackball_rpc.c
 * in response to a BLE write from the companion app).
 *
 * param1 (which layer to activate) is still devicetree-fixed, since that
 * wasn't asked to be runtime-adjustable.
 */

#define DT_DRV_COMPAT lism_input_processor_temp_layer

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define MAX_LAYERS ZMK_KEYMAP_LAYERS_LEN

/* Runtime-adjustable idle threshold, live-settable via BLE. Defaults to
 * the value trackball_rpc.c starts with (see its `state.aml_idle_ms`);
 * kept in sync since trackball_rpc_apply_aml_idle_ms() is called once at
 * boot with the persisted/default value too. */
static atomic_t lism_aml_idle_ms = ATOMIC_INIT(1000);

void trackball_rpc_apply_aml_idle_ms(uint16_t idle_ms) {
    atomic_set(&lism_aml_idle_ms, idle_ms);
}

struct lism_aml_config {
    int16_t require_prior_idle_ms;
    const uint16_t *excluded_positions;
    size_t num_positions;
};

struct lism_aml_state {
    uint8_t toggle_layer;
    bool is_active;
    int64_t last_tapped_timestamp;
};

struct lism_aml_data {
    const struct device *dev;
    struct k_mutex lock;
    struct lism_aml_state state;
};

static struct k_work_delayable lism_aml_layer_disable_works[MAX_LAYERS];

static bool lism_aml_position_is_excluded(const struct lism_aml_config *config,
                                            uint32_t position) {
    if (!config->excluded_positions || !config->num_positions) {
        return false;
    }

    const uint16_t *end = config->excluded_positions + config->num_positions;
    for (const uint16_t *pos = config->excluded_positions; pos < end; pos++) {
        if (*pos == position) {
            return true;
        }
    }

    return false;
}

static bool lism_aml_should_quick_tap(const struct lism_aml_config *config, int64_t last_tapped,
                                        int64_t current_time) {
    return (last_tapped + config->require_prior_idle_ms) > current_time;
}

static void lism_aml_update_layer_state(struct lism_aml_state *state, bool activate) {
    if (state->is_active == activate) {
        return;
    }

    state->is_active = activate;
    if (activate) {
        zmk_keymap_layer_activate(state->toggle_layer);
        LOG_DBG("LisM AML: layer %d activated", state->toggle_layer);
    } else {
        zmk_keymap_layer_deactivate(state->toggle_layer);
        LOG_DBG("LisM AML: layer %d deactivated", state->toggle_layer);
    }
}

struct lism_aml_layer_action {
    uint8_t layer;
    bool activate;
};

K_MSGQ_DEFINE(lism_aml_action_msgq, sizeof(struct lism_aml_layer_action),
              CONFIG_TRACKBALL_RPC_AML_MAX_ACTION_EVENTS, 4);

static void lism_aml_action_work_cb(struct k_work *work) {
    const struct device *dev = DEVICE_DT_INST_GET(0);
    struct lism_aml_data *data = (struct lism_aml_data *)dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        LOG_ERR("LisM AML: error locking for updating %d", ret);
        return;
    }

    struct lism_aml_layer_action action;

    while (k_msgq_get(&lism_aml_action_msgq, &action, K_MSEC(10)) >= 0) {
        if (!action.activate) {
            if (zmk_keymap_layer_active(action.layer)) {
                lism_aml_update_layer_state(&data->state, false);
            }
        } else {
            lism_aml_update_layer_state(&data->state, true);
        }
    }

    k_mutex_unlock(&data->lock);
}

static K_WORK_DEFINE(lism_aml_action_work, lism_aml_action_work_cb);

static void lism_aml_layer_disable_callback(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    int layer_index = ARRAY_INDEX(lism_aml_layer_disable_works, d_work);

    struct lism_aml_layer_action action = {.layer = layer_index, .activate = false};

    k_msgq_put(&lism_aml_action_msgq, &action, K_MSEC(10));
    k_work_submit(&lism_aml_action_work);
}

static int lism_aml_handle_layer_state_changed(const struct device *dev, const zmk_event_t *eh) {
    struct lism_aml_data *data = (struct lism_aml_data *)dev->data;
    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }
    if (!zmk_keymap_layer_active(zmk_keymap_layer_index_to_id(data->state.toggle_layer))) {
        LOG_DBG("LisM AML: deactivating layer that was activated by this processor");
        data->state.is_active = false;
        k_work_cancel_delayable(&lism_aml_layer_disable_works[data->state.toggle_layer]);
    }
    ret = k_mutex_unlock(&data->lock);
    if (ret < 0) {
        return ret;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

static int lism_aml_handle_position_state_changed(const struct device *dev,
                                                     const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct lism_aml_data *data = (struct lism_aml_data *)dev->data;
    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    const struct lism_aml_config *cfg = dev->config;

    if (data->state.is_active && cfg->excluded_positions && cfg->num_positions > 0) {
        if (!lism_aml_position_is_excluded(cfg, ev->position)) {
            lism_aml_update_layer_state(&data->state, false);
        }
    }

    k_mutex_unlock(&data->lock);

    return ZMK_EV_EVENT_BUBBLE;
}

static int lism_aml_handle_keycode_state_changed(const struct device *dev,
                                                    const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct lism_aml_data *data = (struct lism_aml_data *)dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    data->state.last_tapped_timestamp = ev->timestamp;

    ret = k_mutex_unlock(&data->lock);
    if (ret < 0) {
        return ret;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

static int lism_aml_handle_state_changed_dispatcher(const struct device *dev,
                                                       const zmk_event_t *eh) {
    if (as_zmk_layer_state_changed(eh) != NULL) {
        return lism_aml_handle_layer_state_changed(dev, eh);
    } else if (as_zmk_position_state_changed(eh) != NULL) {
        return lism_aml_handle_position_state_changed(dev, eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        return lism_aml_handle_keycode_state_changed(dev, eh);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

#define LISM_AML_DISPATCH_EVENT(inst)                                                             \
    {                                                                                             \
        int err = lism_aml_handle_state_changed_dispatcher(DEVICE_DT_INST_GET(inst), eh);         \
        if (err < 0) {                                                                            \
            return err;                                                                           \
        }                                                                                          \
    }

static int lism_aml_handle_event_dispatcher(const zmk_event_t *eh) {
    DT_INST_FOREACH_STATUS_OKAY(LISM_AML_DISPATCH_EVENT)

    return 0;
}

static int lism_aml_handle_event(const struct device *dev, struct input_event *event,
                                    uint32_t param1, uint32_t param2,
                                    struct zmk_input_processor_state *state) {
    ARG_UNUSED(param2); /* idle threshold comes from lism_aml_idle_ms instead, see above */

    if (param1 >= MAX_LAYERS) {
        LOG_ERR("LisM AML: invalid layer index: %d", param1);
        return -EINVAL;
    }

    struct lism_aml_data *data = (struct lism_aml_data *)dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    const struct lism_aml_config *cfg = dev->config;

    data->state.toggle_layer = param1;

    if (!data->state.is_active &&
        !lism_aml_should_quick_tap(cfg, data->state.last_tapped_timestamp, k_uptime_get())) {
        struct lism_aml_layer_action action = {.layer = param1, .activate = true};

        k_msgq_put(&lism_aml_action_msgq, &action, K_MSEC(10));
        k_work_submit(&lism_aml_action_work);
    }

    uint32_t idle_ms = atomic_get(&lism_aml_idle_ms);
    if (idle_ms > 0) {
        k_work_reschedule(&lism_aml_layer_disable_works[param1], K_MSEC(idle_ms));
    }

    k_mutex_unlock(&data->lock);

    return ZMK_INPUT_PROC_CONTINUE;
}

static int lism_aml_init(const struct device *dev) {
    struct lism_aml_data *data = (struct lism_aml_data *)dev->data;
    k_mutex_init(&data->lock);

    for (int i = 0; i < MAX_LAYERS; i++) {
        k_work_init_delayable(&lism_aml_layer_disable_works[i], lism_aml_layer_disable_callback);
    }

    return 0;
}

static const struct zmk_input_processor_driver_api lism_aml_driver_api = {
    .handle_event = lism_aml_handle_event,
};

#define LISM_AML_NEEDS_POSITION_HANDLERS(n, ...) DT_INST_PROP_HAS_IDX(n, excluded_positions, 0)
#define LISM_AML_NEEDS_KEYCODE_HANDLERS(n, ...)                                                   \
    (DT_INST_PROP_OR(n, require_prior_idle_ms, 0) > 0)

ZMK_LISTENER(lism_aml_processor, lism_aml_handle_event_dispatcher);
ZMK_SUBSCRIPTION(lism_aml_processor, zmk_layer_state_changed);

#if DT_INST_FOREACH_STATUS_OKAY_VARGS(LISM_AML_NEEDS_POSITION_HANDLERS, ||)
ZMK_SUBSCRIPTION(lism_aml_processor, zmk_position_state_changed);
#endif

#if DT_INST_FOREACH_STATUS_OKAY_VARGS(LISM_AML_NEEDS_KEYCODE_HANDLERS, ||)
ZMK_SUBSCRIPTION(lism_aml_processor, zmk_keycode_state_changed);
#endif

#define LISM_AML_INST(n)                                                                          \
    static struct lism_aml_data lism_aml_data_##n = {};                                           \
    static const uint16_t lism_aml_excluded_positions_##n[] = DT_INST_PROP(n, excluded_positions); \
    static const struct lism_aml_config lism_aml_config_##n = {                                   \
        .require_prior_idle_ms = DT_INST_PROP_OR(n, require_prior_idle_ms, 0),                    \
        .excluded_positions = lism_aml_excluded_positions_##n,                                    \
        .num_positions = DT_INST_PROP_LEN(n, excluded_positions),                                 \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, lism_aml_init, NULL, &lism_aml_data_##n, &lism_aml_config_##n,        \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &lism_aml_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LISM_AML_INST)
