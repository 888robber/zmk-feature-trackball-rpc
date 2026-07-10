/*
 * LisM trackball swipe-gesture detector input processor.
 *
 * Meant to sit behind a momentary "gesture layer" (activated by holding a
 * 2-key combo — see the combos {} block in lism.keymap): while active,
 * accumulates raw X/Y trackball motion and, once the dominant axis
 * crosses a threshold, invokes one of 4 configured behaviors
 * (left/right/up/down — typically &kp bindings sending macOS Mission
 * Control shortcuts like LC(LEFT)/LC(RIGHT)/LC(UP)/LC(DOWN)) via ZMK's
 * own zmk_behavior_invoke_binding(), the same primitive ZMK's own
 * input_processor_behaviors.c uses (app/src/pointing/
 * input_processor_behaviors.c, v0.3.0, Copyright (c) 2024 The ZMK
 * Contributors, MIT licensed) to invoke a behavior from a custom input
 * processor. After firing, resets the accumulator and enters a cooldown
 * window so one big swipe doesn't fire repeatedly.
 *
 * Raw motion is swallowed (ZMK_INPUT_PROC_STOP) rather than passed
 * through, so the cursor doesn't also jump around while gesturing.
 */

#define DT_DRV_COMPAT lism_input_processor_gesture

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct lism_gesture_config {
    uint8_t index;
    int32_t threshold;
    int32_t cooldown_ms;
    const struct zmk_behavior_binding *bindings; /* [left, right, up, down] */
};

struct lism_gesture_data {
    int32_t accum_x;
    int32_t accum_y;
    int64_t cooldown_until_ms;
};

static void lism_gesture_fire(const struct zmk_behavior_binding *binding,
                                struct zmk_input_processor_state *state) {
    struct zmk_behavior_binding_event behavior_event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(state->input_device_index, 0),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    zmk_behavior_invoke_binding(binding, behavior_event, true);
    zmk_behavior_invoke_binding(binding, behavior_event, false);
}

static int lism_gesture_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    const struct lism_gesture_config *cfg = dev->config;
    struct lism_gesture_data *data = dev->data;

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now = k_uptime_get();
    if (now < data->cooldown_until_ms) {
        return ZMK_INPUT_PROC_STOP;
    }

    if (event->code == INPUT_REL_X) {
        data->accum_x += event->value;
    } else {
        data->accum_y += event->value;
    }

    int32_t abs_x = data->accum_x < 0 ? -data->accum_x : data->accum_x;
    int32_t abs_y = data->accum_y < 0 ? -data->accum_y : data->accum_y;

    const struct zmk_behavior_binding *fire = NULL;
    if (abs_x >= cfg->threshold && abs_x >= abs_y) {
        fire = data->accum_x > 0 ? &cfg->bindings[1] /* right */ : &cfg->bindings[0] /* left */;
    } else if (abs_y >= cfg->threshold) {
        fire = data->accum_y > 0 ? &cfg->bindings[3] /* down */ : &cfg->bindings[2] /* up */;
    }

    if (fire) {
        LOG_DBG("LisM gesture fired: dx=%d dy=%d", data->accum_x, data->accum_y);
        lism_gesture_fire(fire, state);
        data->accum_x = 0;
        data->accum_y = 0;
        data->cooldown_until_ms = now + cfg->cooldown_ms;
    }

    return ZMK_INPUT_PROC_STOP;
}

static struct zmk_input_processor_driver_api lism_gesture_driver_api = {
    .handle_event = lism_gesture_handle_event,
};

#define LISM_GESTURE_INST(n)                                                                       \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == 4,                                               \
                 "lism,input-processor-gesture needs exactly 4 bindings: left, right, up, down");   \
    static const struct zmk_behavior_binding lism_gesture_bindings_##n[] = {                        \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))};  \
    static const struct lism_gesture_config lism_gesture_config_##n = {                             \
        .index = n,                                                                                \
        .threshold = DT_INST_PROP_OR(n, threshold, 300),                                            \
        .cooldown_ms = DT_INST_PROP_OR(n, cooldown_ms, 300),                                        \
        .bindings = lism_gesture_bindings_##n,                                                      \
    };                                                                                              \
    static struct lism_gesture_data lism_gesture_data_##n;                                          \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &lism_gesture_data_##n, &lism_gesture_config_##n,           \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                          \
                          &lism_gesture_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LISM_GESTURE_INST)
