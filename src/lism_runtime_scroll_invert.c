/*
 * LisM runtime-adjustable scroll-direction invert input processor.
 *
 * Adapted from zmkfirmware/zmk's own input_processor_scaler.c
 * (app/src/pointing/input_processor_scaler.c, v0.3.0, Copyright (c) 2024
 * The ZMK Contributors, MIT licensed): same event-type/codes matching
 * shape, but instead of a devicetree-fixed multiplier, this just flips
 * the sign of matched events when a runtime flag is set — live-settable
 * via trackball_rpc_apply_scroll_invert() (called from trackball_rpc.c
 * in response to a BLE write from the companion app).
 */

#define DT_DRV_COMPAT lism_input_processor_runtime_scroll_invert

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct lism_invert_config {
    uint8_t type;
    size_t codes_len;
    const uint16_t *codes;
};

static atomic_t lism_scroll_invert_flag = ATOMIC_INIT(0);

void trackball_rpc_apply_scroll_invert(bool invert) {
    atomic_set(&lism_scroll_invert_flag, invert ? 1 : 0);
}

static int lism_invert_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct lism_invert_config *cfg = dev->config;

    if (event->type != cfg->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    for (size_t i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == event->code) {
            if (atomic_get(&lism_scroll_invert_flag)) {
                event->value = -event->value;
            }
            break;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api lism_invert_driver_api = {
    .handle_event = lism_invert_handle_event,
};

#define LISM_INVERT_INST(n)                                                                       \
    static const uint16_t lism_invert_codes_##n[] = DT_INST_PROP(n, codes);                       \
    static const struct lism_invert_config lism_invert_config_##n = {                             \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                           \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                  \
        .codes = lism_invert_codes_##n,                                                           \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &lism_invert_config_##n, POST_KERNEL,               \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &lism_invert_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LISM_INVERT_INST)
