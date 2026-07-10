/*
 * LisM cursor acceleration curve input processor.
 *
 * Unlike cursor sensitivity (a flat CPI multiplier applied at the sensor
 * itself, see trackball_rpc.c's apply_cursor_sensitivity()), this adds a
 * SPEED-DEPENDENT extra multiplier on top: move the trackball fast and
 * cursor jumps get proportionally bigger, similar to typical mouse
 * "pointer acceleration" settings. Speed is estimated per-axis from the
 * event's delta value and the time since the last motion event (ZMK
 * reports X and Y as separate input_event calls, so there's no single
 * true 2D speed available here — this is an approximation, tuned by feel
 * on real hardware rather than derived analytically).
 *
 * All four parameters (enabled, max multiplier, kick-in speed, ramp
 * width) are runtime-mutable, live-settable via the trackball RPC
 * service (trackball_rpc_apply_accel(), called from trackball_rpc.c).
 */

#define DT_DRV_COMPAT lism_input_processor_accel

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static atomic_t accel_enabled = ATOMIC_INIT(0);
static atomic_t accel_max_x100 = ATOMIC_INIT(140);  /* 1.40x default */
static atomic_t accel_kick_in = ATOMIC_INIT(400);   /* counts/sec */
static atomic_t accel_ramp = ATOMIC_INIT(800);      /* counts/sec width */

static int64_t lism_accel_last_event_ms;

void trackball_rpc_apply_accel(bool enabled, uint16_t max_x100, uint16_t kick_in_speed,
                                 uint16_t ramp_width) {
    atomic_set(&accel_enabled, enabled ? 1 : 0);
    atomic_set(&accel_max_x100, max_x100);
    atomic_set(&accel_kick_in, kick_in_speed);
    atomic_set(&accel_ramp, ramp_width > 0 ? ramp_width : 1);
}

static int lism_accel_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    ARG_UNUSED(dev);
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    if (!atomic_get(&accel_enabled)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now = k_uptime_get();
    int64_t dt = now - lism_accel_last_event_ms;
    lism_accel_last_event_ms = now;
    if (dt <= 0) {
        dt = 1;
    }
    dt = MIN(dt, 1000); /* clamp so a long idle gap doesn't read as "slow" */

    int32_t magnitude = event->value < 0 ? -event->value : event->value;
    int32_t speed = (magnitude * 1000) / (int32_t)dt; /* counts/sec */

    int32_t kick_in = atomic_get(&accel_kick_in);
    int32_t ramp = atomic_get(&accel_ramp);
    int32_t max_x100 = atomic_get(&accel_max_x100);

    int32_t mult_x100 = 100;
    if (speed > kick_in) {
        int32_t over = speed - kick_in;
        int32_t t_x100 = MIN((over * 100) / ramp, 100);
        mult_x100 = 100 + (t_x100 * (max_x100 - 100)) / 100;
    }

    event->value = (int16_t)((event->value * mult_x100) / 100);

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api lism_accel_driver_api = {
    .handle_event = lism_accel_handle_event,
};

#define LISM_ACCEL_INST(n)                                                                        \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &lism_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LISM_ACCEL_INST)
