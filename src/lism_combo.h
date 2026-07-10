/*
 * LisM runtime-editable combos — shared definitions between the matcher
 * engine (lism_runtime_combo.c) and the BLE RPC service (lism_combo_rpc.c).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/toolchain.h>

#define LISM_COMBO_MAX_KEYS 4
#define LISM_COMBO_SLOTS CONFIG_LISM_COMBO_RPC_MAX_COMBOS

#define LISM_COMBO_FLAG_ACTIVE BIT(0)
#define LISM_COMBO_FLAG_SLOW_RELEASE BIT(1)

/* One combo definition. This exact layout is used in three places and they
 * must stay in sync by construction: the in-RAM slot table, the persisted
 * settings blob, and the BLE wire format (21 bytes, little-endian). */
struct lism_combo_record {
    uint8_t flags;
    uint8_t key_count;
    uint8_t key_positions[LISM_COMBO_MAX_KEYS]; /* unused entries = 0xFF */
    uint16_t timeout_ms;
    uint16_t require_prior_idle_ms;
    uint8_t layer_mask; /* bit per layer index; 0 = active on all layers */
    uint16_t behavior_local_id;
    uint32_t param1;
    uint32_t param2;
} __packed;

#define LISM_COMBO_RECORD_WIRE_LEN 21
BUILD_ASSERT(sizeof(struct lism_combo_record) == LISM_COMBO_RECORD_WIRE_LEN,
             "lism_combo_record layout must match the BLE wire format");

/* All of these must only be called from the system workqueue (the same
 * context the matcher itself runs in); the BLE service defers to it. */
int lism_combo_set(uint8_t slot, const struct lism_combo_record *rec);
int lism_combo_delete(uint8_t slot);
void lism_combo_clear_all(void);
/* Returns NULL for an out-of-range or inactive slot. */
const struct lism_combo_record *lism_combo_get(uint8_t slot);
int lism_combo_active_count(void);
