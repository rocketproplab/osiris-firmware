#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"

/* ===================================================================
 *  Fixed-size overwrite-oldest FIFO.
 *
 *  When the buffer is full, a push discards the OLDEST unread entry to
 *  make room for the newest. That's the right trade for flight data:
 *  if a consumer stalls, you'd rather keep the freshest samples and
 *  lose stale ones than block a sensor task or drop the newest reading.
 *
 *  Unlike the sensor drivers, this is a plain public struct rather than
 *  an opaque handle. A ring buffer is a data structure, not a hardware
 *  resource: you need several of them (SD, telemetry, flight logic),
 *  each with its own storage, and the caller has to be able to declare
 *  one statically. An opaque handle would force a fixed instance pool
 *  and buy nothing, since there's no hardware state to protect.
 *
 *  Storage is caller-provided -- no malloc anywhere, same as every
 *  other component here.
 * =================================================================== */

typedef struct {
    uint32_t pushed;
    uint32_t popped;
    uint32_t overwrites;   /* nonzero => a consumer is falling behind */
    size_t   count;
    size_t   capacity;
    size_t   high_water;   /* deepest the buffer ever got              */
} ring_buffer_diag_t;

typedef struct {
    uint8_t     *storage;      /* caller-provided backing array        */
    size_t       elem_size;    /* bytes per entry                      */

    size_t       head;         /* next write slot                      */
    size_t       tail;         /* next read slot                       */

    /* count/capacity/pushed/popped/overwrites/high_water all live in
     * here now instead of as separate flat fields -- this struct IS
     * the storage ring_buffer_get_diagnostics() hands back, the same
     * composition fix applied to bmp388_t/bmp388_diag_t earlier. count
     * and capacity aren't purely informational despite living in the
     * "diag" struct: rb_push_locked()/rb_pop_locked() read and write
     * them as part of normal push/pop logic, not just for reporting. */
    ring_buffer_diag_t diag;

    portMUX_TYPE lock;
    bool         initialized;
} ring_buffer_t;

/* Declare a ring buffer and its storage together, correctly sized.
 * Saves you from the classic bug of sizing the array in bytes when the
 * capacity is in entries (or vice versa).
 *
 *   RING_BUFFER_DECLARE(static, sd_rb, sd_baro_record_t, 128);
 *   ring_buffer_init(&sd_rb, sd_rb_storage, sizeof(sd_baro_record_t), 128);
 */
#define RING_BUFFER_DECLARE(qualifier, name, type, cap) \
    qualifier uint8_t name##_storage[(cap) * sizeof(type)]; \
    qualifier ring_buffer_t name

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

/* Binds a buffer to caller-provided storage. storage must be at least
 * elem_size * capacity bytes. Returns false on bad arguments. */
bool ring_buffer_init(ring_buffer_t *rb, void *storage,
                      size_t elem_size, size_t capacity);

/* Drops every entry. Diagnostic counters are preserved. */
void ring_buffer_reset(ring_buffer_t *rb);

/* ===================================================================
 *  Producer / consumer
 * =================================================================== */

/* Copies elem_size bytes in. Never fails for lack of space: if full,
 * the oldest entry is discarded and overwrites is incremented.
 * Returns true if an entry was overwritten, false if there was room. */
bool ring_buffer_push(ring_buffer_t *rb, const void *item);

/* Copies the oldest entry out and removes it. Returns false if empty. */
bool ring_buffer_pop(ring_buffer_t *rb, void *out);

/* Copies the oldest entry out WITHOUT removing it. Returns false if
 * empty. Useful when a consumer might fail and need to retry -- e.g.
 * peek, attempt the SD write, pop only once it succeeded. */
bool ring_buffer_peek(const ring_buffer_t *rb, void *out);

/* Pops up to max_items into a contiguous caller buffer. Returns how
 * many were actually copied. Batching like this is how you keep SD
 * writes on block boundaries instead of one fwrite per sample. */
size_t ring_buffer_pop_bulk(ring_buffer_t *rb, void *out, size_t max_items);

/* ===================================================================
 *  ISR variants
 *
 *  Same behavior, but taking the lock with the ISR-safe critical
 *  section. Use these and ONLY these from inside an interrupt handler;
 *  mixing the two families on one buffer is a bug.
 * =================================================================== */

bool ring_buffer_push_from_isr(ring_buffer_t *rb, const void *item);
bool ring_buffer_pop_from_isr(ring_buffer_t *rb, void *out);

/* ===================================================================
 *  Queries
 * =================================================================== */

size_t ring_buffer_count(const ring_buffer_t *rb);
size_t ring_buffer_free_space(const ring_buffer_t *rb);
bool   ring_buffer_is_empty(const ring_buffer_t *rb);
bool   ring_buffer_is_full(const ring_buffer_t *rb);

void ring_buffer_get_diagnostics(const ring_buffer_t *rb, ring_buffer_diag_t *out);

#endif /* RING_BUFFER_H */
