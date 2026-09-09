#include "ring_buffer.h"

#include <string.h>

/* ===================================================================
 *  Internal helpers -- assume the caller already holds the lock.
 *
 *  Index arithmetic uses modulo wraparound: when head reaches capacity
 *  it wraps back to 0, so the array is walked in a circle and no data
 *  ever has to be shifted. That's the whole reason a ring buffer is
 *  O(1) per push instead of O(n).
 * =================================================================== */

static inline void rb_push_locked(ring_buffer_t *rb, const void *item)
{
    memcpy(&rb->storage[rb->head * rb->elem_size], item, rb->elem_size);
    rb->head = (rb->head + 1) % rb->diag.capacity;

    if (rb->diag.count == rb->diag.capacity) {
        /* Full: head just landed on top of the oldest entry, so tail
         * has to move forward too. count stays at capacity -- one entry
         * in, one entry silently gone. */
        rb->tail = (rb->tail + 1) % rb->diag.capacity;
        rb->diag.overwrites++;
    } else {
        rb->diag.count++;
        if (rb->diag.count > rb->diag.high_water) {
            rb->diag.high_water = rb->diag.count;
        }
    }
    rb->diag.pushed++;
}

static inline bool rb_pop_locked(ring_buffer_t *rb, void *out)
{
    if (rb->diag.count == 0) {
        return false;
    }
    memcpy(out, &rb->storage[rb->tail * rb->elem_size], rb->elem_size);
    rb->tail = (rb->tail + 1) % rb->diag.capacity;
    rb->diag.count--;
    rb->diag.popped++;
    return true;
}

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

bool ring_buffer_init(ring_buffer_t *rb, void *storage,
                      size_t elem_size, size_t capacity)
{
    if (!rb || !storage || elem_size == 0 || capacity == 0) {
        return false;
    }

    memset(rb, 0, sizeof(*rb));
    rb->storage   = (uint8_t *)storage;
    rb->elem_size = elem_size;
    rb->diag.capacity  = capacity;

    portMUX_TYPE tmp = portMUX_INITIALIZER_UNLOCKED;
    rb->lock = tmp;

    rb->initialized = true;
    return true;
}

void ring_buffer_reset(ring_buffer_t *rb)
{
    if (!rb || !rb->initialized) {
        return;
    }
    portENTER_CRITICAL(&rb->lock);
    rb->head  = 0;
    rb->tail  = 0;
    rb->diag.count = 0;
    portEXIT_CRITICAL(&rb->lock);
}

/* ===================================================================
 *  Producer / consumer -- task context
 * =================================================================== */

bool ring_buffer_push(ring_buffer_t *rb, const void *item)
{
    if (!rb || !rb->initialized || !item) {
        return false;
    }

    portENTER_CRITICAL(&rb->lock);
    bool overwrote = (rb->diag.count == rb->diag.capacity);
    rb_push_locked(rb, item);
    portEXIT_CRITICAL(&rb->lock);

    return overwrote;
}

bool ring_buffer_pop(ring_buffer_t *rb, void *out)
{
    if (!rb || !rb->initialized || !out) {
        return false;
    }

    portENTER_CRITICAL(&rb->lock);
    bool ok = rb_pop_locked(rb, out);
    portEXIT_CRITICAL(&rb->lock);

    return ok;
}

bool ring_buffer_peek(const ring_buffer_t *rb, void *out)
{
    if (!rb || !rb->initialized || !out) {
        return false;
    }

    /* Cast away const only to take the lock -- peek genuinely does not
     * modify the buffer's contents or indices. */
    ring_buffer_t *m = (ring_buffer_t *)rb;

    portENTER_CRITICAL(&m->lock);
    bool ok = (m->diag.count > 0);
    if (ok) {
        memcpy(out, &m->storage[m->tail * m->elem_size], m->elem_size);
    }
    portEXIT_CRITICAL(&m->lock);

    return ok;
}

size_t ring_buffer_pop_bulk(ring_buffer_t *rb, void *out, size_t max_items)
{
    if (!rb || !rb->initialized || !out || max_items == 0) {
        return 0;
    }

    uint8_t *dst = (uint8_t *)out;
    size_t   n   = 0;

    /* One critical section for the whole batch rather than one per
     * item: fewer lock acquisitions, and the batch comes out as a
     * consistent snapshot instead of interleaving with a producer. */
    portENTER_CRITICAL(&rb->lock);
    while (n < max_items && rb->diag.count > 0) {
        memcpy(&dst[n * rb->elem_size],
               &rb->storage[rb->tail * rb->elem_size],
               rb->elem_size);
        rb->tail = (rb->tail + 1) % rb->diag.capacity;
        rb->diag.count--;
        rb->diag.popped++;
        n++;
    }
    portEXIT_CRITICAL(&rb->lock);

    return n;
}

/* ===================================================================
 *  ISR variants
 * =================================================================== */

bool ring_buffer_push_from_isr(ring_buffer_t *rb, const void *item)
{
    if (!rb || !rb->initialized || !item) {
        return false;
    }

    portENTER_CRITICAL_ISR(&rb->lock);
    bool overwrote = (rb->diag.count == rb->diag.capacity);
    rb_push_locked(rb, item);
    portEXIT_CRITICAL_ISR(&rb->lock);

    return overwrote;
}

bool ring_buffer_pop_from_isr(ring_buffer_t *rb, void *out)
{
    if (!rb || !rb->initialized || !out) {
        return false;
    }

    portENTER_CRITICAL_ISR(&rb->lock);
    bool ok = rb_pop_locked(rb, out);
    portEXIT_CRITICAL_ISR(&rb->lock);

    return ok;
}

/* ===================================================================
 *  Queries
 * =================================================================== */

size_t ring_buffer_count(const ring_buffer_t *rb)
{
    if (!rb || !rb->initialized) {
        return 0;
    }
    ring_buffer_t *m = (ring_buffer_t *)rb;

    portENTER_CRITICAL(&m->lock);
    size_t n = m->diag.count;
    portEXIT_CRITICAL(&m->lock);

    return n;
}

size_t ring_buffer_free_space(const ring_buffer_t *rb)
{
    if (!rb || !rb->initialized) {
        return 0;
    }
    return rb->diag.capacity - ring_buffer_count(rb);
}

bool ring_buffer_is_empty(const ring_buffer_t *rb)
{
    return ring_buffer_count(rb) == 0;
}

bool ring_buffer_is_full(const ring_buffer_t *rb)
{
    if (!rb || !rb->initialized) {
        return false;
    }
    return ring_buffer_count(rb) == rb->diag.capacity;
}

void ring_buffer_get_diagnostics(const ring_buffer_t *rb, ring_buffer_diag_t *out)
{
    if (!rb || !out) {
        return;
    }
    ring_buffer_t *m = (ring_buffer_t *)rb;

    portENTER_CRITICAL(&m->lock);
    *out = m->diag;
    portEXIT_CRITICAL(&m->lock);
}
