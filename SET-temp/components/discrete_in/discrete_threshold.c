#include "discrete_threshold.h"

void dthr_init(dthr_t *t, uint32_t fire_mv, uint32_t release_mv, uint16_t confirm)
{
    t->fire_mv      = fire_mv;
    /* A release above the fire threshold would make the two overlap and
     * the line could never settle; clamp rather than misbehave. */
    t->release_mv   = release_mv <= fire_mv ? release_mv : fire_mv;
    t->confirm      = confirm ? confirm : 1;
    t->high         = false;
    t->run          = 0;
    t->run_start_us = 0;
}

dthr_result_t dthr_step(dthr_t *t, uint32_t line_mv, uint64_t now_us, uint64_t *edge_us)
{
    bool crossing = t->high ? (line_mv <= t->release_mv)
                            : (line_mv >= t->fire_mv);
    if (!crossing) {
        t->run = 0;                 /* the run is broken: start over */
        return DTHR_NONE;
    }
    if (t->run == 0) {
        t->run_start_us = now_us;
    }
    if (++t->run < t->confirm) {
        return DTHR_NONE;
    }
    t->run  = 0;
    t->high = !t->high;
    if (edge_us) {
        *edge_us = t->run_start_us;
    }
    return t->high ? DTHR_ROSE : DTHR_FELL;
}
