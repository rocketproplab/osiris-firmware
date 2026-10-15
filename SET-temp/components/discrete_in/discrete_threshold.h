#ifndef DISCRETE_THRESHOLD_H
#define DISCRETE_THRESHOLD_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 *  "Is the line fired?" from a stream of voltage samples.
 *
 *  Pure logic, no hardware, so it is tested on the host directly. Used
 *  by discrete_in in analog mode, where the Blue Raven line is read by
 *  the ADC because both its idle and fired levels (~2.86 V and 3.7 V
 *  through a 2k series resistor, measured 2026-10-10) sit above the
 *  digital HIGH level, which a digital input cannot tell apart.
 *
 *  Two thresholds, not one: the line goes FIRED at or above fire_mv and
 *  only comes back at or below release_mv. With a single threshold,
 *  noise on a line sitting near it would produce an edge per wobble.
 *
 *  And `confirm` consecutive samples must agree before either change is
 *  believed, so a single noisy sample cannot fire it. The edge time
 *  reported is that of the FIRST of those samples -- the moment the
 *  line actually crossed, not the moment we became sure of it.
 * =================================================================== */

typedef enum {
    DTHR_NONE = 0,
    DTHR_ROSE,      /* idle -> fired */
    DTHR_FELL,      /* fired -> idle */
} dthr_result_t;

typedef struct {
    uint32_t fire_mv;
    uint32_t release_mv;
    uint16_t confirm;
    bool     high;          /* currently FIRED */
    uint16_t run;           /* consecutive samples on the other side */
    uint64_t run_start_us;  /* when that run began */
} dthr_t;

void dthr_init(dthr_t *t, uint32_t fire_mv, uint32_t release_mv, uint16_t confirm);

/* Feed one sample (line voltage in mV, at the Blue Raven side). Returns
 * a change, with *edge_us = time of the first sample of the run that
 * confirmed it. */
dthr_result_t dthr_step(dthr_t *t, uint32_t line_mv, uint64_t now_us, uint64_t *edge_us);

#endif /* DISCRETE_THRESHOLD_H */
