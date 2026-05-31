/*
 * modulation.h — modulation sources for the filter: an envelope follower
 * (auto-wah) and an LFO. Both are pure per-sample DSP blocks with no host
 * dependency, so they are unit-tested in the native measurement harness.
 * filter.c advances them and sums their output into the normalized cutoff.
 */
#ifndef MODULATION_H
#define MODULATION_H

#include <stdint.h>

/* ---- Envelope follower ---- */
typedef struct {
    double fs;
    double atk_coef;   /* one-pole coefficient when level rises */
    double rel_coef;   /* one-pole coefficient when level falls */
    double env;        /* current envelope, ~0..1 */
} envfollow_t;

void   envf_init(envfollow_t *e, double fs);
void   envf_set(envfollow_t *e, double atk_ms, double rel_ms);
double envf_process(envfollow_t *e, double x);  /* rectify + one-pole; returns env */

/* ---- LFO ---- */
typedef enum {
    LFO_SINE = 0,
    LFO_TRI,
    LFO_SAW,
    LFO_SQR,
    LFO_SH      /* sample & hold (random, updates once per cycle) */
} lfo_shape_t;

typedef struct {
    double   fs;
    double   phase;    /* 0..1 */
    double   inc;      /* phase increment per sample (= hz/fs) */
    uint32_t rng;      /* xorshift state for S&H (deterministic) */
    double   sh_val;   /* current sample&hold value, bipolar */
} lfo_t;

void   lfo_init(lfo_t *l, double fs);
void   lfo_set_rate_hz(lfo_t *l, double hz);
double lfo_process(lfo_t *l, int shape);        /* advances phase; returns bipolar [-1,1] */

#endif /* MODULATION_H */
