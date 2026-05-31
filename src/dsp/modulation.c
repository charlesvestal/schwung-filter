#include "modulation.h"
#include <math.h>

#define MOD_PI 3.14159265358979323846

/* ---- Envelope follower ---- */

void envf_init(envfollow_t *e, double fs) {
    e->fs = fs;
    e->env = 0.0;
    envf_set(e, 10.0, 150.0);
}

void envf_set(envfollow_t *e, double atk_ms, double rel_ms) {
    if (atk_ms < 0.05) atk_ms = 0.05;
    if (rel_ms < 0.05) rel_ms = 0.05;
    e->atk_coef = 1.0 - exp(-1.0 / (atk_ms * 0.001 * e->fs));
    e->rel_coef = 1.0 - exp(-1.0 / (rel_ms * 0.001 * e->fs));
}

double envf_process(envfollow_t *e, double x) {
    double a = fabs(x);
    double c = (a > e->env) ? e->atk_coef : e->rel_coef;
    e->env += c * (a - e->env);
    return e->env;
}

/* ---- LFO ---- */

void lfo_init(lfo_t *l, double fs) {
    l->fs = fs;
    l->phase = 0.0;
    l->inc = 0.0;
    l->rng = 0x9E3779B9u;   /* fixed seed — deterministic, no Math.random/Date */
    l->sh_val = 0.0;
    lfo_set_rate_hz(l, 1.0);
}

void lfo_set_rate_hz(lfo_t *l, double hz) {
    if (hz < 0.0) hz = 0.0;
    l->inc = hz / l->fs;
}

/* xorshift32 — deterministic PRNG for sample & hold */
static uint32_t xorshift32(uint32_t *s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

static double lfo_shape_val(const lfo_t *l, int shape) {
    double p = l->phase;   /* 0..1 */
    switch (shape) {
        case LFO_SINE: return sin(2.0 * MOD_PI * p);
        case LFO_TRI:  return 4.0 * fabs(p - 0.5) - 1.0;   /* +1 -> -1 -> +1 */
        case LFO_SAW:  return 2.0 * p - 1.0;               /* -1 -> +1 ramp */
        case LFO_SQR:  return (p < 0.5) ? 1.0 : -1.0;
        case LFO_SH:   return l->sh_val;
        default:       return sin(2.0 * MOD_PI * p);
    }
}

double lfo_process(lfo_t *l, int shape) {
    double v = lfo_shape_val(l, shape);
    l->phase += l->inc;
    if (l->phase >= 1.0) {
        l->phase -= 1.0;
        /* new random hold value at each cycle boundary, bipolar [-1,1] */
        l->sh_val = ((double)xorshift32(&l->rng) / 4294967295.0) * 2.0 - 1.0;
    }
    return v;
}
