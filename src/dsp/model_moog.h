/*
 * model_moog.h — Moog-style transistor ladder filter (clean-room).
 * 4-pole / 24 dB-per-octave lowpass with tanh nonlinearity in the
 * input+feedback path (the warm, saturating Moog character) and resonance up to
 * self-oscillation. Internally 2x oversampled to tame aliasing from the
 * nonlinearity. Cascade of four TPT one-pole lowpasses (consistent with the SVF
 * core) wrapped in a saturated feedback loop.
 */
#ifndef MODEL_MOOG_H
#define MODEL_MOOG_H

typedef struct {
    double fs;       /* base sample rate */
    double g;        /* TPT one-pole coefficient at the oversampled rate */
    double fb;       /* feedback amount (resonance), 0..~4 (self-osc near 4) */
    double s[4];     /* one-pole integrator states */
    double zfb;      /* one-sample-delayed feedback (at oversampled rate) */
} moog_t;

void   moog_init(moog_t *m, double fs);
void   moog_set(moog_t *m, double cutoff_hz, double res01);  /* res 0..1 */
double moog_process(moog_t *m, double in);                   /* 2x oversampled */

#endif /* MODEL_MOOG_H */
