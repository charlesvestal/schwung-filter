#include "model_moog.h"
#include <math.h>

#define MOOG_PI 3.14159265358979323846
#define MOOG_OS 2          /* oversample factor */
#define MOOG_FB_MAX 4.0    /* feedback at res=1 (>= 4 self-oscillates) */

void moog_init(moog_t *m, double fs) {
    m->fs = fs;
    m->g = 0.0;
    m->fb = 0.0;
    m->zfb = 0.0;
    m->s[0] = m->s[1] = m->s[2] = m->s[3] = 0.0;
    moog_set(m, 1000.0, 0.2);
}

void moog_set(moog_t *m, double fc, double res01) {
    double osfs = m->fs * MOOG_OS;
    if (fc < 20.0) fc = 20.0;
    double nyq = osfs * 0.5;
    if (fc > nyq * 0.49) fc = nyq * 0.49;
    m->g = tan(MOOG_PI * fc / osfs);   /* TPT one-pole coefficient at OS rate */
    if (res01 < 0.0) res01 = 0.0;
    if (res01 > 1.0) res01 = 1.0;
    m->fb = res01 * MOOG_FB_MAX;
}

/* one tick at the oversampled rate */
static inline double moog_tick(moog_t *m, double in) {
    /* Input + feedback through a saturator. Passband makeup ~(1 + 0.5*fb) keeps
     * the low end from collapsing as resonance (feedback) rises; tanh gives the
     * Moog warmth and bounds self-oscillation. */
    double x = in * (1.0 + 0.5 * m->fb) - m->fb * m->zfb;
    x = tanh(x);

    double G = m->g / (1.0 + m->g);
    double y = x;
    for (int i = 0; i < 4; i++) {
        double v = (y - m->s[i]) * G;
        double o = v + m->s[i];
        m->s[i] = o + v;
        y = o;
    }
    m->zfb = y;
    return y;
}

double moog_process(moog_t *m, double in) {
    /* ZOH upsample, run the nonlinear core at 2x, average to decimate
     * (a 2-tap FIR with a null at Nyquist — cheap anti-alias). */
    double acc = 0.0;
    for (int k = 0; k < MOOG_OS; k++) acc += moog_tick(m, in);
    return acc / (double)MOOG_OS;
}
