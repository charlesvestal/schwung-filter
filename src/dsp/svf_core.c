#include "svf_core.h"
#include <math.h>
#define SVF_PI 3.14159265358979323846

void svf_init(svf_t *s, double fs) {
    s->fs = fs; s->ic1eq = s->ic2eq = 0.0; s->mode = SVF_LP;
    svf_set(s, 1000.0, 0.0, SVF_LP);
}

void svf_set(svf_t *s, double fc, double res, svf_mode_t mode) {
    if (fc < 20.0) fc = 20.0;
    double nyq = s->fs * 0.5;
    if (fc > nyq * 0.99) fc = nyq * 0.99;
    if (res < 0.0) res = 0.0; if (res > 1.0) res = 1.0;
    s->g = tan(SVF_PI * fc / s->fs);
    /* res 0..1 -> k 2..0 (Q 0.5..inf). Floor k to stay stable at self-osc. */
    s->k = 2.0 - 2.0 * res; if (s->k < 0.0001) s->k = 0.0001;
    s->a1 = 1.0 / (1.0 + s->g * (s->g + s->k));
    s->a2 = s->g * s->a1;
    s->a3 = s->g * s->a2;
    s->mode = mode;
}

double svf_process(svf_t *s, double v0) {
    double v3 = v0 - s->ic2eq;
    double v1 = s->a1 * s->ic1eq + s->a2 * v3;
    double v2 = s->ic2eq + s->a2 * s->ic1eq + s->a3 * v3;
    s->ic1eq = 2.0 * v1 - s->ic1eq;
    s->ic2eq = 2.0 * v2 - s->ic2eq;
    double low = v2, band = v1, high = v0 - s->k * v1 - v2;
    switch (s->mode) {
        case SVF_LP:    return low;
        case SVF_HP:    return high;
        case SVF_BP:    return band;
        case SVF_NOTCH: return low + high;            /* v0 - k*band */
        case SVF_PEAK:  return low - high;
        case SVF_AP:    return v0 - 2.0 * s->k * band;
        default:        return low;
    }
}
