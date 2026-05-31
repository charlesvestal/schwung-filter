#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "svf_core.h"

#define FS 44100.0
#define PI 3.14159265358979323846

static int g_fail = 0;
#define CHECK(cond, ...) do { if(!(cond)){ \
    fprintf(stderr, "FAIL: " __VA_ARGS__); fprintf(stderr,"\n"); g_fail=1; } \
    else { printf("ok: " __VA_ARGS__); printf("\n"); } } while(0)

/* Steady-state linear gain of one SVF at frequency f_hz. */
static double svf_gain_at(svf_t *s, double f_hz) {
    const int warm = 8192, meas = 8192;
    double w = 2.0*PI*f_hz/FS, ph = 0.0, sumsq = 0.0;
    for (int i = 0; i < warm+meas; i++) {
        double x = sin(ph); ph += w; if (ph > 2*PI) ph -= 2*PI;
        double y = svf_process(s, x);
        if (i >= warm) sumsq += y*y;
    }
    double rms = sqrt(sumsq/meas);
    return rms / sqrt(0.5);   /* normalize: input sine RMS = 1/sqrt(2) */
}

static double gain_db(svf_t *s, double f) { return 20.0*log10(svf_gain_at(s,f)+1e-12); }

int main(void) {
    printf("harness up\n");

    svf_t s; svf_init(&s, FS);
    /* res 0.2929 -> k=sqrt(2) -> Butterworth (Q=0.707), the -3dB-at-cutoff point.
       res=0 would be Q=0.5 (critically damped, -6dB at cutoff). */
    svf_set(&s, /*cutoff_hz*/1000.0, /*res 0..1*/0.2929, SVF_LP);
    CHECK(fabs(gain_db(&s, 1000.0) - (-3.0)) < 1.0, "LP -3dB at cutoff (got %.2f)", gain_db(&s,1000.0));
    CHECK(fabs(gain_db(&s, 100.0)) < 0.5,  "LP passband flat at 100Hz (got %.2f)", gain_db(&s,100.0));
    double slope = gain_db(&s, 4000.0) - gain_db(&s, 2000.0);
    CHECK(fabs(slope - (-12.0)) < 2.0, "LP slope ~-12dB/oct (got %.2f)", slope);

    return g_fail;
}
