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
    (void)svf_gain_at;
    (void)gain_db;
    printf("harness up\n");
    return g_fail;
}
