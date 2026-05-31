#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "svf_core.h"
#include "smoother.h"

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

/* Max per-sample output jump when cutoff steps 200->8000 Hz mid-stream, probing
   with a steady 300 Hz tone. The tone's own per-sample slew is tiny (~0.02), so a
   large jump isolates the coefficient-step transient (zipper). 300 Hz also sits in
   the gain-transition region across the sweep, where a cutoff step is most audible.
   smoothed=0 steps cutoff instantly; smoothed=1 ramps it in log space. */
static double antizip_max_jump(int smoothed) {
    smoother_t sm; smooth_init(&sm, FS); smooth_set_tau(&sm, 0.018); smooth_reset(&sm, log(200.0));
    svf_t f; svf_init(&f, FS);
    double prev=0.0, mx=0.0, fc=200.0, ph=0.0, w=2.0*PI*300.0/FS;
    for (int i=0;i<8000;i++) {
        if (i==2000) { if (smoothed) smooth_target(&sm, log(8000.0)); else fc=8000.0; }
        double c = smoothed ? exp(smooth_next(&sm)) : fc;
        svf_set(&f, c, 0.7, SVF_LP);
        double x = sin(ph); ph += w; if (ph > 2*PI) ph -= 2*PI;
        double y = svf_process(&f, x);
        double j = fabs(y - prev); if (i>10 && j>mx) mx=j;
        prev = y;
    }
    return mx;
}

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

    svf_set(&s, 1000.0, 0.0, SVF_HP);
    CHECK(gain_db(&s, 50.0) < -20.0, "HP rejects sub-cutoff (got %.2f)", gain_db(&s,50.0));
    CHECK(fabs(gain_db(&s, 10000.0)) < 0.5, "HP passband flat high");

    /* BP raw band tap peaks at 1/k; res=0.5 -> k=1 -> unity (0 dB) at center.
       (res=0 -> k=2 -> -6 dB at center, since this SVF does not k-normalize the
       band output.) */
    svf_set(&s, 1000.0, 0.5, SVF_BP);
    CHECK(gain_db(&s, 50.0) < -15.0 && gain_db(&s, 20000.0) < -15.0, "BP rejects both ends");
    CHECK(fabs(gain_db(&s, 1000.0)) < 1.0, "BP ~unity at center (got %.2f)", gain_db(&s,1000.0));

    svf_set(&s, 1000.0, 0.5, SVF_NOTCH);
    CHECK(gain_db(&s, 1000.0) < -12.0, "Notch nulls at cutoff (got %.2f)", gain_db(&s,1000.0));

    svf_set(&s, 1000.0, 0.0, SVF_AP);
    CHECK(fabs(gain_db(&s, 500.0)) < 0.5 && fabs(gain_db(&s, 4000.0)) < 0.5, "AP flat magnitude");

    for (double fc=40; fc<18000; fc*=1.5)
      for (double r=0.0; r<=1.0; r+=0.1) {
        svf_set(&s, fc, r, SVF_LP); double acc=0;
        for (int i=0;i<2000;i++) acc += svf_process(&s, (i%2?1.0:-1.0));
        CHECK(isfinite(acc), "stable fc=%.0f res=%.1f", fc, r);
      }

    /* Anti-zipper: the smoothed cutoff step must not glitch, AND the test must
       discriminate — an unsmoothed (instant) step on the same signal MUST exceed
       the threshold, else the test would pass trivially and protect nothing. */
    double az_smooth = antizip_max_jump(1), az_step = antizip_max_jump(0);
    CHECK(az_smooth < 0.3, "smoothed cutoff step: jump %.3f < 0.3", az_smooth);
    CHECK(az_step > 0.6, "unsmoothed step zippers: jump %.3f > 0.6 (test discriminates)", az_step);

    return g_fail;
}
