#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "svf_core.h"
#include "smoother.h"
#include "modulation.h"
#include "model_moog.h"

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

/* Moog gain probe at LOW amplitude so the tanh nonlinearity stays ~linear and we
   measure the filter's frequency response (slope/peak), not saturation. */
static double moog_gain_at(moog_t *m, double f_hz) {
    const int warm=8192, meas=8192; const double A=0.05;
    double w=2.0*PI*f_hz/FS, ph=0.0, sumsq=0.0;
    for (int i=0;i<warm+meas;i++){ double x=A*sin(ph); ph+=w; if(ph>2*PI)ph-=2*PI;
        double y=moog_process(m,x); if(i>=warm) sumsq+=y*y; }
    return sqrt(sumsq/meas)/(A*sqrt(0.5));
}
static double moog_gain_db(moog_t *m, double f){ return 20.0*log10(moog_gain_at(m,f)+1e-12); }

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

    /* ===== M2 modulation: envelope follower ===== */
    {
        envfollow_t e; envf_init(&e, FS); envf_set(&e, 5.0, 200.0);
        double env = 0;
        for (int i=0;i<FS/10;i++) env = envf_process(&e, 1.0);
        CHECK(env > 0.9, "envf rises to ~1 on full input (got %.3f)", env);
        CHECK(env <= 1.0001, "envf bounded <=1 (got %.3f)", env);
        for (int i=0;i<FS/2;i++) env = envf_process(&e, 0.0);
        CHECK(env < 0.1, "envf decays toward 0 on silence (got %.3f)", env);

        /* attack faster than release: samples for 0->0.5 rising vs 1->0.5 falling.
         * Must PRIME env to ~1 before measuring the fall (else it starts at 0.5
         * and crosses immediately). */
        envfollow_t e2; envf_init(&e2, FS); envf_set(&e2, 5.0, 200.0);
        int up=0; while (envf_process(&e2, 1.0) < 0.5 && up < (int)FS) up++;
        for (int i=0;i<FS/10;i++) envf_process(&e2, 1.0);   /* prime to ~1 */
        int dn=0; while (envf_process(&e2, 0.0) > 0.5 && dn < (int)FS) dn++;
        CHECK(up < dn, "envf attack (%d) faster than release (%d)", up, dn);
    }

    /* ===== M2 modulation: LFO ===== */
    {
        /* continuous shapes: bipolar and swing the full range over a cycle */
        int cont[] = { LFO_SINE, LFO_TRI, LFO_SAW, LFO_SQR };
        for (int si=0; si<4; si++) {
            int shp = cont[si];
            lfo_t l; lfo_init(&l, FS); lfo_set_rate_hz(&l, 2.0);
            double lo=2, hi=-2; int ok=1;
            for (int i=0;i<FS;i++){ double v=lfo_process(&l, shp);
                if (v<lo)lo=v; if (v>hi)hi=v; if (!isfinite(v)||v<-1.0001||v>1.0001) ok=0; }
            CHECK(ok, "lfo shape %d stays in [-1,1]", shp);
            CHECK(hi>0.5 && lo<-0.5, "lfo shape %d swings full (lo %.2f hi %.2f)", shp, lo, hi);
        }
        /* square is exactly +/-1 */
        { lfo_t l; lfo_init(&l, FS); lfo_set_rate_hz(&l, 1.0);
          int sq=1; for (int i=0;i<FS;i++){ double v=lfo_process(&l, LFO_SQR);
              if (fabs(fabs(v)-1.0) > 1e-9) sq=0; }
          CHECK(sq, "lfo square is exactly +/-1"); }
        /* 1 Hz period: sine returns near its start value after FS samples */
        { lfo_t l; lfo_init(&l, FS); lfo_set_rate_hz(&l, 1.0);
          double first = lfo_process(&l, LFO_SINE);
          for (int i=1;i<FS;i++) lfo_process(&l, LFO_SINE);
          double after = lfo_process(&l, LFO_SINE);
          CHECK(fabs(after-first) < 0.02, "lfo 1Hz cycle returns to start (%.3f vs %.3f)", after, first); }
        /* S&H: bipolar and varies across many cycles (random, stepwise) */
        { lfo_t l; lfo_init(&l, FS); lfo_set_rate_hz(&l, 200.0);
          double lo=2, hi=-2; int ok=1;
          for (int i=0;i<FS;i++){ double v=lfo_process(&l, LFO_SH);
              if (v<lo)lo=v; if (v>hi)hi=v; if (!isfinite(v)||v<-1.0001||v>1.0001) ok=0; }
          CHECK(ok, "lfo S&H stays in [-1,1]");
          CHECK(hi>0.3 && lo<-0.3, "lfo S&H varies over cycles (lo %.2f hi %.2f)", lo, hi); }
        /* S&H holds a constant value within one cycle */
        { lfo_t l; lfo_init(&l, FS); lfo_set_rate_hz(&l, 4.0); /* cycle = FS/4 */
          double a = lfo_process(&l, LFO_SH); int held = 1;
          for (int i=1;i<FS/8;i++){ double v=lfo_process(&l, LFO_SH);
              if (fabs(v-a) > 1e-12) held = 0; }
          CHECK(held, "lfo S&H holds value within a cycle"); }
    }

    /* ===== M3: Moog ladder model ===== */
    {
        moog_t m; moog_init(&m, FS);
        /* low res: flat passband, steep 4-pole slope */
        moog_set(&m, 1000.0, 0.0);
        CHECK(fabs(moog_gain_db(&m, 100.0)) < 2.0, "moog passband flat at low res (got %.2f)", moog_gain_db(&m,100.0));
        double slope = moog_gain_db(&m, 4000.0) - moog_gain_db(&m, 2000.0);
        CHECK(slope < -16.0, "moog steep (4-pole) slope (got %.1f dB/oct)", slope);

        /* strong resonance just below self-oscillation: resonant peak well above
         * the passband, AND the low end thins out (the Moog "honk" that
         * distinguishes it from the body-keeping SVF). Probe is valid here because
         * the filter isn't self-oscillating yet. */
        moog_set(&m, 1000.0, 0.7);
        double peak = moog_gain_db(&m, 1000.0);
        double pass = moog_gain_db(&m, 100.0);
        CHECK(peak - pass > 7.0, "moog resonance peaks above passband (+%.1f dB)", peak - pass);
        CHECK(pass < -4.0, "moog low end thins with resonance (got %.1f dB)", pass);

        /* max resonance self-oscillates: kick it, let input go silent, the tail
         * keeps ringing (a sustained limit cycle, bounded by the saturators). */
        moog_set(&m, 1000.0, 1.0);
        moog_process(&m, 1.0);
        for (int i=0;i<4000;i++) moog_process(&m, 0.0);
        double sq=0; for (int i=0;i<8192;i++){ double y=moog_process(&m,0.0); sq+=y*y; }
        double osc_rms = sqrt(sq/8192);
        CHECK(osc_rms > 0.01, "moog self-oscillates at max res (tail rms %.3f)", osc_rms);

        int stable=1;
        for (double fc=60; fc<16000; fc*=1.7)
          for (double r=0.0; r<=1.0; r+=0.2){ moog_set(&m, fc, r); double acc=0;
            for (int i=0;i<3000;i++) acc += moog_process(&m, (i%2?0.3:-0.3));
            if (!isfinite(acc)) stable=0; }
        CHECK(stable, "moog stable across fc/res grid");
    }

    return g_fail;
}
