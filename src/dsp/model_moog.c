/*
 * model_moog.c — Moog transistor-ladder filter.
 *
 * C port of Stefano D'Angelo's "ImprovedMoog" (the D'Angelo-Valimaki physically-
 * derived nonlinear ladder) from ddiakopoulos/MoogLadders. The original is
 * ISC-licensed; its copyright/permission notice is reproduced below verbatim as
 * the license requires.
 *
 * ---------------------------------------------------------------------------
 * Copyright 2012 Stefano D'Angelo <zanga.mail@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * ---------------------------------------------------------------------------
 *
 * Port notes (schwung-filter): C++ -> C single-instance struct; 2x oversampled
 * (the original is single-rate — oversampling keeps the tuning coefficient g
 * positive/valid up to 20 kHz and halves tanh aliasing); res 0..1 mapped to the
 * feedback term, calibrated so res=1 just reaches self-oscillation.
 */
#include "model_moog.h"
#include <math.h>
#include <string.h>

#define MOOG_PI 3.14159265358979323846
#define VT 0.312          /* thermal voltage (D'Angelo) */
#define MOOG_OS 2          /* oversample factor */
#define MOOG_RES_MAX 4.0   /* analog self-osc threshold; compensated per cutoff */

void moog_init(moog_t *m, double fs) {
    m->fs = fs;
    memset(m->V,  0, sizeof(m->V));
    memset(m->dV, 0, sizeof(m->dV));
    memset(m->tV, 0, sizeof(m->tV));
    m->drive = 1.0;
    m->g = 0.0;
    m->resonance = 0.1;
    moog_set(m, 1000.0, 0.1);
}

void moog_set(moog_t *m, double fc, double res01) {
    double osfs = m->fs * MOOG_OS;
    if (fc < 20.0) fc = 20.0;
    if (fc > osfs * 0.45) fc = osfs * 0.45;
    double x = (MOOG_PI * fc) / osfs;
    double warp = (1.0 - x) / (1.0 + x);   /* bilinear warp factor */
    m->g = 4.0 * MOOG_PI * VT * fc * warp;
    if (res01 < 0.0) res01 = 0.0;
    if (res01 > 1.0) res01 = 1.0;
    /* Compensate feedback for the discretization loop-gain excess at high cutoff.
     * Without this, the digital model self-oscillates at progressively lower
     * resonance as cutoff rises (54% at 10 kHz vs ~100% at 300 Hz). The onset
     * tracks warp^~1 but applying raw warp over-corrects at low frequencies.
     * Empirical fit: scale by warp, but compute the max feedback from the onset
     * calibration at the reference frequency (300 Hz, onset=4.0). At each cutoff
     * the target onset is ~4.0/warp_ratio where warp_ratio = warp(fc)/warp(ref).
     * Simplification: just divide by warp to normalize, capped so low-freq
     * doesn't exceed the analog limit. */
    double fb_comp = MOOG_RES_MAX;
    if (warp > 0.01) {
        double warp_ref = 0.977;  /* warp at ~300 Hz reference */
        fb_comp = MOOG_RES_MAX * (warp / warp_ref);
        if (fb_comp > MOOG_RES_MAX) fb_comp = MOOG_RES_MAX;
        if (fb_comp < 2.2) fb_comp = 2.2;  /* floor: keep self-osc reachable even at 10 kHz+ */
    }
    m->resonance = res01 * fb_comp;
}

/* one tick at the oversampled rate (D'Angelo's trapezoidal stage updates) */
static inline double moog_tick(moog_t *m, double in) {
    double two_osfs = 2.0 * m->fs * MOOG_OS;

    double dV0 = -m->g * (tanh((m->drive * in + m->resonance * m->V[3]) / (2.0 * VT)) + m->tV[0]);
    m->V[0] += (dV0 + m->dV[0]) / two_osfs;
    m->dV[0] = dV0;
    m->tV[0] = tanh(m->V[0] / (2.0 * VT));

    double dV1 = m->g * (m->tV[0] - m->tV[1]);
    m->V[1] += (dV1 + m->dV[1]) / two_osfs;
    m->dV[1] = dV1;
    m->tV[1] = tanh(m->V[1] / (2.0 * VT));

    double dV2 = m->g * (m->tV[1] - m->tV[2]);
    m->V[2] += (dV2 + m->dV[2]) / two_osfs;
    m->dV[2] = dV2;
    m->tV[2] = tanh(m->V[2] / (2.0 * VT));

    double dV3 = m->g * (m->tV[2] - m->tV[3]);
    m->V[3] += (dV3 + m->dV[3]) / two_osfs;
    m->dV[3] = dV3;
    m->tV[3] = tanh(m->V[3] / (2.0 * VT));

    return m->V[3];
}

double moog_process(moog_t *m, double in) {
    /* ZOH upsample, run the nonlinear core at 2x, average to decimate. */
    double acc = 0.0;
    for (int k = 0; k < MOOG_OS; k++) acc += moog_tick(m, in);
    return acc / (double)MOOG_OS;
}
