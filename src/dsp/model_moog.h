/*
 * model_moog.h — Moog transistor-ladder filter.
 * C port of Stefano D'Angelo's "ImprovedMoog" (the D'Angelo-Valimaki physically-
 * derived nonlinear ladder) from ddiakopoulos/MoogLadders, ISC-licensed — see
 * the copyright header in model_moog.c. Four tanh-saturated stages with thermal
 * voltage VT, trapezoidal integration; 2x oversampled here for HF coefficient
 * validity and reduced aliasing.
 */
#ifndef MODEL_MOOG_H
#define MODEL_MOOG_H

typedef struct {
    double fs;                 /* base sample rate */
    double V[4], dV[4], tV[4]; /* per-stage integrator/derivative/tanh state */
    double g;                  /* tuning coefficient (at oversampled rate) */
    double drive;              /* input drive into the saturators */
    double resonance;          /* feedback amount (mapped from res 0..1) */
} moog_t;

void   moog_init(moog_t *m, double fs);
void   moog_set(moog_t *m, double cutoff_hz, double res01);  /* res 0..1 */
double moog_process(moog_t *m, double in);                   /* 2x oversampled */

#endif /* MODEL_MOOG_H */
