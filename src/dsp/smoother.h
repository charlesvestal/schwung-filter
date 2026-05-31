#ifndef SMOOTHER_H
#define SMOOTHER_H
typedef struct { double fs, coeff, cur, target; } smoother_t;
void   smooth_init(smoother_t *s, double fs);
void   smooth_set_tau(smoother_t *s, double tau_sec);
void   smooth_reset(smoother_t *s, double v);   /* jump cur+target to v */
void   smooth_target(smoother_t *s, double v);  /* set target only */
double smooth_next(smoother_t *s);              /* advance one sample, return cur */
#endif
