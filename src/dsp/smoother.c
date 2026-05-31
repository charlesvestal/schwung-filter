#include "smoother.h"
#include <math.h>
void smooth_init(smoother_t *s, double fs){ s->fs=fs; s->cur=s->target=0; smooth_set_tau(s,0.015); }
void smooth_set_tau(smoother_t *s, double tau){ s->coeff = 1.0 - exp(-1.0/(tau*s->fs)); }
void smooth_reset(smoother_t *s, double v){ s->cur=s->target=v; }
void smooth_target(smoother_t *s, double v){ s->target=v; }
double smooth_next(smoother_t *s){ s->cur += (s->target - s->cur)*s->coeff; return s->cur; }
