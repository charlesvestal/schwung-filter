#ifndef SVF_CORE_H
#define SVF_CORE_H
typedef enum { SVF_LP=0, SVF_HP, SVF_BP, SVF_NOTCH, SVF_PEAK, SVF_AP } svf_mode_t;
typedef struct {
    double fs;
    double g, k, a1, a2, a3;   /* coefficients */
    double ic1eq, ic2eq;       /* integrator state */
    svf_mode_t mode;
} svf_t;
void   svf_init(svf_t *s, double fs);
void   svf_set(svf_t *s, double cutoff_hz, double res, svf_mode_t mode); /* res 0..1 */
double svf_process(svf_t *s, double v0);
#endif
