/*
 * FILTER Audio FX Plugin - TPT State-Variable Filter (M1)
 *
 * Stereo state-variable filter (two svf_t sharing the same coefficients) with
 * per-sample parameter smoothing, dry/wet mix, and output makeup gain.
 *
 * Models:
 *   SVF   - TPT state-variable filter (only model in M1)
 *
 * Modes (svf_mode_t):
 *   LP, HP, BP, Notch, Peak, AP
 *
 * Parameters:
 *   model      - SVF (only one in M1)
 *   mode       - LP / HP / BP / Notch / Peak / AP
 *   cutoff     - cutoff frequency in Hz (20 to 20000), smoothed in LOG space
 *   resonance  - 0 to 1 (k=2..0, Q 0.5..inf, self-osc near 1)
 *   drive      - 0 to 1 (M1: smoothed but NOT applied; nonlinear models in M3 use it)
 *   mix        - dry/wet 0 to 1
 *   output     - makeup gain in dB (-24 to 12), smoothed in LINEAR domain
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>   /* strcasecmp (POSIX) — used for tolerant enum parsing */
#include <math.h>
#include <stdint.h>

#include "audio_fx_api_v1.h"
#include "svf_core.h"
#include "smoother.h"
#include "modulation.h"
#include "model_moog.h"

#define SAMPLE_RATE 44100.0

/* Filter models */
enum {
    MODEL_SVF = 0,
    MODEL_MOOG
};

static const char *MODEL_NAMES[] = { "SVF", "Schwoog" };
#define MODEL_COUNT 2
static int model_from_string(const char *s) {
    for (int i = 0; i < MODEL_COUNT; i++) if (strcasecmp(s, MODEL_NAMES[i]) == 0) return i;
    if (strcasecmp(s, "Moog") == 0) return MODEL_MOOG;   /* backward-compat alias */
    return MODEL_SVF;
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float db_to_linear(float db) {
    return powf(10.0f, db / 20.0f);
}

/* Cutoff is a normalized 0..1 control mapped LOGARITHMICALLY to 20..20000 Hz
 * (constant ratio per unit -> each knob detent is a fixed fraction of an octave,
 * so the sweep feels even and covers the whole range in a reasonable turn).
 * 20 * 1000^norm: norm 0 -> 20 Hz, norm 1 -> 20000 Hz. Smoothing the *normalized*
 * value linearly is identical to smoothing log-Hz, so the anti-zipper smoother
 * works unchanged. */
#define CUTOFF_LOG_SPAN 6.9077553f   /* logf(20000/20) = logf(1000) */
static inline float cutoff_norm_to_hz(float norm) {
    return 20.0f * expf(norm * CUTOFF_LOG_SPAN);
}

/* Resonance taper: user 0..1 -> effective res for svf_set.
 * svf_core maps res linearly to damping (k = 2 - 2*res), so raw Q = 1/(2-2res)
 * crams nearly all the high-Q action into the top few % and runs away to a loud
 * pure self-oscillator at the very top. Instead, rise Q *exponentially* (even
 * perceptual sweep) from Q~0.5 to Q~RES_Q_MAX and CAP below runaway, so max is
 * "screaming but stable". To restore true self-oscillation, raise RES_Q_MAX (or
 * remove the cap). Derivation: Q = 0.5 * base^r with base = 2*RES_Q_MAX, so
 * k = 1/Q = 2*base^-r and res_eff = 1 - k/2 = 1 - base^-r. */
#define RES_Q_MAX 18.0f
static inline float resonance_taper(float r) {
    if (r <= 0.0f) return 0.0f;
    if (r > 1.0f) r = 1.0f;
    return 1.0f - powf(2.0f * RES_Q_MAX, -r);
}

/* Soft output limiter: transparent (identity) for |x| <= THR so normal-level,
 * undriven signals stay clean, then a smooth tanh knee saturates peaks toward
 * LIM instead of hard-clipping at full scale. Prevents the harsh digital clip a
 * high-resonance peak would otherwise produce. */
static inline float soft_limit(float x) {
    const float thr = 0.70f;   /* linear below this */
    const float lim = 0.98f;   /* output ceiling */
    float a = fabsf(x);
    if (a <= thr) return x;
    float range = lim - thr;
    float s = (x < 0.0f) ? -1.0f : 1.0f;
    return s * (thr + range * tanhf((a - thr) / range));
}

/* ---- JSON helper ---- */
static int json_get_number(const char *json, const char *key, float *out) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (!p) return -1;
    p += strlen(pattern);
    while (*p && (*p == ' ' || *p == ':' || *p == '\t')) p++;
    if (!*p) return -1;
    *out = strtof(p, NULL);
    return 0;
}

static int json_get_string(const char *json, const char *key, char *out, int out_len) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (!p) return -1;
    p += strlen(pattern);
    while (*p && *p != '"') p++;
    if (*p != '"') return -1;
    p++;
    const char *end = strchr(p, '"');
    if (!end) return -1;
    int len = (int)(end - p);
    if (len >= out_len) len = out_len - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return 0;
}

/* ---- Mode <-> string helpers ---- */
static int mode_from_string(const char *s, int *out) {
    /* tolerant: accept any case */
    if (!s) return -1;
    if (strcasecmp(s, "LP") == 0)         { *out = SVF_LP;    return 0; }
    if (strcasecmp(s, "HP") == 0)         { *out = SVF_HP;    return 0; }
    if (strcasecmp(s, "BP") == 0)         { *out = SVF_BP;    return 0; }
    if (strcasecmp(s, "NOTCH") == 0)      { *out = SVF_NOTCH; return 0; }
    if (strcasecmp(s, "PEAK") == 0)       { *out = SVF_PEAK;  return 0; }
    if (strcasecmp(s, "AP") == 0)         { *out = SVF_AP;    return 0; }
    return -1;
}

static const char* mode_to_string(int mode) {
    switch (mode) {
        case SVF_LP:    return "LP";
        case SVF_HP:    return "HP";
        case SVF_BP:    return "BP";
        case SVF_NOTCH: return "Notch";
        case SVF_PEAK:  return "Peak";
        case SVF_AP:    return "AP";
        default:        return "LP";
    }
}

/* ---- LFO shape <-> string ---- */
static const char *LFO_SHAPE_NAMES[] = { "Sine", "Tri", "Saw", "Sqr", "S&H" };
static int lfo_shape_from_string(const char *s) {
    for (int i = 0; i < 5; i++) if (strcasecmp(s, LFO_SHAPE_NAMES[i]) == 0) return i;
    return LFO_SINE;
}

/* ---- LFO tempo divisions: beats (quarter notes) per LFO cycle ---- */
static const char  *LFO_DIV_NAMES[] = { "1/1","1/2","1/4","1/4.","1/8","1/8.","1/8T","1/16","1/16T","1/32" };
static const double LFO_DIV_BEATS[] = { 4.0, 2.0, 1.0, 1.5, 0.5, 0.75, 1.0/3.0, 0.25, 1.0/6.0, 0.125 };
#define LFO_DIV_COUNT 10
static int lfo_div_from_string(const char *s) {
    for (int i = 0; i < LFO_DIV_COUNT; i++) if (strcasecmp(s, LFO_DIV_NAMES[i]) == 0) return i;
    return 4; /* 1/8 */
}

/* ---- Instance state ---- */
typedef struct {
    char module_dir[256];

    /* targets from set_param */
    int   model;            /* MODEL_SVF=0 (only one in M1) */
    int   mode;             /* svf_mode_t */
    float cutoff;           /* 0 to 1, log-mapped to 20..20000 Hz */
    float resonance;        /* 0 to 1 */
    float drive;            /* 0 to 1 */
    float mix;              /* 0 to 1 */
    float output_db;        /* -24 to 12 */

    /* modulation params */
    float env_amount;       /* -1..+1 bipolar cutoff mod from envelope follower */
    float env_attack_ms;    /* 1..500 */
    float env_release_ms;   /* 1..500 */
    float lfo_amount;       /* 0..1 cutoff mod depth */
    int   lfo_shape;        /* lfo_shape_t */
    int   lfo_sync;         /* 0 = Free (Hz), 1 = Sync (division) */
    float lfo_rate_hz;      /* 0.01..20, used when Free */
    int   lfo_rate_div;     /* index into LFO_DIV_*, used when Sync */

    /* dsp */
    svf_t fL, fR;
    moog_t moogL, moogR;
    smoother_t sm_cut;      /* normalized 0..1 (== log-Hz, see cutoff_norm_to_hz) */
    smoother_t sm_res;
    smoother_t sm_drive;
    smoother_t sm_mix;
    smoother_t sm_outlin;   /* linear makeup gain */
    smoother_t sm_env_amt;  /* modulation depths smoothed to avoid zipper */
    smoother_t sm_lfo_amt;
    envfollow_t env;
    lfo_t lfo;
} filter_instance_t;

/* ---- Globals ---- */
static const host_api_v1_t *g_host = NULL;

static void filter_log(const char *msg) {
    if (g_host && g_host->log) {
        char buf[256];
        snprintf(buf, sizeof(buf), "[FILTER] %s", msg);
        g_host->log(buf);
    }
}

/* ---- Instance lifecycle ---- */

static void* filter_create_instance(const char *module_dir, const char *config_json) {
    (void)config_json;
    filter_log("Creating instance");

    filter_instance_t *inst = (filter_instance_t*)calloc(1, sizeof(filter_instance_t));
    if (!inst) {
        filter_log("Failed to allocate instance");
        return NULL;
    }

    if (module_dir) {
        strncpy(inst->module_dir, module_dir, sizeof(inst->module_dir) - 1);
    }

    /* Defaults */
    inst->model      = MODEL_SVF;
    inst->mode       = SVF_LP;
    inst->cutoff     = 0.5f;     /* ~632 Hz — clearly audible default */
    inst->resonance  = 0.2f;
    inst->drive      = 0.0f;
    inst->mix        = 1.0f;
    inst->output_db  = 0.0f;

    inst->env_amount     = 0.0f;
    inst->env_attack_ms  = 10.0f;
    inst->env_release_ms = 150.0f;
    inst->lfo_amount     = 0.0f;
    inst->lfo_shape      = LFO_SINE;
    inst->lfo_sync       = 1;        /* default Sync */
    inst->lfo_rate_hz    = 1.0f;
    inst->lfo_rate_div   = 4;        /* 1/8 */

    /* DSP init */
    svf_init(&inst->fL, SAMPLE_RATE);
    svf_init(&inst->fR, SAMPLE_RATE);
    moog_init(&inst->moogL, SAMPLE_RATE);
    moog_init(&inst->moogR, SAMPLE_RATE);
    envf_init(&inst->env, SAMPLE_RATE);
    envf_set(&inst->env, inst->env_attack_ms, inst->env_release_ms);
    lfo_init(&inst->lfo, SAMPLE_RATE);

    /* Smoothers */
    smooth_init(&inst->sm_cut, SAMPLE_RATE);
    smooth_set_tau(&inst->sm_cut, 0.018);
    smooth_reset(&inst->sm_cut, 0.5f);   /* normalized cutoff */

    smooth_init(&inst->sm_res, SAMPLE_RATE);
    smooth_set_tau(&inst->sm_res, 0.020);
    smooth_reset(&inst->sm_res, 0.2f);

    smooth_init(&inst->sm_drive, SAMPLE_RATE);
    smooth_set_tau(&inst->sm_drive, 0.012);
    smooth_reset(&inst->sm_drive, 0.0f);

    smooth_init(&inst->sm_mix, SAMPLE_RATE);
    smooth_set_tau(&inst->sm_mix, 0.012);
    smooth_reset(&inst->sm_mix, 1.0f);

    smooth_init(&inst->sm_outlin, SAMPLE_RATE);
    smooth_set_tau(&inst->sm_outlin, 0.012);
    smooth_reset(&inst->sm_outlin, 1.0f);  /* linear of 0 dB */

    smooth_init(&inst->sm_env_amt, SAMPLE_RATE);
    smooth_set_tau(&inst->sm_env_amt, 0.015);
    smooth_reset(&inst->sm_env_amt, 0.0f);

    smooth_init(&inst->sm_lfo_amt, SAMPLE_RATE);
    smooth_set_tau(&inst->sm_lfo_amt, 0.015);
    smooth_reset(&inst->sm_lfo_amt, 0.0f);

    filter_log("Instance created");
    return inst;
}

static void filter_destroy_instance(void *instance) {
    filter_instance_t *inst = (filter_instance_t*)instance;
    if (!inst) return;
    filter_log("Destroying instance");
    free(inst);
}

/* ---- Audio processing (in-place, per-sample) ---- */

static void filter_process_block(void *instance, int16_t *audio_inout, int frames) {
    filter_instance_t *inst = (filter_instance_t*)instance;
    if (!inst) return;

    /* Denormal safety: when the input goes silent the SVF integrator state
     * decays through the denormal range, which is a real RT-performance hazard
     * on ARM. Today that is flushed by build.sh's -Ofast (-ffast-math sets the
     * FPSCR FTZ/DAZ bit). If that flag is ever dropped, OR by M3's oversampled
     * nonlinear models (sustained tiny feedback residue), add an explicit flush
     * to the per-sample state. Keep -ffast-math until then. */
    /* LFO rate is computed once per block. Sync derives Hz from host tempo
     * (get_bpm, NULL-guarded) and the chosen note division; Free uses lfo_rate_hz. */
    double bpm = (g_host && g_host->get_bpm) ? (double)g_host->get_bpm() : 120.0;
    if (bpm < 20.0) bpm = 120.0;
    double lfo_hz = inst->lfo_sync
        ? (bpm / 60.0) / LFO_DIV_BEATS[inst->lfo_rate_div]   /* cycles/sec */
        : (double)inst->lfo_rate_hz;
    lfo_set_rate_hz(&inst->lfo, lfo_hz);

    for (int i = 0; i < frames; i++) {
        float base_cut = (float)smooth_next(&inst->sm_cut);               /* normalized 0..1 */
        float res_raw = (float)smooth_next(&inst->sm_res);                /* user resonance 0..1 */
        float res = resonance_taper(res_raw);                            /* SVF: even Q, capped */
        float drv = (float)smooth_next(&inst->sm_drive);
        float mix = (float)smooth_next(&inst->sm_mix);
        float og  = (float)smooth_next(&inst->sm_outlin);
        float env_amt = (float)smooth_next(&inst->sm_env_amt);
        float lfo_amt = (float)smooth_next(&inst->sm_lfo_amt);

        float xl = audio_inout[i * 2]     / 32768.0f;
        float xr = audio_inout[i * 2 + 1] / 32768.0f;

        /* Modulation: the envelope follower tracks the pre-drive input level; the
         * LFO free-runs/syncs. Both sum into the NORMALIZED cutoff (so depth is
         * octave-proportional and respects the manual cutoff as the center), then
         * map to Hz. env in [0,1], lfo bipolar [-1,1]. */
        float envv = (float)envf_process(&inst->env, 0.5f * (xl + xr));
        float lfov = (float)lfo_process(&inst->lfo, inst->lfo_shape);
        float cut_norm = clampf(base_cut + env_amt * envv + lfo_amt * lfov, 0.0f, 1.0f);
        float cut = cutoff_norm_to_hz(cut_norm);

        /* Pre-filter soft saturation. Input gain into tanh, normalized so a
         * full-scale input stays ~unity; higher drive pushes more signal into
         * the nonlinear region (adds harmonics, lifts quiet detail) and feeds
         * the filter — classic for resonant/acid sweeps. (Analog models add their
         * own intrinsic nonlinearity on top of this.) */
        if (drv > 0.0005f) {
            float k = 1.0f + drv * 7.0f;
            float inv = 1.0f / tanhf(k);
            xl = tanhf(xl * k) * inv;
            xr = tanhf(xr * k) * inv;
        }

        /* Model dispatch. Moog is a nonlinear 4-pole ladder (LP only; `mode` does
         * not apply) that self-makeups internally. SVF is the clean multimode with
         * resonance makeup on the wet path (peak gain ~ Q would otherwise clip). */
        float wl, wr;
        if (inst->model == MODEL_MOOG) {
            moog_set(&inst->moogL, cut, res_raw);
            moog_set(&inst->moogR, cut, res_raw);
            wl = (float)moog_process(&inst->moogL, xl);
            wr = (float)moog_process(&inst->moogR, xr);
            /* Fatten: gentle saturation on the ladder output adds harmonics to the
             * resonance / self-oscillation so it reads as a thick growl rather than
             * a pure-sine whistle. Near-transparent at low level. */
            wl = tanhf(wl * 1.6f);
            wr = tanhf(wr * 1.6f);
        } else {
            /* svf_set called twice/sample (L+R) — acceptable; a later pass can
             * compute coefficients once and copy to the second filter. */
            svf_set(&inst->fL, cut, res, (svf_mode_t)inst->mode);
            svf_set(&inst->fR, cut, res, (svf_mode_t)inst->mode);
            float kk = 2.0f - 2.0f * res;
            float g_res = powf(kk * 0.5f, 0.25f);   /* 1.0 at low res .. ~0.41 at max */
            wl = (float)svf_process(&inst->fL, xl) * g_res;
            wr = (float)svf_process(&inst->fR, xr) * g_res;
        }

        float yl = (xl * (1.0f - mix) + wl * mix) * og;
        float yr = (xr * (1.0f - mix) + wr * mix) * og;

        audio_inout[i * 2]     = (int16_t)(soft_limit(yl) * 32767.0f);
        audio_inout[i * 2 + 1] = (int16_t)(soft_limit(yr) * 32767.0f);
    }
}

/* ---- Parameter handling ---- */

static void filter_set_param(void *instance, const char *key, const char *val) {
    filter_instance_t *inst = (filter_instance_t*)instance;
    if (!inst) return;

    float v = atof(val);

    if (strcmp(key, "cutoff") == 0) {
        inst->cutoff = clampf(v, 0.0f, 1.0f);
        smooth_target(&inst->sm_cut, inst->cutoff);
    } else if (strcmp(key, "resonance") == 0) {
        inst->resonance = clampf(v, 0.0f, 1.0f);
        smooth_target(&inst->sm_res, inst->resonance);
    } else if (strcmp(key, "drive") == 0) {
        inst->drive = clampf(v, 0.0f, 1.0f);
        smooth_target(&inst->sm_drive, inst->drive);
    } else if (strcmp(key, "mix") == 0) {
        inst->mix = clampf(v, 0.0f, 1.0f);
        smooth_target(&inst->sm_mix, inst->mix);
    } else if (strcmp(key, "output") == 0) {
        inst->output_db = clampf(v, -24.0f, 12.0f);
        smooth_target(&inst->sm_outlin, db_to_linear(inst->output_db));
    } else if (strcmp(key, "mode") == 0) {
        int m;
        if (mode_from_string(val, &m) == 0) inst->mode = m;
    } else if (strcmp(key, "model") == 0) {
        inst->model = model_from_string(val);
    } else if (strcmp(key, "env_amount") == 0) {
        inst->env_amount = clampf(v, -1.0f, 1.0f);
        smooth_target(&inst->sm_env_amt, inst->env_amount);
    } else if (strcmp(key, "env_attack") == 0) {
        inst->env_attack_ms = clampf(v, 1.0f, 500.0f);
        envf_set(&inst->env, inst->env_attack_ms, inst->env_release_ms);
    } else if (strcmp(key, "env_release") == 0) {
        inst->env_release_ms = clampf(v, 1.0f, 500.0f);
        envf_set(&inst->env, inst->env_attack_ms, inst->env_release_ms);
    } else if (strcmp(key, "lfo_amount") == 0) {
        inst->lfo_amount = clampf(v, 0.0f, 1.0f);
        smooth_target(&inst->sm_lfo_amt, inst->lfo_amount);
    } else if (strcmp(key, "lfo_shape") == 0) {
        inst->lfo_shape = lfo_shape_from_string(val);
    } else if (strcmp(key, "lfo_sync") == 0) {
        inst->lfo_sync = (strcasecmp(val, "Sync") == 0) ? 1 : 0;
    } else if (strcmp(key, "lfo_rate_hz") == 0) {
        inst->lfo_rate_hz = clampf(v, 0.01f, 20.0f);
    } else if (strcmp(key, "lfo_rate_div") == 0) {
        inst->lfo_rate_div = lfo_div_from_string(val);
    } else if (strcmp(key, "state") == 0) {
        /* Restore all parameters from JSON state.
         * Use smooth_reset (instant) so a freshly-loaded patch is immediately
         * at the right value with no audible ramp from the default. */
        float fval;
        char sval[32];

        if (json_get_string(val, "model", sval, sizeof(sval)) == 0)
            inst->model = model_from_string(sval);
        if (json_get_string(val, "mode", sval, sizeof(sval)) == 0) {
            int m;
            if (mode_from_string(sval, &m) == 0) inst->mode = m;
        }
        if (json_get_number(val, "cutoff", &fval) == 0) {
            inst->cutoff = clampf(fval, 0.0f, 1.0f);
            smooth_reset(&inst->sm_cut, inst->cutoff);
        }
        if (json_get_number(val, "resonance", &fval) == 0) {
            inst->resonance = clampf(fval, 0.0f, 1.0f);
            smooth_reset(&inst->sm_res, inst->resonance);
        }
        if (json_get_number(val, "drive", &fval) == 0) {
            inst->drive = clampf(fval, 0.0f, 1.0f);
            smooth_reset(&inst->sm_drive, inst->drive);
        }
        if (json_get_number(val, "mix", &fval) == 0) {
            inst->mix = clampf(fval, 0.0f, 1.0f);
            smooth_reset(&inst->sm_mix, inst->mix);
        }
        if (json_get_number(val, "output", &fval) == 0) {
            inst->output_db = clampf(fval, -24.0f, 12.0f);
            smooth_reset(&inst->sm_outlin, db_to_linear(inst->output_db));
        }
        if (json_get_number(val, "env_amount", &fval) == 0) {
            inst->env_amount = clampf(fval, -1.0f, 1.0f);
            smooth_reset(&inst->sm_env_amt, inst->env_amount);
        }
        if (json_get_number(val, "env_attack", &fval) == 0)
            inst->env_attack_ms = clampf(fval, 1.0f, 500.0f);
        if (json_get_number(val, "env_release", &fval) == 0)
            inst->env_release_ms = clampf(fval, 1.0f, 500.0f);
        envf_set(&inst->env, inst->env_attack_ms, inst->env_release_ms);
        if (json_get_number(val, "lfo_amount", &fval) == 0) {
            inst->lfo_amount = clampf(fval, 0.0f, 1.0f);
            smooth_reset(&inst->sm_lfo_amt, inst->lfo_amount);
        }
        if (json_get_string(val, "lfo_shape", sval, sizeof(sval)) == 0)
            inst->lfo_shape = lfo_shape_from_string(sval);
        if (json_get_string(val, "lfo_sync", sval, sizeof(sval)) == 0)
            inst->lfo_sync = (strcasecmp(sval, "Sync") == 0) ? 1 : 0;
        if (json_get_number(val, "lfo_rate_hz", &fval) == 0)
            inst->lfo_rate_hz = clampf(fval, 0.01f, 20.0f);
        if (json_get_string(val, "lfo_rate_div", sval, sizeof(sval)) == 0)
            inst->lfo_rate_div = lfo_div_from_string(sval);
    }
}

static int filter_get_param(void *instance, const char *key, char *buf, int buf_len) {
    filter_instance_t *inst = (filter_instance_t*)instance;
    if (!inst) return -1;

    if (strcmp(key, "cutoff") == 0)
        return snprintf(buf, buf_len, "%.3f", inst->cutoff);
    if (strcmp(key, "resonance") == 0)
        return snprintf(buf, buf_len, "%.2f", inst->resonance);
    if (strcmp(key, "drive") == 0)
        return snprintf(buf, buf_len, "%.2f", inst->drive);
    if (strcmp(key, "mix") == 0)
        return snprintf(buf, buf_len, "%.2f", inst->mix);
    if (strcmp(key, "output") == 0)
        return snprintf(buf, buf_len, "%.1f", inst->output_db);
    if (strcmp(key, "mode") == 0)
        return snprintf(buf, buf_len, "%s", mode_to_string(inst->mode));
    if (strcmp(key, "model") == 0)
        return snprintf(buf, buf_len, "%s", MODEL_NAMES[inst->model]);
    if (strcmp(key, "env_amount") == 0)
        return snprintf(buf, buf_len, "%.2f", inst->env_amount);
    if (strcmp(key, "env_attack") == 0)
        return snprintf(buf, buf_len, "%.0f", inst->env_attack_ms);
    if (strcmp(key, "env_release") == 0)
        return snprintf(buf, buf_len, "%.0f", inst->env_release_ms);
    if (strcmp(key, "lfo_amount") == 0)
        return snprintf(buf, buf_len, "%.2f", inst->lfo_amount);
    if (strcmp(key, "lfo_shape") == 0)
        return snprintf(buf, buf_len, "%s", LFO_SHAPE_NAMES[inst->lfo_shape]);
    if (strcmp(key, "lfo_sync") == 0)
        return snprintf(buf, buf_len, "%s", inst->lfo_sync ? "Sync" : "Free");
    if (strcmp(key, "lfo_rate_hz") == 0)
        return snprintf(buf, buf_len, "%.2f", inst->lfo_rate_hz);
    if (strcmp(key, "lfo_rate_div") == 0)
        return snprintf(buf, buf_len, "%s", LFO_DIV_NAMES[inst->lfo_rate_div]);
    if (strcmp(key, "name") == 0)
        return snprintf(buf, buf_len, "FILTER");

    /* State save */
    if (strcmp(key, "state") == 0) {
        return snprintf(buf, buf_len,
            "{\"model\":\"%s\",\"mode\":\"%s\",\"cutoff\":%.3f,"
            "\"resonance\":%.2f,\"drive\":%.2f,\"mix\":%.2f,\"output\":%.1f,"
            "\"env_amount\":%.3f,\"env_attack\":%.0f,\"env_release\":%.0f,"
            "\"lfo_amount\":%.3f,\"lfo_shape\":\"%s\",\"lfo_sync\":\"%s\","
            "\"lfo_rate_hz\":%.2f,\"lfo_rate_div\":\"%s\"}",
            MODEL_NAMES[inst->model], mode_to_string(inst->mode), inst->cutoff,
            inst->resonance, inst->drive, inst->mix, inst->output_db,
            inst->env_amount, inst->env_attack_ms, inst->env_release_ms,
            inst->lfo_amount, LFO_SHAPE_NAMES[inst->lfo_shape],
            inst->lfo_sync ? "Sync" : "Free",
            inst->lfo_rate_hz, LFO_DIV_NAMES[inst->lfo_rate_div]);
    }

    /* UI hierarchy for shadow parameter editor */
    if (strcmp(key, "ui_hierarchy") == 0) {
        const char *hierarchy = "{"
            "\"modes\":null,"
            "\"levels\":{"
                "\"root\":{"
                    "\"children\":null,"
                    "\"knobs\":[\"cutoff\",\"resonance\",\"drive\",\"mix\",\"env_amount\",\"lfo_amount\",\"lfo_rate_div\",\"mode\"],"
                    "\"params\":[\"cutoff\",\"resonance\",\"drive\",\"mix\",\"env_amount\",\"lfo_amount\",\"mode\",\"model\","
                        "{\"level\":\"envelope\",\"label\":\"Envelope\"},"
                        "{\"level\":\"lfo\",\"label\":\"LFO\"},"
                        "{\"level\":\"output\",\"label\":\"Output\"}]"
                "},"
                "\"envelope\":{"
                    "\"label\":\"Envelope\",\"children\":null,"
                    "\"knobs\":[\"env_amount\",\"env_attack\",\"env_release\"],"
                    "\"params\":[\"env_amount\",\"env_attack\",\"env_release\"]"
                "},"
                "\"lfo\":{"
                    "\"label\":\"LFO\",\"children\":null,"
                    /* Order matters to the knob grid: the LFO graphic is drawn
                     * over a RUN of adjacent cells, so depth, rate and shape
                     * have to sit together. With the division enum between
                     * amount and rate_hz the run was broken and the page drew
                     * five loose dials instead of one LFO. Division and Sync
                     * follow it -- they still get their own cells. */
                    "\"knobs\":[\"lfo_amount\",\"lfo_rate_hz\",\"lfo_shape\",\"lfo_rate_div\",\"lfo_sync\"],"
                    "\"params\":[\"lfo_amount\",\"lfo_shape\",\"lfo_sync\",\"lfo_rate_div\",\"lfo_rate_hz\"]"
                "},"
                "\"output\":{"
                    "\"label\":\"Output\",\"children\":null,"
                    "\"knobs\":[\"mix\",\"output\"],"
                    "\"params\":[\"mix\",\"output\"]"
                "}"
            "}"
        "}";
        int len = strlen(hierarchy);
        if (len < buf_len) {
            strcpy(buf, hierarchy);
            return len;
        }
        return -1;
    }

    /* Chain params metadata */
    if (strcmp(key, "chain_params") == 0) {
        const char *params_json = "["
            "{\"key\":\"model\",\"name\":\"Model\",\"type\":\"enum\",\"options\":[\"SVF\",\"Schwoog\"],\"default\":\"SVF\"},"
            "{\"key\":\"mode\",\"name\":\"Mode\",\"type\":\"enum\",\"options\":[\"LP\",\"HP\",\"BP\",\"Notch\",\"Peak\",\"AP\"],\"default\":\"LP\"},"
            "{\"key\":\"cutoff\",\"name\":\"Cutoff\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":0.5,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"resonance\",\"name\":\"Resonance\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":0.2,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"drive\",\"name\":\"Drive\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":0,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"mix\",\"name\":\"Mix\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":1,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"output\",\"name\":\"Output\",\"type\":\"float\",\"min\":-24,\"max\":12,\"default\":0,\"step\":0.5,\"unit\":\"dB\"},"
            "{\"key\":\"env_amount\",\"name\":\"Env Amt\",\"type\":\"float\",\"min\":-1,\"max\":1,\"default\":0,\"step\":0.02},"
            "{\"key\":\"env_attack\",\"name\":\"Env Atk\",\"type\":\"float\",\"min\":1,\"max\":500,\"default\":10,\"step\":1,\"unit\":\"ms\"},"
            "{\"key\":\"env_release\",\"name\":\"Env Rel\",\"type\":\"float\",\"min\":1,\"max\":500,\"default\":150,\"step\":1,\"unit\":\"ms\"},"
            "{\"key\":\"lfo_amount\",\"name\":\"LFO Amt\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":0,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"lfo_shape\",\"name\":\"LFO Shape\",\"type\":\"enum\",\"options\":[\"Sine\",\"Tri\",\"Saw\",\"Sqr\",\"S&H\"],\"default\":\"Sine\"},"
            "{\"key\":\"lfo_sync\",\"name\":\"LFO Sync\",\"type\":\"enum\",\"options\":[\"Free\",\"Sync\"],\"default\":\"Sync\"},"
            "{\"key\":\"lfo_rate_hz\",\"name\":\"LFO Rate\",\"type\":\"float\",\"min\":0.01,\"max\":20,\"default\":1,\"step\":0.01,\"unit\":\"Hz\"},"
            "{\"key\":\"lfo_rate_div\",\"name\":\"LFO Div\",\"type\":\"enum\",\"options\":[\"1/1\",\"1/2\",\"1/4\",\"1/4.\",\"1/8\",\"1/8.\",\"1/8T\",\"1/16\",\"1/16T\",\"1/32\"],\"default\":\"1/8\"}"
        "]";
        int len = strlen(params_json);
        if (len < buf_len) {
            strcpy(buf, params_json);
            return len;
        }
        return -1;
    }

    return -1;
}

/* ---- Plugin entry point ---- */

static audio_fx_api_v2_t g_fx_api_v2;

audio_fx_api_v2_t* move_audio_fx_init_v2(const host_api_v1_t *host) {
    g_host = host;

    memset(&g_fx_api_v2, 0, sizeof(g_fx_api_v2));
    g_fx_api_v2.api_version      = AUDIO_FX_API_VERSION_2;
    g_fx_api_v2.create_instance  = filter_create_instance;
    g_fx_api_v2.destroy_instance = filter_destroy_instance;
    g_fx_api_v2.process_block    = filter_process_block;
    g_fx_api_v2.set_param        = filter_set_param;
    g_fx_api_v2.get_param        = filter_get_param;

    filter_log("FILTER v2 plugin initialized");

    return &g_fx_api_v2;
}
