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

#define SAMPLE_RATE 44100.0

/* Models (only SVF in M1) */
enum {
    MODEL_SVF = 0
};

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

    /* dsp */
    svf_t fL, fR;
    smoother_t sm_cut;      /* normalized 0..1 (== log-Hz, see cutoff_norm_to_hz) */
    smoother_t sm_res;
    smoother_t sm_drive;
    smoother_t sm_mix;
    smoother_t sm_outlin;   /* linear makeup gain */
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

    /* DSP init */
    svf_init(&inst->fL, SAMPLE_RATE);
    svf_init(&inst->fR, SAMPLE_RATE);

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
    for (int i = 0; i < frames; i++) {
        float cut = cutoff_norm_to_hz((float)smooth_next(&inst->sm_cut)); /* 0..1 -> Hz (log) */
        float res = resonance_taper((float)smooth_next(&inst->sm_res));   /* even Q, capped */
        float drv = (float)smooth_next(&inst->sm_drive);
        float mix = (float)smooth_next(&inst->sm_mix);
        float og  = (float)smooth_next(&inst->sm_outlin);

        /* NOTE: svf_set is called twice per sample (L and R), recomputing
         * tan() twice. Acceptable for M1; a later milestone optimizes by
         * computing coefficients once and copying them to the second filter. */
        svf_set(&inst->fL, cut, res, (svf_mode_t)inst->mode);
        svf_set(&inst->fR, cut, res, (svf_mode_t)inst->mode);

        float xl = audio_inout[i * 2]     / 32768.0f;
        float xr = audio_inout[i * 2 + 1] / 32768.0f;

        /* Pre-filter soft saturation. Input gain into tanh, normalized so a
         * full-scale input stays ~unity; higher drive pushes more signal into
         * the nonlinear region (adds harmonics, lifts quiet detail) and feeds
         * the filter — classic for resonant/acid sweeps. (M3's analog models
         * will layer their own per-model nonlinearity on top of this.) */
        if (drv > 0.0005f) {
            float k = 1.0f + drv * 7.0f;
            float inv = 1.0f / tanhf(k);
            xl = tanhf(xl * k) * inv;
            xr = tanhf(xr * k) * inv;
        }

        float yl = (float)svf_process(&inst->fL, xl);
        float yr = (float)svf_process(&inst->fR, xr);

        yl = (xl * (1.0f - mix) + yl * mix) * og;
        yr = (xr * (1.0f - mix) + yr * mix) * og;

        audio_inout[i * 2]     = (int16_t)clampf(yl * 32768.0f, -32768.0f, 32767.0f);
        audio_inout[i * 2 + 1] = (int16_t)clampf(yr * 32768.0f, -32768.0f, 32767.0f);
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
        if (strcasecmp(val, "SVF") == 0) inst->model = MODEL_SVF;
        /* ignore unknown models */
    } else if (strcmp(key, "state") == 0) {
        /* Restore all parameters from JSON state.
         * Use smooth_reset (instant) so a freshly-loaded patch is immediately
         * at the right value with no audible ramp from the default. */
        float fval;
        char sval[32];

        if (json_get_string(val, "model", sval, sizeof(sval)) == 0) {
            if (strcasecmp(sval, "SVF") == 0) inst->model = MODEL_SVF;
        }
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
        return snprintf(buf, buf_len, "SVF");
    if (strcmp(key, "name") == 0)
        return snprintf(buf, buf_len, "FILTER");

    /* State save */
    if (strcmp(key, "state") == 0) {
        return snprintf(buf, buf_len,
            "{\"model\":\"SVF\",\"mode\":\"%s\",\"cutoff\":%.3f,"
            "\"resonance\":%.2f,\"drive\":%.2f,\"mix\":%.2f,\"output\":%.1f}",
            mode_to_string(inst->mode), inst->cutoff,
            inst->resonance, inst->drive, inst->mix, inst->output_db);
    }

    /* UI hierarchy for shadow parameter editor */
    if (strcmp(key, "ui_hierarchy") == 0) {
        const char *hierarchy = "{"
            "\"modes\":null,"
            "\"levels\":{"
                "\"root\":{"
                    "\"children\":null,"
                    "\"knobs\":[\"cutoff\",\"resonance\",\"drive\",\"mix\",\"output\",\"mode\"],"
                    "\"params\":[\"cutoff\",\"resonance\",\"drive\",\"mix\",\"output\",\"mode\",\"model\"]"
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
            "{\"key\":\"model\",\"name\":\"Model\",\"type\":\"enum\",\"options\":[\"SVF\"],\"default\":\"SVF\"},"
            "{\"key\":\"mode\",\"name\":\"Mode\",\"type\":\"enum\",\"options\":[\"LP\",\"HP\",\"BP\",\"Notch\",\"Peak\",\"AP\"],\"default\":\"LP\"},"
            "{\"key\":\"cutoff\",\"name\":\"Cutoff\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":0.5,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"resonance\",\"name\":\"Resonance\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":0.2,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"drive\",\"name\":\"Drive\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":0,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"mix\",\"name\":\"Mix\",\"type\":\"float\",\"min\":0,\"max\":1,\"default\":1,\"step\":0.02,\"unit\":\"%\"},"
            "{\"key\":\"output\",\"name\":\"Output\",\"type\":\"float\",\"min\":-24,\"max\":12,\"default\":0,\"step\":0.5,\"unit\":\"dB\"}"
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
