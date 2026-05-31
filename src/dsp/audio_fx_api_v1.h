/*
 * Audio FX Plugin API v1 and v2
 *
 * Interface for audio effect plugins that process stereo audio in-place.
 */

#ifndef AUDIO_FX_API_V1_H
#define AUDIO_FX_API_V1_H

#include <stdint.h>
#include "plugin_api_v1.h"  /* For host_api_v1_t */

#define AUDIO_FX_API_VERSION 1
#define AUDIO_FX_INIT_SYMBOL "move_audio_fx_init_v1"

/* Audio FX plugin interface v1 (singleton) */
typedef struct audio_fx_api_v1 {
    uint32_t api_version;
    int (*on_load)(const char *module_dir, const char *config_json);
    void (*on_unload)(void);
    void (*process_block)(int16_t *audio_inout, int frames);
    void (*set_param)(const char *key, const char *val);
    int (*get_param)(const char *key, char *buf, int buf_len);
} audio_fx_api_v1_t;

typedef audio_fx_api_v1_t* (*audio_fx_init_v1_fn)(const host_api_v1_t *host);

/* Audio FX plugin interface v2 (multi-instance) */
#define AUDIO_FX_API_VERSION_2 2
#define AUDIO_FX_INIT_V2_SYMBOL "move_audio_fx_init_v2"

typedef struct audio_fx_api_v2 {
    uint32_t api_version;
    void* (*create_instance)(const char *module_dir, const char *config_json);
    void (*destroy_instance)(void *instance);
    void (*process_block)(void *instance, int16_t *audio_inout, int frames);
    void (*set_param)(void *instance, const char *key, const char *val);
    int (*get_param)(void *instance, const char *key, char *buf, int buf_len);
} audio_fx_api_v2_t;

typedef audio_fx_api_v2_t* (*audio_fx_init_v2_fn)(const host_api_v1_t *host);

#endif /* AUDIO_FX_API_V1_H */
