#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ch.h"

// Algo
static void orthopus_pwm_callback(void);
static THD_FUNCTION(orthopus_thread, arg);
static volatile bool orthopus_thread_stop,
                     orthopus_thread_running;
// Config
typedef struct
{
  float encoder_offset;
} orthopus_config_t;

static orthopus_config_t orthopus_config;

// Utils
static bool orthopus_config_load(orthopus_config_t* cfg);
static bool orthopus_config_save(const orthopus_config_t* cfg);

static float orthopus_read_encoder(void);
static float orthopus_read_encoder_raw(void);
static float orthopus_set_joint_offset(float v, bool use_v);
static float orthopus_set_encoder_offset(float v, bool use_v);


static void orthopus_cmd_init(void);
static void orthopus_cmd_deinit(void);
static void orthopus_init_lisp(void);