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
  float encoder_filter_anglestep;
  bool encoder_filter_enable;
  bool encoder_filter_plot_enable;
  bool limits_enable;
  bool orthopus_config_set;
  float limits_pos_max;
  float limits_pos_min;
  float angle_division;
  float limits_reach_angle; //angle margin before the max/min pos limit whitin which the speed is limited (deg)
  float limits_reach_speed; //speed limit in the reach angle (rpm)
} orthopus_config_t; //don't forget to add padding bytes uint8_t pad[1--3];

static orthopus_config_t orthopus_config;

// Utils
static bool orthopus_config_load(orthopus_config_t* cfg);
static bool orthopus_config_save(const orthopus_config_t* cfg);

static float orthopus_read_encoder(void);
static float orthopus_read_encoder_raw(void);
//static float orthopus_read_encoder_filtered(void);
//static float orthopus_read_pos_multiturn(void);
//static float app_orthopus_get_enc_pos_filtered(void);
//static float app_orthopus_get_pos_multiturn(void);
static float orthopus_set_joint_offset(float v, bool use_v);
static float orthopus_set_encoder_offset(float v, bool use_v);


static void orthopus_cmd_init(void);
static void orthopus_cmd_deinit(void);
static void orthopus_init_lisp(void);

//global variables (intefaces with lispBM and terminal)
static volatile float actual_pos_multiturn = 0;
static volatile float enc_pos_filter = 0.0;
static volatile float speed_now = 0.0;