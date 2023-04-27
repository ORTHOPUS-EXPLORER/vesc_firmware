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
  float encoder_filter_error_gain;
  int rate_hz; bool perf_compensateexectime; uint8_t pad[1];
} orthopus_config_t; //don't forget to add padding bytes uint8_t pad[1--3];

static orthopus_config_t orthopus_config;

//global variables (interfaces with lispBM and terminal)
typedef struct
{
  float pos_multiturn_now;
  float enc_pos_filter;
  float speed_now;
  float enc_pos;
  float time_diff;
  float time_diff_filt;
  int time_lag_filt;
  int time_lag_compensation;
  bool perfplot;
  uint maxperiod;
  uint minperiod;
  uint exectime;
  float ADC3val;
  float ADC3zero;
  float Torque;
  float ADC3init;
  float torque_filter_const;
  bool ctrl_enable;
  bool ctrl_plot;
  float ctrl_kp;
  float ctrl_command;
} orthopus_state_t;

static volatile orthopus_state_t orthopus_state;

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

// cmd
static void orthopus_cmd_init(void);
static void orthopus_cmd_deinit(void);
// lisp
static void orthopus_init_lisp(void);

//algo
static void orthopus_estop(void);
static void orthopus_limits(void);
static void orthopus_plot_encoder_filtering(int ns);
static void orthopus_plot_cycletime(int ns);
static void orthopus_plot_impedance(int ns);
