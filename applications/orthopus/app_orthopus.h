#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ch.h"
#include "datatypes.h"

// Algo
static void orthopus_pwm_callback(void);
static THD_FUNCTION(orthopus_thread, arg);
static volatile bool orthopus_thread_stop,
                     orthopus_thread_running;
// Config
// 1: uint8_t, int8_t, bool
// 2: uint16_t, int16_t
// 4: uint32_t, int32_t, int, float
typedef struct
{
  /* EEPROM Addr - Size */
  /* 00 - 4 */float encoder_offset;
  /* 01 - 4 */float encoder_filter_anglestep;
  /* 02 - 1 */bool encoder_filter_enable;
  /*    - 1 */bool encoder_filter_plot_enable;
  /*    - 1 */bool limits_enable;
  /*    - 1 */bool orthopus_config_set;
  /* 03 - 4 */float limits_pos_max;
  /* 04 - 4 */float limits_pos_min;
  /* 05 - 4 */float angle_division;
  /* 06 - 4 */float limits_reach_angle; //angle margin before the max/min pos limit whitin which the speed is limited (deg)
  /* 07 - 4 */float limits_reach_speed; //speed limit in the reach angle (rpm)
  /* 08 - 4 */float encoder_filter_error_gain;
  /* 09 - 4 */int rate_hz; 
  /* 10 - 1 */bool perf_compensateexectime; 
  /*    - 3 */uint8_t pad[3];
  /* 11 - 4 */float Torquezero;
  /* 12 - 4 */float Torquegain;
  /* 13 - 4 */float limits_kp;
  /* 14 - 4 */float limits_kd;
  /* 15 - 4 */int limits_powp;
  /* 16 - 4 */int limits_powd;
  /* 17 - 4 */int limits_damp_reachangle;
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
  int maxperiod;
  int minperiod;
  int exectime;
  float ADC3val;
  float ADC3zero;
  float Torque;
  float ADC3init;
  float torque_filter_const;
  bool ctrl_enable;
  bool ctrl_plot;
  float ctrl_kp;
  float ctrl_command;
  bool deadzone;
  float a;
  float stiffness;
  int turn_now;
  float ext_torque_setpoint;
  float ext_pos_setpoint;
  bool ctrl_overwrite;
  float limitreaction;
  float torqueerror;
  float damping;
} orthopus_state_t;

extern volatile orthopus_state_t orthopus_state;

// Utils
static bool orthopus_config_load(orthopus_config_t* cfg);
static bool orthopus_config_save(const orthopus_config_t* cfg);
static void orthopus_config_reset(orthopus_config_t* cfg);

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
