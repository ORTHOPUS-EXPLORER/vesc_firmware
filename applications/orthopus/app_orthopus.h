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
  /*    - 1 */bool or_conf_set;
  /* 03 - 4 */float limits_pos_max;
  /* 04 - 4 */float limits_pos_min;
  /* 05 - 4 */float angle_division;
  /* 06 - 4 */float limits_reach_angle; //angle margin before the max/min pos limit whitin which the speed is limited (deg)
  /* 07 - 4 */float limits_reach_speed; //speed limit in the reach angle (rpm)
  /* 08 - 4 */float encoder_filter_error_gain;
  /* 09 - 4 */int perf_rate_hz;
  /* 10 - 1 */bool perf_compensateexectime;
  /* 10 - 1 */bool ctrl_deadzone;
  /* 10 - 1 */bool ctrl_sample_adc3;
  /*    - 1 */uint8_t pad[1];
  /* 11 - 4 */float ctrl_torquezero;
  /* 12 - 4 */float ctrl_torquegain;
  /* 13 - 4 */float limits_kp;
  /* 14 - 4 */float limits_kd;
  /* 15 - 4 */int limits_powp;
  /* 16 - 4 */int limits_powd;
  /* 17 - 4 */int limits_damp_reachangle;
  /* 18 - 4 */float ctrl_stiffness;
  /* 19 - 4 */float ctrl_damping;
  /* 19 - 4 */float torque_filter_const;
  /* 20 - 4 */float ctrl_kp;
  /* 21 - 4 */float ctrl_a;
  /* 22 - 4 */float ctrl_kd;
  /* 23 - 4 */float ctrl_kd_filter;
  /* 24 - 4 */float encoder_max_diff;
} orthopus_config_t; //don't forget to add padding bytes uint8_t pad[1--3];

static orthopus_config_t or_conf;

//global variables (interfaces with lispBM and terminal)
typedef struct
{
  float pos_multiturn_now;            //multiturn position based from mc_interface
  float enc_pos_filter;//encoder position filtered (removed outliers)
  float enc_pos_filter_multiturn;//encoder position filtred and multiturn
  int enc_turn; //encoder angle turn count
  float speed_now; //actual speed from mc_interface
  float enc_pos; //raw encoder position
  float time_diff;
  float time_diff_filt;
  int time_lag_filt;
  int time_lag_compensation;
  bool perf_plot;                             //enables realtime performance plot 
  int perf_max_period;                                  //maximum execution time in us
  int perf_min_period;                                  //maximum execution time in us
  int perf_exec_time;                                      //loop time in system ticks
  float adc3_val;
  float adc3_zero;
  float torque_now;
  float adc3_init;
  bool ctrl_enable;
  bool ctrl_plot;
  float ctrl_command;
  int turn_now;
  float ext_torque_setpoint;
  float ext_pos_setpoint;
  bool ctrl_overwrite;
  float limit_reaction;
  float torque_err;
  float last_ctrl_command;
  float nid1; //number of non null identical ctrl command 
  bool stopped;
  float adc3_filt;
  float torque_err_last;
  float d_torque_err;
  float d_torque_err_filt;
  bool encoders_init;
} orthopus_state_t;

extern volatile orthopus_state_t or_state;

// Utils
static bool orthopus_config_load(orthopus_config_t* cfg);
static bool orthopus_config_save(const orthopus_config_t* cfg);
static void orthopus_config_reset(orthopus_config_t* cfg);

static float orthopus_read_encoder(void);
static float orthopus_read_encoder_raw(void);
static float orthopus_set_joint_offset(float v, bool use_v);
static float orthopus_set_encoder_offset(float v, bool use_v);

// cmd
static void orthopus_cmd_init(void);
static void orthopus_cmd_deinit(void);
// lisp
static void orthopus_init_lisp(void);

//algo
static void orthopus_estop(void);
static bool orthopus_safety(void);
static void orthopus_limits(void);
static void orthopus_plot_encoder_filtering(int ns);
static void orthopus_plot_cycletime(int ns);
static void orthopus_plot_impedance(int ns);
