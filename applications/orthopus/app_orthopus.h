#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ch.h"
#include "datatypes.h"

// Algo
void orthopus_pwm_callback(void);
THD_FUNCTION(orthopus_thread, arg);
extern volatile bool orthopus_thread_stop,
                     orthopus_thread_running;
THD_FUNCTION(orthopus_comm_thread, arg);
extern volatile bool orthopus_comm_thread_stop,
                    orthopus_comm_thread_running;

// MAX Number of U32 words to store/load to/from EEPROM fo Config
#define MAX_CONFIG_U32_SIZE 32
// Config
// 1: uint8_t, int8_t, bool
// 2: uint16_t, int16_t
// 4: uint32_t, int32_t, int, float
typedef struct
{
  /* EEPROM Addr - Size */
  /* 00 - 4 */float encoder_offset; 
  /* 01 - 4 */
  /* 02 - 1 */
  /*    - 1 */uint8_t pad[2];
  /*    - 1 */bool limits_enable;
  /*    - 1 */uint8_t stream_rate_10;
  /* 03 - 4 */float limits_pos_max;
  /* 04 - 4 */float limits_pos_min;
  /* 05 - 4 */float angle_division;
  /* 06 - 4 */float limits_reach_angle; //angle margin before the max/min pos limit whitin which the speed is limited (deg)
  /* 07 - 4 */float limits_reach_speed; //speed limit in the reach angle (rpm)
  /* 08 - 4 */
  /* 09 - 4 */int perf_rate_hz;
  /* 10 - 1 */bool perf_compensateexectime;
  /*    - 1 */bool ctrl_deadzone;
  /*    - 1 */bool ctrl_sample_adc3;
  /*    - 1 */bool simu_mode;
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
  /* 25 - 4 */char joint_name[4];
  /* 26 - 4 */uint32_t signature;
  /* 27 - 4 */float safety_max_q_error;
} orthopus_config_t; // don't forget to add padding bytes uint8_t pad[1--3];

extern orthopus_config_t or_conf;

//global variables (interfaces with lispBM and terminal)
typedef struct
{
  float pos_multiturn_now;//multiturn position based from mc_interface
  float enc_pos_multiturn;//encoder position filtred and multiturn
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
  float ext_vel_setpoint;
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

typedef struct 
{
  uint16_t   word;
  float     pos,
            vel,
            trq;
} orthopus_comm_control_t;

typedef struct 
{
  uint16_t  word;
  float     pos,
            vel,
            trq;
} orthopus_comm_state_t;

typedef struct
{
  orthopus_comm_state_t   st0, 
                          st1;
  volatile orthopus_comm_state_t *state;
  orthopus_comm_control_t ctrl0, 
                          ctrl1;
  volatile orthopus_comm_control_t*ctrl;
  volatile bool process_ctrl,
                process_rx;
} orthopus_comm_t;

extern orthopus_comm_t orthopus_comm;

// Utils
bool orthopus_config_load(orthopus_config_t* cfg);
bool orthopus_config_save(const orthopus_config_t* cfg);
bool orthopus_config_set(orthopus_config_t* cfg, const uint8_t* buffer);

float orthopus_read_encoder(void);
float orthopus_read_encoder_raw(void);
float orthopus_set_joint_offset(float v, bool use_v);
float orthopus_set_encoder_offset(float v, bool use_v);

// cmd
void orthopus_cmd_init(void);
void orthopus_cmd_deinit(void);
// lisp
void orthopus_init_lisp(void);

//algo
void orthopus_estop(void);
bool orthopus_safety(void);
void orthopus_limits(void);
void orthopus_plot_encoder_filtering(int ns);
void orthopus_plot_cycletime(int ns);
void orthopus_plot_impedance(int ns);

/**
 * @brief   System ticks to microseconds.
 * @details Converts from system ticks number to microseconds.
 * @note    The result is rounded up to the next microsecond boundary.
 *
 * @param[in] n         number of system ticks
 * @return              The number of microseconds.
 *
 * @api
 */
#define ST2US2(n) (((n) * 1000000UL + 10000UL - 1UL) /           \
                  10000UL)
//TODO: replace By ST2US after testing


// Orthopus RT COMM definitions
// CAN Interface
#define CAN_RT_UPSTREAM_INTF   0
// CAN Endpoints
#define CAN_RT_DATA_UPSTREAM    179
#define CAN_RT_DATA_DOWNSTREAM  180
//#define CAN_AUX_DATA_UPSTREAM    181
#define CAN_AUX_DATA_DOWNSTREAM 182

// Float scaling
#define ORTHOPUS_COMM_RT_POS_SCALE 50
#define ORTHOPUS_COMM_RT_VEL_SCALE 1
#define ORTHOPUS_COMM_RT_TRQ_SCALE 50
#define ORTHOPUS_COMM_AUX_SERVO_SCALE 1000

#define ORTHOPUS_CTRL_MODE_OFF 0x0000
#define ORTHOPUS_CTRL_MODE_POS 0x0001
#define ORTHOPUS_CTRL_MODE_VEL 0x0002
#define ORTHOPUS_CTRL_MODE_TRQ 0x0004
#define ORTHOPUS_CTRL_MODE_IMP 0x0005
#define ORTHOPUS_CTRL_MODE_CST 0x0006
#define ORTHOPUS_CTRL_MODE_MSK 0x000F

#define ORTHOPUS_STATE_MODE_OFF     ORTHOPUS_CTRL_MODE_OFF
#define ORTHOPUS_STATE_MODE_POS     ORTHOPUS_CTRL_MODE_POS
#define ORTHOPUS_STATE_MODE_VEL     ORTHOPUS_CTRL_MODE_VEL
#define ORTHOPUS_STATE_MODE_TRQ     ORTHOPUS_CTRL_MODE_TRQ
#define ORTHOPUS_STATE_MODE_IMP     ORTHOPUS_CTRL_MODE_IMP
#define ORTHOPUS_STATE_MODE_CST     ORTHOPUS_CTRL_MODE_CST
#define ORTHOPUS_STATE_MODE_MSK     ORTHOPUS_CTRL_MODE_MSK
#define ORTHOPUS_STATE_ERR_POS_STEP 0x0010
#define ORTHOPUS_STATE_ERR_VEL_STEP 0x0020
#define ORTHOPUS_STATE_ERR_MSK      0x0030

// We could remove these if we define the right values in the XML file: _gen/orthopus_settings.xml (using VESC Tool XML Editor)
/*
#define ORTHOPUS_CFG_DEF_ENCODER_OFFSET             0.0
#define ORTHOPUS_CFG_DEF_ENCODER_FILTER_ANGLESTEP   0.25 //TODO remove
#define ORTHOPUS_CFG_DEF_ENCODER_FILTER_ENABLE      true //TODO remove // keep enabled or move encoder filtered multiturn angle estimation
#define ORTHOPUS_CFG_DEF_ENCODER_FILTER_PLOT_ENABLE false //TODO remove
#define ORTHOPUS_CFG_DEF_ENCODER_FILTER_ERROR_GAIN  1 //TODO remove
#define ORTHOPUS_CFG_DEF_ENCODER_MAX_DIFF           5.0
#define ORTHOPUS_CFG_DEF_LIMITS_ENABLE              false
#define ORTHOPUS_CFG_DEF_LIMITS_POS_MIN             -90.0
#define ORTHOPUS_CFG_DEF_LIMITS_POS_MAX             90.0
#define ORTHOPUS_CFG_DEF_LIMITS_KP                  0.0
#define ORTHOPUS_CFG_DEF_LIMITS_KD                  5.0
#define ORTHOPUS_CFG_DEF_LIMITS_POWP                6.0
#define ORTHOPUS_CFG_DEF_LIMITS_POWD                1.0
#define ORTHOPUS_CFG_DEF_LIMITS_DAMP_REACHANGLE     7.0
#define ORTHOPUS_CFG_DEF_LIMITS_REACH_ANGLE         15
#define ORTHOPUS_CFG_DEF_LIMITS_REACH_SPEED         2
#define ORTHOPUS_CFG_DEF_ANGLE_DIVISION             700
#define ORTHOPUS_CFG_DEF_PERF_RATE_HZ               2000
#define ORTHOPUS_CFG_DEF_PERF_COMPENSATEEXECTIME    false
#define ORTHOPUS_CFG_DEF_CTRL_DEADZONE              true
#define ORTHOPUS_CFG_DEF_CTRL_SAMPLE_ADC3           true
#define ORTHOPUS_CFG_DEF_CTRL_STIFFNESS             0.0
#define ORTHOPUS_CFG_DEF_CTRL_TORQUEZERO            0.0
#define ORTHOPUS_CFG_DEF_CTRL_TORQUEGAIN            34.8
#define ORTHOPUS_CFG_DEF_CTRL_DAMPING               0.0
#define ORTHOPUS_CFG_DEF_CTRL_KP                    4.0
#define ORTHOPUS_CFG_DEF_CTRL_A                     1.0
#define ORTHOPUS_CFG_DEF_CTRL_KD                    0.0
#define ORTHOPUS_CFG_DEF_CTRL_KD_FILTER             1.0
#define ORTHOPUS_CFG_DEF_TORQUE_FILTER_CONST        0.01*/

#include "_gen/orthopus_confparser.h"
#include "_gen/orthopus_confxml.h"
#include "_gen/orthopus_conf_default.h" // Should not do anything since we just defined the default values above