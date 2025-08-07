#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ch.h"
#include "datatypes.h"
#include "chconf.h"

// Algo
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
  /* 01 - 4 */uint32_t signature;
  /* 02 - 1 */bool limits_enable_reaction;
  /* 02 - 1 */bool auto_clear_errors;
  /*    - 1 */bool limits_enable;
  /*    - 1 */uint8_t stream_rate_10;
  /* 03 - 4 */float limits_pos_max;
  /* 04 - 4 */float limits_pos_min;
  /* 05 - 4 */float limits_reach_angle; //angle margin before the max/min pos limit whitin which the speed is limited (deg)
  /* 06 - 4 */float limits_reach_speed; //speed limit in the reach angle (rpm)
  /* 07 - 4 */char joint_name[4];
  /* 08 - 4 */int perf_rate_hz;
  /* 09 - 1 */bool perf_compensateexectime;
  /*    - 1 */bool ctrl_deadzone;
  /*    - 1 */bool ctrl_sample_adc3;
  /*    - 1 */bool simu_mode;
  /* 10 - 4 */float ctrl_torquezero;
  /* 11 - 4 */float ctrl_torquegain;
  /* 12 - 4 */float limits_kp;
  /* 13 - 4 */float limits_kd;
  /* 14 - 4 */int limits_powp;
  /* 15 - 4 */int limits_powd;
  /* 16 - 4 */int limits_damp_reachangle;
  /* 17 - 4 */float ctrl_stiffness;
  /* 18 - 4 */float ctrl_damping;
  /* 19 - 4 */float torque_filter_const;
  /* 20 - 4 */float ctrl_kp;
  /* 21 - 4 */float ctrl_a;
  /* 22 - 4 */float ctrl_kd;
  /* 23 - 4 */float ctrl_kd_filter;
  /* 24 - 4 */float encoder_max_diff;
  /* 25 - 4 */float safety_max_q_error;
  /* 26 - 4 */float safety_max_speed;
  /* 27 - 1 */bool safety_track_disable;
  /*    - 1 */bool safety_timeout_disable;
  /*    - 1 */bool safety_max_speed_disable;
  /*    - 1 */bool pad[1];
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
} or_state_t;

extern volatile or_state_t or_state;

typedef struct 
{
  uint16_t   word;
  float     pos,
            vel,
            trq;
} or_comm_control_t;

typedef struct 
{
  uint16_t  word;
  float     pos,
            vel,
            trq;
} or_comm_state_t;

typedef struct
{
  or_comm_state_t   st0, 
                          st1;
  volatile or_comm_state_t *state;
  or_comm_control_t ctrl0, 
                          ctrl1;
  volatile or_comm_control_t *ctrl,
                                    *ctrl_prev;
  volatile bool process_ctrl,
                process_rx;
  systime_t last_update;
} or_comm_t;

/** @brief Error severity levels */
typedef enum {
  ERR_LEVEL_NONE   = 0,
  ERR_LEVEL_WARNING,     // Can continue
  ERR_LEVEL_HOLD,        // Needs HOLD
  ERR_LEVEL_BRAKE,       // Needs BRAKE
  ERR_LEVEL_ESTOP,        // Must stop immediately
  ERR_LEVEL_COUNT,
} or_error_level_t;

/** @brief Internal error types */
typedef enum {
  ERR_NONE = 0,

  // Control-related
  ERR_POS_STEP,
  ERR_VEL_STEP,
  ERR_TRQ_STEP,
  ERR_SAME_CTRL_OUT,

  //Limits:
  ERR_POS_LIMIT,
  ERR_SPEED_LIMIT,

  //Test errors:
  ERR_TST_WARNING,
  ERR_TST_HOLD,
  ERR_TST_BRAKE,
  ERR_TST_ESTOP,

  // Add others...
  ERR_STP_HOLD,
  ERR_CAN_TIMEOUT,
  ERR_MAX_SPEED,

  ERR_COUNT // Always last
} or_error_t;

extern or_comm_t or_comm;
extern bool or_active_errors[ERR_COUNT];
extern bool or_error_triggered[ERR_COUNT];

// Utils
bool or_config_load(orthopus_config_t* cfg);
bool or_config_save(const orthopus_config_t* cfg);
bool or_config_set(orthopus_config_t* cfg, const uint8_t* buffer);

float or_read_encoder(void);
float or_read_encoder_raw(void);
float or_set_joint_offset(float v, bool use_v);
float or_set_encoder_offset(float v, bool use_v);

// cmd
void or_cmd_init(void);
void or_cmd_deinit(void);
// lisp
void or_init_lisp(void);

//algo
void or_pwm_callback(void);
void or_estop(void);
bool or_safety(void);
void or_limits_reaction(void);
void or_plot_encoder_filtering(int ns);
void or_plot_cycletime(int ns);
void or_plot_impedance(int ns);
or_error_level_t or_get_error_severity(or_error_t err);
or_error_level_t or_compute_max_error_level(void);
void or_raise_error(or_error_t err);
void or_clear_error(or_error_t err);
uint16_t or_evaluate_safety_state(void);
void or_set_safety_mode(uint32_t mode);
void or_sync_error_flags(void);
void or_interface_torquecontrol(void);
void or_set_control_mode(uint32_t mode);

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
#define OR_COMM_RT_POS_SCALE 90  // 0->360 deg
#define OR_COMM_RT_VEL_SCALE 600 // -50->50 rpm
#define OR_COMM_RT_TRQ_SCALE 600 // -50->50 Nm
#define OR_COMM_AUX_SERVO_SCALE 1000

#define OR_CTRL_MODE_OFF 0x0000 //*0000
#define OR_CTRL_MODE_POS 0x0001 //*0001
#define OR_CTRL_MODE_VEL 0x0002 //*0010
#define OR_CTRL_MODE_TRQ 0x0004 //*0100
#define OR_CTRL_MODE_IMP 0x0007 //*0111
#define OR_CTRL_MODE_CST 0x000F //*1111 custom mode
#define OR_CTRL_MODE_MSK 0x000F

#define OR_STATE_MODE_OFF     OR_CTRL_MODE_OFF
#define OR_STATE_MODE_POS     OR_CTRL_MODE_POS
#define OR_STATE_MODE_VEL     OR_CTRL_MODE_VEL
#define OR_STATE_MODE_TRQ     OR_CTRL_MODE_TRQ
#define OR_STATE_MODE_IMP     OR_CTRL_MODE_IMP
#define OR_STATE_MODE_CST     OR_CTRL_MODE_CST
#define OR_STATE_MODE_MSK     OR_CTRL_MODE_MSK
#define OR_STATE_ERR_POS_STEP 0x0010
#define OR_STATE_ERR_VEL_STEP 0x0020
#define OR_STATE_ERR_TRQ_STEP 0x0040
#define OR_STATE_ERR_OTHER    0x0080
#define OR_STATE_ERR_MSK      0x00F0

#define OR_SAFETY_INIT    0x0000
#define OR_SAFETY_IDLE    0x0100
#define OR_SAFETY_ENABLE  0x0200
#define OR_SAFETY_HOLD    0x0300
#define OR_SAFETY_BRAKE   0x0400
#define OR_SAFETY_ESTOP   0x0500

#define OR_SAFETY_MSK     0x0F00

#include "_gen/orthopus_confparser.h"
#include "_gen/orthopus_confxml.h"
#include "_gen/orthopus_conf_default.h"