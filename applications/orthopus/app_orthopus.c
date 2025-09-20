/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "lispif.h"
#include "lispbm.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "util/mempools.h"
#include "encoder/encoder.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "buffer.h"
#include "driver/pwm_servo.h"

#include "conf_custom.h" // For conf_custom_add_config, conf_custom_clear_configs

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "app_orthopus.h"

// Threads
THD_WORKING_AREA(orthopus_thread_wa, 512);
THD_WORKING_AREA(orthopus_comm_thread_wa, 512);

// Just make sure to pad to a 32bit aligned size.
// Eg: if you add an uint8_t param, add 3 bytes of padding after.
//                   uint16_t param     2
// uint8_t myparam; uint8_t[3] pad;
orthopus_config_t or_conf =
{
  .encoder_offset = 0.0,
}; //init values to zero in case flash can't be read

or_comm_t or_comm =
{
  .st0 = {},
  .st1 = {},
  .state = &or_comm.st0,
  .ctrl0 = { 
    .word = 0x0,
  },
  .ctrl1 = { 
    .word = 0x0,
  },
  .ctrl  = &or_comm.ctrl0,
  .ctrl_prev = &or_comm.ctrl0,
  .process_ctrl = true,
  .process_rx = true,
};

void or_process_custom_app_data(unsigned char *rx_d, unsigned int len);
void or_process_custom_hw_data(unsigned char *rx_d, unsigned int len);

bool or_process_can_sid(uint32_t id, uint8_t *data, uint8_t len);
bool or_process_can_eid(uint32_t id, uint8_t *data, uint8_t len);

int app_custom_get_cfg(uint8_t *data, bool is_default);
bool app_custom_set_cfg(uint8_t *data);
int app_custom_get_cfg_xml(uint8_t **data);

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
  commands_printf("OrthopusAppStart");
  if(app_get_configuration()->can_baud_rate != CAN_BAUD_1M)
  {
    app_configuration *appconf = mempools_alloc_appconf();
    *appconf = *app_get_configuration();
    appconf->can_baud_rate = CAN_BAUD_1M;
    conf_general_store_app_configuration(appconf);
    app_set_configuration(appconf);
    mempools_free_appconf(appconf);
  }

  // Load config from EEPROM
  if(!or_config_load(&or_conf))
     commands_printf("Orthopus_config_load failed");
  // if orthopus config not set, set default values
  if (or_conf.signature != ORTHOPUS_CONFIG_T_SIGNATURE)
  {
    or_config_set(&or_conf, NULL);
  }

  //conf_custom_clear_configs():
  conf_custom_add_config(
    &app_custom_get_cfg,
    &app_custom_set_cfg,
    &app_custom_get_cfg_xml);

  // Init AMS sensor
  SENSOR_PORT_3V3();
  if(!enc_as504x_init(&encoder_cfg_as504x))
    commands_printf("AMS init failed");

  // Re-init ADC_Ext3 since enc_as504x_init() may have overwritten it...
	// Use SCL/MOSI/TX as ADC_EXT3.
	//   Header pin is tied to PA7 (AIN7) and PB10.
	//   Set PA7 as Analog input and PB10 in Hi-Z.
	palSetPadMode(GPIOA,  7, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 10, PAL_MODE_INPUT);

  // Add shell commands
  or_cmd_init();

  // Add LISP commands/symbols
  lispif_add_ext_load_callback(&or_init_lisp);

  // Custom Packets Handlers
  commands_set_app_data_handler(or_process_custom_app_data);
  commands_set_hw_data_handler(or_process_custom_hw_data);

  // Custom CAN handlers
  comm_can_set_sid_rx_callback(or_process_can_sid);
  comm_can_set_eid_rx_callback(or_process_can_eid);

 // Hard-RT context
	mc_interface_set_pwm_callback(or_pwm_callback);

  // Custom thread
	orthopus_thread_stop = false;
	chThdCreateStatic(orthopus_thread_wa, sizeof(orthopus_thread_wa), NORMALPRIO+40, orthopus_thread, NULL);
  orthopus_comm_thread_stop = false;
  chThdCreateStatic(orthopus_comm_thread_wa, sizeof(orthopus_comm_thread_wa), NORMALPRIO, orthopus_comm_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
  // RT Context
	mc_interface_set_pwm_callback(0);
  // Custom thread
	orthopus_thread_stop = true;
	while (orthopus_thread_running)
		chThdSleepMilliseconds(1);

  // CustomPacket handlers
  commands_set_app_data_handler(0);
  commands_set_hw_data_handler(0);
  // Custom CAN handlers
  comm_can_set_sid_rx_callback(0);
  comm_can_set_eid_rx_callback(0);

  // Commands
	or_cmd_deinit();
}

void app_custom_configure(app_configuration *conf) 
{
	(void)conf;
}

int app_custom_get_cfg(uint8_t *data, bool is_default)
{
  orthopus_config_t cfg;
  memcpy(&cfg,&or_conf, sizeof(orthopus_config_t));
	if (is_default) {
    or_config_set(&cfg, NULL);
	}
	
	return orthopus_confparser_serialize_orthopus_config_t(data, &cfg);
}

bool app_custom_set_cfg(uint8_t *data)
{
	orthopus_config_t cfg;
	bool res = or_config_set(&cfg, data);
  if(res)
  {
    memcpy(&or_conf,&cfg, sizeof(orthopus_config_t));
  }
	
  // FIXME: uncomment WHEN we decide it's a good idea to save or_conf here
  //if(res)
  // res = or_config_save(&or_conf);
	
	return res;
}

int app_custom_get_cfg_xml(uint8_t **data)
{
	*data = data_orthopus_config_t_;
	return DATA_ORTHOPUS_CONFIG_T__SIZE;
}

volatile bool orthopus_comm_thread_stop = true;
volatile bool orthopus_comm_thread_running = false;

#define SIMU_LP_ALPHA 0.005
#define SIMU_INERTIA 0.01f  // kg*m^2 - simulation inertia
#define SIMU_DAMPING 0.1f   // Nm*s/rad - simulation damping

// Simulation state variables for physics calculations
static float simu_prev_pos = 0.0f;
static float simu_prev_vel = 0.0f;
static float simu_prev_time = 0.0f;

THD_FUNCTION(orthopus_comm_thread, arg) 
{
  (void)arg;
	chRegSetThreadName("OrthoCommTh");
  //systime_t time_now, time_last, time_start, time_end, exectime;

  orthopus_comm_thread_running = true;
  for(;;)
  {
    //time_start = chVTGetSystemTimeX();
		// Check if it is time to stop.
		if (orthopus_comm_thread_stop) {
			orthopus_comm_thread_running = false;
			return;
		}

    // Read RX, done in CAN Callback
    if(or_conf.simu_mode)
    {
      or_comm_state_t* st = (or_comm_state_t*)or_comm.state;
      if(or_comm.process_ctrl)
      {
        // Get the current Refs and refs
        const or_comm_control_t* ctrl = (or_comm_control_t*)or_comm.ctrl;
        
        // Calculate time step for physics integration
        float current_time = chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
        float dt = (simu_prev_time > 0.0f) ? (current_time - simu_prev_time) : 0.001f; // Default 1ms if first call
        if (dt > 0.1f) dt = 0.001f; // Cap dt to prevent instability
        
        // Determine control mode from control word
        uint16_t ctrl_mode = ctrl->word & OR_CTRL_MODE_MSK;
        
        switch(ctrl_mode) {
          case OR_CTRL_MODE_POS:
            // Position control mode - calculate velocity from position derivative
            {
              // Handle position with proper angle wrapping
              float pos_diff = utils_angle_difference_rad(ctrl->pos, st->pos);
              st->pos += SIMU_LP_ALPHA * pos_diff;
              // Normalize to [0, 2*pi]
              while (st->pos < 0.0) st->pos += 2.0 * M_PI;
              while (st->pos >= 2.0 * M_PI) st->pos -= 2.0 * M_PI;
              
              // Calculate velocity from position change
              if (simu_prev_time > 0.0f) {
                float pos_change = utils_angle_difference_rad(st->pos, simu_prev_pos);
                st->vel = pos_change / dt;
                // Apply low-pass filter to smooth velocity
                st->vel = simu_prev_vel + SIMU_LP_ALPHA * (st->vel - simu_prev_vel);
              }
              
              // Torque follows with low-pass filter
              st->trq += SIMU_LP_ALPHA * (ctrl->trq - st->trq);
            }
            break;
            
          case OR_CTRL_MODE_VEL:
            // Velocity control mode - integrate velocity to get position
            {
              // Apply velocity control with low-pass filter
              st->vel += SIMU_LP_ALPHA * (ctrl->vel - st->vel);
              
              // Integrate velocity to get position
              if (simu_prev_time > 0.0f) {
                st->pos += st->vel * dt;
                // Normalize to [0, 2*pi]
                while (st->pos < 0.0) st->pos += 2.0 * M_PI;
                while (st->pos >= 2.0 * M_PI) st->pos -= 2.0 * M_PI;
              }
              
              // Torque follows with low-pass filter
              st->trq += SIMU_LP_ALPHA * (ctrl->trq - st->trq);
            }
            break;
            
          case OR_CTRL_MODE_TRQ:
            // Torque control mode - use physics (F=ma) to calculate acceleration, then integrate
            {
              // Apply torque control with low-pass filter
              st->trq += SIMU_LP_ALPHA * (ctrl->trq - st->trq);
              
              if (simu_prev_time > 0.0f) {
                // Calculate acceleration from torque: a = (T - damping*v) / inertia
                float damping_torque = SIMU_DAMPING * st->vel;
                float net_torque = st->trq - damping_torque;
                float acceleration = net_torque / SIMU_INERTIA;
                
                // Integrate acceleration to get velocity
                st->vel += acceleration * dt;
                
                // Integrate velocity to get position
                st->pos += st->vel * dt;
                // Normalize to [0, 2*pi]
                while (st->pos < 0.0) st->pos += 2.0 * M_PI;
                while (st->pos >= 2.0 * M_PI) st->pos -= 2.0 * M_PI;
              }
            }
            break;
            
          default:
            // Default case - simple low-pass filter on all variables
            {
              float pos_diff = utils_angle_difference_rad(ctrl->pos, st->pos);
              st->pos += SIMU_LP_ALPHA * pos_diff;
              while (st->pos < 0.0) st->pos += 2.0 * M_PI;
              while (st->pos >= 2.0 * M_PI) st->pos -= 2.0 * M_PI;
              
              st->vel += SIMU_LP_ALPHA * (ctrl->vel - st->vel);
              st->trq += SIMU_LP_ALPHA * (ctrl->trq - st->trq);
            }
            break;
        }
        
        // Store current values for next iteration
        simu_prev_pos = st->pos;
        simu_prev_vel = st->vel;
        simu_prev_time = current_time;
      }
    }
    else
    {
      or_comm_state_t* st = (or_comm_state_t*)or_comm.state;
      st->pos = DEG2RAD_f(mc_interface_get_pid_pos_now());
      st->vel = RPM2RADPS_f(or_state.speed_now);
      st->trq = or_state.torque_now;
    }
    
    // Do not send meas in INIT state
    if((or_comm.state->word & OR_STATE_MSK) != OR_STATE_INIT)
    {
      // Write TX
      unsigned int rate = or_conf.stream_rate_10*10; // Fast copy to avoid locking it before use
      if(rate > 0)
      {
        // TX
        // Get the current buffer
        or_comm_state_t* st = (or_comm_state_t*)or_comm.state;
        // Copy the data to the send buffer
        unsigned char tx_d[8]; // One CAN Message
        long int olen=0;
        buffer_append_float16(tx_d, st->pos, OR_COMM_RT_POS_SCALE,  &olen); // 2
        buffer_append_float16(tx_d, st->vel, OR_COMM_RT_VEL_SCALE,  &olen); // 4
        buffer_append_float16(tx_d, st->trq, OR_COMM_RT_TRQ_SCALE,  &olen); // 6
        buffer_append_uint16 (tx_d, st->word, &olen);                             // 8
        const uint16_t can_id = ((uint16_t)CAN_RT_DATA_UPSTREAM<<8)|(app_get_configuration()->controller_id);
        comm_can_transmit_eid_if(can_id, tx_d, olen, CAN_RT_UPSTREAM_INTF);

        chThdSleepMicroseconds(1000000.0*1.0/rate); // FIXME: Thread loop should run faster and this if() {} should only trigger on select intervals, but for now, all this do is stream so let's sleep the whole thread
      }
      else
        chThdSleepMilliseconds(1000); // FIXME: Fallback when rate is zero, wait for update
    }
    /*time_end = chVTGetSystemTimeX();
    exectime = time_end - time_start;
    if (or_conf.perf_compensateexectime)
      chThdSleepMicroseconds(1000000.0*1.0/rate-ST2US2(exectime));
    else 
      chThdSleepMicroseconds(1000000.0*1.0/rate);
    */
    //time_last = time_now;
  }
}

bool or_process_can_eid(uint32_t id, uint8_t *data, uint8_t len)
{
  // Do not handle messages that are not for us
  if((id&0x00FF) != app_get_configuration()->controller_id)
    return false;
  
  switch((id>>8)&0xFF)
  {
    case CAN_RT_DATA_DOWNSTREAM:
    {
      if(len != 8 || !or_comm.process_rx)
        break;

      // Get the "free" buffer
      or_comm_control_t* ctrl = or_comm.ctrl  == &(or_comm.ctrl1)
                                      ? &(or_comm.ctrl0) 
                                      : &(or_comm.ctrl1);
      long int ilen = 0;
      // Fill in some data from the received packet
      // Ignore commands in INIT state
      if((or_comm.state->word & OR_STATE_MSK) != OR_STATE_INIT)
      {
        ctrl->pos  = buffer_get_float16(data, OR_COMM_RT_POS_SCALE, &ilen); // 2
        ctrl->vel  = buffer_get_float16(data, OR_COMM_RT_VEL_SCALE, &ilen); // 4
        ctrl->trq  = buffer_get_float16(data, OR_COMM_RT_TRQ_SCALE, &ilen); // 6
        ctrl->word = buffer_get_uint16 (data, &ilen);                             // 8
        // Activate
        or_comm.ctrl_prev = or_comm.ctrl;
        or_comm.ctrl = ctrl; // Swap ! //TODO: keep or not?
        or_comm.last_update = chVTGetSystemTimeX();
        return true;
      } else {
        break;
      }
    }
    case CAN_AUX_DATA_DOWNSTREAM:
    {
      if(len != 2 || !or_comm.process_rx)
        break;
      long int ilen = 0;
      // Okay let's do it right here for now...
      float servo_pos  = buffer_get_float16(data, OR_COMM_AUX_SERVO_SCALE, &ilen); // 2
      if(or_comm.process_ctrl)
        // Ignore commands in INIT state
        if((or_comm.state->word & OR_STATE_MSK) != OR_STATE_INIT)
        {
          pwm_servo_set_servo_out(servo_pos + or_conf.servo_offset);
        }
    }
    default:
      break;
  }
  return false;
}

void or_process_custom_app_data(unsigned char *rx_d, unsigned int len)
{
  (void)rx_d; (void)len;
  /*
  Moved to CAN with custom IDs for Upstream/Downstream
  // RX
  const size_t isize = sizeof(or_comm_control_t)+2;
  if(len == isize && rx_d[0] == 0x70)
  {
    // Get the "free" buffer
    or_comm_control_t* ctrl = or_comm.ctrl  = or_comm.ctrl  == &(or_comm.ctrl1)
                                  ? &(or_comm.ctrl0) 
                                  : &(or_comm.ctrl1);
    long int ilen = 2;
    // Fill in some data from the received packet
    ctrl->word = buffer_get_uint16      (rx_d, &ilen);
    ctrl->pos  = buffer_get_float16(rx_d, OR_COMM_POS_SCALE, &ilen);
    ctrl->vel  = buffer_get_float16(rx_d, OR_COMM_VEL_SCALE, &ilen);
    ctrl->trq  = buffer_get_float16(rx_d, OR_COMM_TRQ_SCALE, &ilen);
    // Activate
    or_comm.ctrl = ctrl; // Swap !
  }

  // TX
  const size_t osize = sizeof(or_comm_state_t)+2; 
  unsigned char tx_d[osize];
  long int olen=2;
  tx_d[0] = 0x12;
  tx_d[1] = 0x45;
  // Get the current buffer
  or_comm_state_t* st = or_comm.state;
  // Copy the data to the send buffer
  buffer_append_uint16      (tx_d, st->word, &olen); // 16
  buffer_append_float16(tx_d, st->pos, OR_COMM_POS_SCALE,  &olen); // 32
  buffer_append_float16(tx_d, st->vel, OR_COMM_VEL_SCALE,  &olen); // 48
  buffer_append_float16(tx_d, st->trq, OR_COMM_TRQ_SCALE,  &olen); // 64
  //buffer_append_float32_auto(tx_d, st->temp, &olen);
  //buffer_append_float32_auto(tx_d, st->curr, &olen);
  commands_send_app_data(tx_d, osize);
  */
}

void or_process_custom_hw_data(unsigned char *rx_d, unsigned int len)
{
  (void)rx_d; (void)len;

  /*
  unsigned int olen=0;
  unsigned char tx_d[256];
  tx_d[0] = 0x98;
  tx_d[1] = 0x42;
  olen = 2;
  if(olen)
    commands_send_hw_data(tx_d, olen);
  */
}


bool or_process_can_sid(uint32_t id, uint8_t *data, uint8_t len)
{
  (void)id; (void)data; (void)len;
  /*
  int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_uint32(buffer, 0x12345678, &send_index);
  if(id == 0x179)
  {
    // RX: can0       179   [8]  11 22 33 44 55 66 77 88
    // TX: can0       179   [4]  12 34 56 78
    if(len > 0 && data[0] == 0x11)
	    comm_can_transmit_sid(id, buffer, send_index);
    // RX: can0       179   [8]  22 33 44 55 66 77 88 99
    // TX: can0  00000179   [4]  12 34 56 78
    else if(len > 0 && data[0] == 0x22)
	    comm_can_transmit_eid(id, buffer, send_index);
    // RX: can0       179   [8]  33 44 55 66 77 88 99 00
    // TX: can0  00000879   [6]  16 00 12 34 56 78
    else if(len > 0 && data[0] == 0x33)
	    comm_can_send_buffer(id, buffer, send_index, 0);
    return true;
  }
  */
  return false;
}



// Meh, not pretty but the toolchain makes it so
#include "_gen/orthopus_confparser.c"
#include "_gen/orthopus_confxml.c"