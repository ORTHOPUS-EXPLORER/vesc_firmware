#include "app_orthopus.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***
#include "mc_interface.h"
#include <math.h> //for atanf function and fmodf function

volatile bool orthopus_thread_stop = true;
volatile bool orthopus_thread_running = false;

volatile or_state_t or_state =
{
  .pos_multiturn_now = 0.0,
  .speed_now = 0.0,
  .enc_pos   = 0.0,
  .adc3_zero = 0.0,
  .turn_now = 0,
  .encoders_init = false,
  // Communication values in internal units (degrees/RPM/N.m) - used directly as setpoints
  .ext_pos_setpoint_deg = 0.0,
  .ext_vel_setpoint_rpm = 0.0,
  .ext_torque_setpoint = 0.0,
  .ext_prev_torque_setpoint = 0.0,
  // Previous setpoint values for change detection
  .ext_prev_pos_setpoint_deg = 0.0,
  .ext_prev_vel_setpoint_rpm = 0.0,
  .prev_control_word = OR_CTRL_MODE_OFF,
  // Internal state management
  .safety_mode = OR_STATE_INIT,
  .control_mode = OR_CTRL_MODE_OFF,
  .last_cmd_time = 0,
  .terminal_timeout_disable = false
};

int get_fw_version_cnt;
float enc_pos_last = 0.0;
unsigned long int nsample = 0; //sample number - used in plots
float pid_pos_now = 0;
float pid_pos_last = 0;
systime_t time_now, time_last, time_start, time_end;
int ninitadc = 0; //number of ADC samples used to compute torque zero
bool or_active_errors[ERR_COUNT] = { false }; //array tracking all errors state
bool or_error_triggered[ERR_COUNT] = { false }; // true = triggered at least once since startup/reset
bool hold_initialized = false;
bool release_on_enable = false;
float hold_position = 0.0;
bool pos_input_shaper_was_enabled = false; // Track previous input shaper state for buffer warm-up

THD_FUNCTION(orthopus_thread, arg) 
{
	(void)arg;

	chRegSetThreadName("OrthopusTh");

  size_t encoder_wait=100; // Increased wait time (100ms instead of 10ms)
  do
  {
    enc_as504x_routine(&encoder_cfg_as504x);
    chThdSleepMilliseconds(1);
    if(encoder_wait && !(--encoder_wait))
      break;
  } while(!encoder_cfg_as504x.state.sensor_diag.is_connected);

  // Check if encoder is still not connected after timeout
  if (!encoder_cfg_as504x.state.sensor_diag.is_connected && !or_conf.simu_mode) {
    // Raise error but allow system to continue in safe state
    or_raise_error(ERR_ENC_DISCONNECTED);
  }

  float v = 0;
  or_set_joint_offset(v, false);

  pid_pos_now = mc_interface_get_pid_pos_now();
  pid_pos_last = pid_pos_now;
  if ( pid_pos_now > 180.0)
  {
    or_state.turn_now = -1;
  }

  if (or_read_encoder() > 180.0)
  {
    or_state.enc_turn = -1;
  }
  get_fw_version_cnt = 0;
  or_state.perf_max_period = -1; //init max period
  or_state.perf_min_period = -1; //init min period
  orthopus_thread_running = true;
  time_now = chVTGetSystemTimeX();
  time_last = time_now;
  //check if torquezero set in config
  if (or_conf.ctrl_torquezero != 0.0 && or_conf.signature == ORTHOPUS_CONFIG_T_SIGNATURE)
  {
      or_state.adc3_init = true;
      or_state.adc3_zero = or_conf.ctrl_torquezero;
  }
  
  // Initialize input shaper variables
  or_state.input_shaper_delay_samples = 0;
  or_state.input_shaper_A1 = or_conf.input_shaper_A1;
  // Clear all delay buffers
  for (int i = 0; i < 1000; i++) {
    or_state.input_shaper_buffer_pos[i] = 0.0f;
    or_state.input_shaper_buffer_vel[i] = 0.0f;
    or_state.input_shaper_buffer_trq[i] = 0.0f;
  }
  /* -------------------------------------------------------------------------- */
  /*                                  MAIN LOOP                                 */
  /* -------------------------------------------------------------------------- */
	for(;;)
  {
    time_start = chVTGetSystemTimeX();
		// Check if it is time to stop.
		if (orthopus_thread_stop) 
    {
			orthopus_thread_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

    // Read AMS
    enc_as504x_routine(&encoder_cfg_as504x);

    or_state.enc_pos = or_read_encoder();

    /* ------------------------------- Count turns ------------------------------ */
    //Output encoder
    if (or_state.enc_pos - enc_pos_last < -350.0)
      ++or_state.enc_turn;
    else if (or_state.enc_pos - enc_pos_last > 350.0)
      --or_state.enc_turn;
    or_state.enc_pos_multiturn = or_conf.encoder_filter_const*(or_state.enc_pos
                                              + 360.0*or_state.enc_turn)
                                        + (1-or_conf.encoder_filter_const)*or_state.enc_pos_multiturn;
    enc_pos_last = or_state.enc_pos;
    //Input encoder
    pid_pos_now = mc_interface_get_pid_pos_now();
    if (!or_state.encoders_init)
    {
      pid_pos_last = pid_pos_now;
      or_state.encoders_init = true;
    }
    if (pid_pos_now - pid_pos_last < -350.0)
      ++or_state.turn_now;
    else if (pid_pos_now - pid_pos_last > 350.0)
      --or_state.turn_now;
    or_state.pos_multiturn_now = pid_pos_now + 360.0*or_state.turn_now;
    pid_pos_last = pid_pos_now;

    // Calculate rotor position: multiply by gear ratio and convert to 0-360°
    float rotor_pos_raw = pid_pos_now * mc_interface_get_configuration()->si_gear_ratio;
    or_state.rotor_pos_now = fmodf(rotor_pos_raw, 360.0f);
    if (or_state.rotor_pos_now < 0.0f) {
        or_state.rotor_pos_now += 360.0f;
    }

    /* ---------------------------- Sample adc3 value --------------------------- */ //TODO: always sample at high freq
    if (or_conf.ctrl_sample_adc3)
    {
      or_state.adc3_val = or_state.adc3_filt;        //get hi freq sampled value
    }
    else
    {
      or_state.adc3_val = ADC_VOLTS(ADC_IND_EXT3);               //get adc value
    }

    /* ---------------------------- Init zero torque ---------------------------- */
    if (!or_state.adc3_init) //init ADC Zero
    {
      ++ninitadc;
      or_state.adc3_zero += or_state.adc3_val/500.0;
      if (ninitadc == 500)
      {
        or_state.adc3_init = true;
        ninitadc = 0;
        or_conf.ctrl_torquezero = or_state.adc3_zero;
        mc_interface_release_motor();
      }
    } 
    else 
    {
      /* ------------------ Scaling ADC3 (volts) -> Torque (N.m) ------------------ */
      if (or_conf.ctrl_sample_adc3)
      {
        or_state.torque_now = or_conf.ctrl_torquegain
                              * (or_state.adc3_filt-or_state.adc3_zero); 
      } else {
        or_state.torque_now = (1-or_conf.torque_filter_const)
                              * or_state.torque_now
                            + or_conf.torque_filter_const
                              * or_conf.ctrl_torquegain
                              * (or_state.adc3_val-or_state.adc3_zero);
      }

      // Apply motor direction inversion to torque
      const volatile mc_configuration *mcconf = mc_interface_get_configuration();
      if (mcconf->m_invert_direction) {
        or_state.torque_now = -or_state.torque_now;
      }

      /* ------------------------------ Control plot ------------------------------ */
      if (or_state.ctrl_plot){
        ++nsample;
        or_plot_impedance(nsample);
      }

      /* ------------------------------ Safety / modes switch --------------------- */

      if (!or_safety())
      {
        //if orthopus_safety failed to handle modes, fallback to ESTOP (should never happend)
        or_set_safety_mode(OR_STATE_ESTOP);
      } 
      else 
      {
        /* -------------------------------------------------------------------------- */
        /*                              Main control loop                             */
        /* -------------------------------------------------------------------------- */

        or_set_safety_mode(or_evaluate_safety_state());

        if(or_comm.process_ctrl && !or_conf.simu_mode) //TODO: deal with those cases properly
        {
          // Store previous values for change detection
          or_state.ext_prev_pos_setpoint_deg = or_state.ext_pos_setpoint_deg;
          or_state.ext_prev_vel_setpoint_rpm = or_state.ext_vel_setpoint_rpm;
          or_state.ext_prev_torque_setpoint = or_state.ext_torque_setpoint;
          or_state.prev_control_word = or_state.control_mode;
          
          // Update or_state setpoints once to avoid repeated conversions
          or_state.ext_pos_setpoint_deg = RAD2DEG_f(or_comm.ctrl->pos);
          or_state.ext_vel_setpoint_rpm = RADPS2RPM_f(or_comm.ctrl->vel);
          or_state.ext_torque_setpoint = or_comm.ctrl->trq;
          
          // Update internal state management from communication
          or_state.control_mode = or_comm.ctrl->word & OR_CTRL_MODE_MSK;
          or_state.last_cmd_time = or_comm.last_update;
          
          // Note: Use or_state.pos_multiturn_now directly as it's already in degrees
          switch(or_state.safety_mode)
          {
            case OR_STATE_INIT:
            {
              hold_initialized = false;
              break;
            }
            case OR_STATE_IDLE:
            {
              mc_interface_release_motor();
              hold_initialized = false;
              break;
            }
            case OR_STATE_ENABLE:
            {
              // Modes are currently exclusive. Refactor this switch for mode fine-grained mode control
              switch(or_state.control_mode) 
              {
                case OR_CTRL_MODE_POS:
                { 

                  // TODO: tunable max position error
                  if ((fabs(fmod((or_state.ext_pos_setpoint_deg - or_state.pos_multiturn_now + 540),360) - 180) >= (double)or_conf.safety_max_q_error) && !or_conf.safety_track_disable)
                  {
                    or_raise_error(ERR_POS_STEP); // Set error flag
                  } else {
                    if (!or_active_errors[ERR_POS_STEP] || or_conf.auto_clear_errors) //if 
                    {
                      // Apply input shaping to position setpoint
                      float shaped_position = or_input_shaper_pos(or_state.ext_pos_setpoint_deg);
                      mc_interface_set_pid_pos(shaped_position);
                    }

                    if (or_conf.auto_clear_errors)
                    {
                      or_clear_error(ERR_POS_STEP); // Clear error
                    }
                  }
                  break;
                }
                case OR_CTRL_MODE_VEL:
                {
                  if (((ST2US2(chVTGetSystemTimeX() - or_state.last_cmd_time) > 10000) && or_state.ext_vel_setpoint_rpm != 0.0) && !or_conf.safety_timeout_disable && !or_state.terminal_timeout_disable) //raise error if command does not ensure at least 100Hz
                  {
                    or_raise_error(ERR_CAN_TIMEOUT);
                    break; // Stop executing velocity control when timeout occurs
                  }
                  // Apply input shaping to velocity setpoint
                  float shaped_velocity = or_input_shaper_vel(or_state.ext_vel_setpoint_rpm);
                  mc_interface_set_pid_speed(mc_interface_get_configuration()->p_pid_ang_div*shaped_velocity);
                  break;
                }
                case OR_CTRL_MODE_TRQ :
                case OR_CTRL_MODE_IMP : //Impedance mode: available for later
                {
                  if (((ST2US2(chVTGetSystemTimeX() - or_state.last_cmd_time) > 10000) && !or_conf.safety_timeout_disable && !or_state.terminal_timeout_disable)) //raise error if command does not ensure at least 100Hz
                  {
                    or_raise_error(ERR_CAN_TIMEOUT);
                    break; // Stop executing torque control when timeout occurs
                  }
                  // Check if torque step error is active
                  if (or_active_errors[ERR_TRQ_STEP])
                  {
                    or_state.ext_torque_setpoint = 0.0;
                  }
                  else
                  {
                    // Torque setpoint is already set from communication values
                    if (or_conf.limits_enable_reaction && or_conf.limits_enable)
                      or_limits_reaction();
                    or_interface_torquecontrol(); // Difference between torque and impedance control is handled inside this function
                  }
                  break;
                }
                case OR_CTRL_MODE_CST : //Cusom mode: TODO
                {
                  if ((ST2US2(chVTGetSystemTimeX() - or_state.last_cmd_time) > 10000) && !or_conf.safety_timeout_disable && !or_state.terminal_timeout_disable) //raise error if command does not ensure at least 100Hz
                  {
                    or_raise_error(ERR_CAN_TIMEOUT);
                    break; // Stop executing custom control when timeout occurs
                  }
                  // Position, velocity, and torque setpoints are already set from communication values
                  break;
                }
                case OR_CTRL_MODE_OFF:
                {
                  if (release_on_enable)
                  {
                    mc_interface_release_motor();
                    release_on_enable = false;
                  }
                  break;
                }
                default:
                  break;
              }
              hold_initialized = false;
              break;
            }
            case OR_STATE_HOLD:
            {
              // Initialize hold by getting the current position as hold position
              if (!hold_initialized)
              {
                hold_position = or_state.pos_multiturn_now;
                hold_initialized = true;
              }

              // compute the actual error
              float error = fabs(fmod((hold_position - or_state.pos_multiturn_now + 540), 360) - 180);

              // check the position error
              if (error >= or_conf.safety_max_q_error)
              {
                or_raise_error(ERR_STP_HOLD);
              }
              else
              {
                mc_interface_set_pid_pos(hold_position);

                // if error was previously set and auto_clear is active,
                // check if current position setpoint error is acceptable
                float pos_error = fabs(fmod((or_state.ext_pos_setpoint_deg - or_state.pos_multiturn_now + 540), 360) - 180);

                if (or_conf.auto_clear_errors && or_active_errors[ERR_POS_STEP] &&
                    pos_error < 0.1*or_conf.safety_max_q_error && or_state.control_mode == OR_CTRL_MODE_POS) //reset if auto_reset AND error is less than 10% the max allowed value. TODO: better definition of the treshold?
                {
                  or_clear_error(ERR_POS_STEP);
                  or_set_safety_mode(OR_STATE_ENABLE);
                }
              }

              break;
            }
            case OR_STATE_BRAKE:
            {
              mc_interface_set_brake_current(3);
              hold_initialized = false;
              break;
            }
            case OR_STATE_ESTOP:
            {
              or_estop();
              hold_initialized = false;
              break;
            }
            default:
            {
              or_estop();
              hold_initialized = false;
              break;
            }
          }
        }
      }
    }
    
    if(!or_conf.simu_mode)
    {
      // Update communication state from internal state
      // Convert to radians and wrap to [-π, π] range for consistent CAN communication
      // This ensures both firmware and PC use the same angular representation
      float pid_pos_rad = DEG2RAD_f(mc_interface_get_pid_pos_now());
      
      // Wrap to [-π, π] range using the same method as PC side
      pid_pos_rad = fmodf(pid_pos_rad + DEG2RAD_f(180.0f), DEG2RAD_f(360.0f)) - DEG2RAD_f(180.0f);
      
      or_comm.state->pos = pid_pos_rad;
      or_comm.state->vel = RPM2RADPS_f(or_state.speed_now);
      or_comm.state->trq = or_state.torque_now;
      
      // Sync internal safety and control modes to communication state
      or_comm.state->word &= ~OR_STATE_MSK; // Clear safety bits
      or_comm.state->word |= or_state.safety_mode; // Set current safety mode
      
      // Clear and set control mode bits in state word (for output)
      or_comm.state->word &= ~OR_CTRL_MODE_MSK; // Clear mode bits
      switch(or_state.control_mode) {
        case OR_CTRL_MODE_POS: or_comm.state->word |= OR_CTRL_MODE_POS; break;
        case OR_CTRL_MODE_VEL: or_comm.state->word |= OR_CTRL_MODE_VEL; break;
        case OR_CTRL_MODE_TRQ: or_comm.state->word |= OR_CTRL_MODE_TRQ; break;
        case OR_CTRL_MODE_IMP: or_comm.state->word |= OR_CTRL_MODE_IMP; break;
        case OR_CTRL_MODE_CST: or_comm.state->word |= OR_CTRL_MODE_CST; break;
        case OR_CTRL_MODE_OFF: 
        default: break; // OFF mode doesn't set any state mode bits
      }
    }
    
/* -------------------------------------------------------------------------- */
/*                          Execution time management                         */
/* -------------------------------------------------------------------------- */

/* ---------------------------- Performances plot --------------------------- */
    if (or_state.perf_plot){
      ++nsample;
      or_plot_cycletime(nsample);
    }
    time_now = chVTGetSystemTimeX();
    or_state.time_diff = ST2US2(time_now - time_last);
    or_state.time_diff_filt = 0.99*or_state.time_diff_filt
                                  + 0.01*(or_state.time_diff
                                  + or_state.time_lag_compensation);
    or_state.time_lag_filt = or_state.time_diff_filt
                           - 1000000.0*1.0/or_conf.perf_rate_hz;
    if (or_state.time_diff > or_state.perf_max_period)
      or_state.perf_max_period = or_state.time_diff;
    if (or_state.time_diff < or_state.perf_min_period)
      or_state.perf_min_period = or_state.time_diff;
    time_end = chVTGetSystemTimeX();
    or_state.perf_exec_time = time_end - time_start;
    /* ------ Loop time compensation activated - sleep compensated duration ----- */
    if (or_conf.perf_compensateexectime)
      chThdSleepMicroseconds(1000000.0*1.0/or_conf.perf_rate_hz
                                              -ST2US2(or_state.perf_exec_time));
    /* --------------------- Non compensated sleep (1/rate) --------------------- */
    else 
      chThdSleepMicroseconds(1000000.0*1.0/or_conf.perf_rate_hz);
    time_last = time_now;
	}
}

// Called in mc_interface.c:1913, in mc_interface_mc_timer_isr()
void or_pwm_callback(void)
{
	// Called for every control iteration in interrupt context.
  //Sample torque sensor ADC at high frequency
  or_state.adc3_filt = or_conf.torque_filter_const*ADC_VOLTS(ADC_IND_EXT3)
                     + (1-or_conf.torque_filter_const)*or_state.adc3_filt;
  //Sample and filter speed at high frequency
  or_state.speed_now = or_conf.speed_filter_const*(mc_interface_get_rpm() / mc_interface_get_configuration()->p_pid_ang_div)
                     + (1-or_conf.speed_filter_const)*or_state.speed_now;
}

/**
 * Torque / impedance control
 *
 * @param
 * void
 *
 * @return
 * void
 */
void or_interface_torquecontrol(void)
{
/* -------------------------------------------------------------------------- */
/*                              Impedance control                             */
/* -------------------------------------------------------------------------- */

  or_state.stopped = false;

  or_state.torque_err = or_state.ext_torque_setpoint
                        - or_state.torque_now;

  if(or_state.control_mode == OR_CTRL_MODE_IMP)
  {
    if ((fabs(fmod((or_state.ext_pos_setpoint_deg - or_state.pos_multiturn_now + 540),360) - 180) >= (double)or_conf.safety_max_q_error) && !or_conf.safety_track_disable)
    {
      or_raise_error(ERR_POS_STEP); // Set error flag
    } 

    //add position action
    or_state.torque_err += or_conf.ctrl_stiffness * M_PI / 180.0 // convert N.m/rad to N.m/deg
                          * (or_state.ext_pos_setpoint_deg - or_state.pos_multiturn_now);
    // add velocity action
    or_state.torque_err += or_conf.ctrl_damping * M_PI / 30.0 // convert N.m/(rad/s) to N.m/RPM
                          * (or_state.ext_vel_setpoint_rpm - or_state.speed_now);
  }

  //add limits action
  or_state.torque_err += or_state.limit_reaction;

  //compute torque error derivative
  or_state.d_torque_err = (or_state.torque_err-or_state.torque_err_last)
                          / (1.0/or_conf.perf_rate_hz);
  or_state.d_torque_err_filt = or_conf.ctrl_kd_filter
                                * or_state.d_torque_err
                              + (1-or_conf.ctrl_kd_filter)
                                * or_state.d_torque_err_filt;
  or_state.torque_err_last = or_state.torque_err;
  if (or_conf.ctrl_deadzone)
  {
    or_state.ctrl_command = or_conf.ctrl_kp * or_state.torque_err
                          + or_conf.ctrl_kd * or_state.d_torque_err;
    or_state.ctrl_command = or_state.ctrl_command
                          - atanf(or_state.ctrl_command*or_conf.ctrl_a)
                            / or_conf.ctrl_a;
  } else {
    or_state.ctrl_command = or_conf.ctrl_kp * (or_state.torque_err)
                          + or_conf.ctrl_kd*or_state.d_torque_err;
  }
  
  // add feedforward term
  if (or_conf.ff_torque_constant != 0.0)
    or_state.ctrl_command += or_state.ext_torque_setpoint / or_conf.ff_torque_constant;

  /* ------------------------ Compute safety indicators ----------------------- */
  if ((or_state.last_ctrl_command == or_state.ctrl_command)
                                      &&
                                      (or_state.ctrl_command!=0.0))
  {
    or_state.nid1 += 1;
  } 
  else 
  {
    or_state.nid1 = 0;
  }
  or_state.last_ctrl_command = or_state.ctrl_command;
  /* -------------------------------------------------------------------------- */
  /*                    Send current setpoint to mc_interface                   */
  /* -------------------------------------------------------------------------- */
  mc_interface_set_current_off_delay(0.1);  //prevent disabling motor if 
                      //torque request is 0 //todo: move somewhere else?
  
  // Apply input shaping to the control command
  float shaped_command = or_input_shaper_trq(or_state.ctrl_command);
  mc_interface_set_current(shaped_command);
}

/**
 * Emergency stop of the actuator
 *
 * @param
 * void
 *
 * @return
 * void
 */
void or_estop(void)
{
  mc_interface_release_motor();   //disable motor
  mc_interface_ignore_input(100);  //disable new inputs for 100 ms
  or_state.ctrl_command = 0.0;
  or_state.torque_err = 0.0;
  or_state.stopped = true;
}

/**
 * Safety: check control variables looking for errors, stop if anomaly detected 
 *
 * @param
 * void
 *
 * @return
 * bool (true: modes set correctly, false: failed to set modes)
 */
bool or_safety(void)
{
  // All values are now tracked in internal state - no need for or_comm access

  // ------------------ new command received: ------------------------//

  // Check for changes by comparing current setpoints with previous values stored in or_state
  bool command_changed = (or_state.ext_pos_setpoint_deg != or_state.ext_prev_pos_setpoint_deg) ||
                        (or_state.ext_vel_setpoint_rpm != or_state.ext_prev_vel_setpoint_rpm) ||
                        (or_state.ext_torque_setpoint != or_state.ext_prev_torque_setpoint) ||
                        (or_state.control_mode != or_state.prev_control_word);

  if (command_changed)
  {
    if (or_state.control_mode != or_state.prev_control_word) //detect change of control word
    {
      //TODO: manage modes switch
      //clear error if word goes from anything to 0x0000
      if (or_state.control_mode == OR_CTRL_MODE_OFF)
      {
        or_clear_error(ERR_POS_STEP); // Clear error //TODO: manage error clear
        if (or_state.ext_torque_setpoint == 0.0)
          or_clear_error(ERR_TRQ_STEP); // Clear error
        if (or_state.ext_vel_setpoint_rpm == 0.0)
          or_clear_error(ERR_VEL_STEP); // Clear error
        
        //setting control word OFF resets the safety mode to ENABLE if not critical
        if (or_state.safety_mode <= OR_STATE_HOLD)
        {
          or_set_safety_mode(OR_STATE_ENABLE);
        }

        mc_interface_release_motor(); //release motor last command -> allows control from VESC
      }
    }
    
    if ((fabsf(or_state.ext_torque_setpoint - or_state.ext_prev_torque_setpoint) > or_conf.safety_max_trq_step) && or_conf.safety_max_trq_step > 0.0)
    {
      or_raise_error(ERR_TRQ_STEP); // Set error flag
    }

    if ((fabsf(or_state.ext_vel_setpoint_rpm - or_state.ext_prev_vel_setpoint_rpm) > or_conf.safety_max_vel_step) && or_conf.safety_max_vel_step > 0.0)
    {
      or_raise_error(ERR_VEL_STEP); // Set error flag
    }
  } 

  // ------------- check other indicators ----------------//
  if (or_state.nid1 > 50 ) {
    or_raise_error(ERR_SAME_CTRL_OUT);
    or_state.nid1 = 0;
  } 

  if ((fabs(or_state.speed_now) > (double)or_conf.safety_max_speed) && !or_conf.safety_max_speed_disable) //check speed limit
  {
    or_raise_error(ERR_MAX_SPEED);
  }

  // ------------------- limits --------------------------//
  if (or_conf.limits_enable)
  {
    if (   (or_state.pos_multiturn_now > or_conf.limits_pos_max)
        || (or_state.pos_multiturn_now < or_conf.limits_pos_min)) //out of limits
    {
      or_raise_error(ERR_POS_LIMIT);
    }
    else if ( (
              (or_state.speed_now > or_conf.limits_reach_speed
                && (or_state.pos_multiturn_now 
                    > or_conf.limits_pos_max - or_conf.limits_reach_angle)
              )
              ||
              ( or_state.speed_now < -or_conf.limits_reach_speed
                && (or_state.pos_multiturn_now 
                    < or_conf.limits_pos_min + or_conf.limits_reach_angle)
              )
              )
            ) //reaching limit too fast
    {
      or_raise_error(ERR_SPEED_LIMIT);
    }
  }

  // -------------- sync errors in state word ----------------- //
  or_sync_error_flags();

  return true;
}

/**
 * Limits reaction: compute a torque setpoint to simulate a physical end stop
 * (position, speed, etc.)
 *
 * @param
 * void
 *
 * @return
 * void
 */
void or_limits_reaction(void)
{
  //check position limits
  or_state.limit_reaction = 0;
  if ( (or_state.pos_multiturn_now 
                         < or_conf.limits_pos_min + or_conf.limits_reach_angle))
  {
/* --------------------------- Reaching min limit --------------------------- */
    or_state.limit_reaction =
        or_conf.limits_kp
        * powf(or_state.pos_multiturn_now
               - or_conf.limits_pos_min
               - or_conf.limits_reach_angle
          ,or_conf.limits_powp);
  }
  if ( (or_state.pos_multiturn_now
                         > or_conf.limits_pos_max - or_conf.limits_reach_angle))
  { 
/* --------------------------- Reaching max limit --------------------------- */
    or_state.limit_reaction =
        -or_conf.limits_kp
        * powf(or_state.pos_multiturn_now
               - or_conf.limits_pos_max
               + or_conf.limits_reach_angle
          ,or_conf.limits_powp);
  }
  if ( (or_state.pos_multiturn_now
          < or_conf.limits_pos_min
            + or_conf.limits_reach_angle
            + or_conf.limits_damp_reachangle)
        && (or_state.speed_now < 0)) //add damping, only in the direction of the
                                                //limit to avoid sticking effect
  {
/* -------------------- Damping before reaching min limit ------------------- */
    or_state.limit_reaction += or_conf.limits_kd
                               *powf(fabs(or_state.speed_now),or_conf.limits_powd);
  }
  if ( (or_state.pos_multiturn_now
          > or_conf.limits_pos_max
            - or_conf.limits_reach_angle
            - or_conf.limits_damp_reachangle)
        && (or_state.speed_now > 0)) //add damping, only in the direction of the
                                                //limit to avoid sticking effect
  {
/* -------------------- Damping before reaching min limit ------------------- */
    or_state.limit_reaction += -or_conf.limits_kd
                               *powf(fabs(or_state.speed_now),or_conf.limits_powd);
  }
}

/**
 * For debug - plot raw, filtered and past variables of encoder filter
 *
 * @param ns
 * Sample number
 *
 * @return
 * void
 */
void or_plot_encoder_filtering(int ns)
{
  bool plot_started=true;
  if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
    get_fw_version_cnt = commands_get_fw_version_sent_cnt();
    plot_started = false;
  }
  if (!plot_started) {
    plot_started = true;
    commands_init_plot("time", "angle");
    commands_plot_add_graph("enc_pos");
    commands_plot_add_graph("enc_pos");
    commands_plot_add_graph("last_nb_enc_filter_error");
    commands_plot_add_graph("enc_pos_last");
  }
  commands_plot_set_graph(0);
  commands_send_plot_points(ns, or_state.enc_pos);
  commands_plot_set_graph(1);
  commands_send_plot_points(ns, or_state.enc_pos);
  commands_plot_set_graph(2);
  commands_send_plot_points(ns, enc_pos_last);
}

/**
 * For debug - plot loop cycle time
 *
 * @param ns
 * Sample number
 *
 * @return
 * void
 */
void or_plot_cycletime(int ns)
{
  bool plot_started=true;
  if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
    get_fw_version_cnt = commands_get_fw_version_sent_cnt();
    plot_started = false;
  }
  if (!plot_started) {
    plot_started = true;
    commands_init_plot("sample", "cycletime");
    commands_plot_add_graph("cycletime");
    commands_plot_add_graph("time_diff_filt");
  }
  commands_plot_set_graph(0);
  commands_send_plot_points(ns, or_state.time_diff*1.0);
  commands_plot_set_graph(1);
  commands_send_plot_points(ns, or_state.time_diff_filt*1.0);
}

/**
 * For debug - plot loop cycle time
 *
 * @param ns
 * Sample number
 *
 * @return
 * void
 */
void or_plot_impedance(int ns)
{
  bool plot_started=true;
  if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
    get_fw_version_cnt = commands_get_fw_version_sent_cnt();
    plot_started = false;
  }
  if (!plot_started) {
    plot_started = true;
    commands_init_plot("sample", "V");
    commands_plot_add_graph("ADC3cal");
    commands_plot_add_graph("torque_now");
    commands_plot_add_graph("Controlout");
  }
  commands_plot_set_graph(0);
  commands_send_plot_points(ns, or_state.adc3_val*1.0);
  commands_plot_set_graph(1);
  commands_send_plot_points(ns, or_state.torque_now*1.0);
  commands_plot_set_graph(2);
  commands_send_plot_points(ns, or_state.ctrl_command*1.0);
}

/**
 * @brief Get the severity level of a specific error.
 * 
 * @param err The error identifier.
 * @return The severity level of the error.
 */
or_error_level_t or_get_error_severity(or_error_t err) {
  switch (err) {

    case ERR_TST_WARNING:
      return ERR_LEVEL_WARNING;

    case ERR_POS_STEP:
    case ERR_TRQ_STEP:
    case ERR_VEL_STEP:
    case ERR_SAME_CTRL_OUT:
    case ERR_TST_HOLD:
    case ERR_CAN_TIMEOUT:
      return ERR_LEVEL_HOLD;

    case ERR_TST_BRAKE:
    case ERR_SPEED_LIMIT:
    case ERR_STP_HOLD:
    case ERR_MAX_SPEED:
      return ERR_LEVEL_BRAKE;

    case ERR_TST_ESTOP:
    case ERR_POS_LIMIT:
    case ERR_ENC_DISCONNECTED:
      return ERR_LEVEL_ESTOP;

    default:
      return ERR_LEVEL_ESTOP;
  }
}

/**
 * @brief Compute the maximum severity level among all active errors.
 * 
 * @return The highest severity level of any currently active error.
 */
or_error_level_t or_compute_max_error_level(void) {
  or_error_level_t max_level = ERR_LEVEL_NONE;

  for (int i = 0; i < ERR_COUNT; ++i) {
    if (or_active_errors[i]) {
      or_error_level_t level = or_get_error_severity((or_error_t)i);
      if (level > max_level)
        max_level = level;
    }
  }

  return max_level;
}

/**
 * @brief Set an error as active.
 * 
 * @param err The error to raise.
 */
void or_raise_error(or_error_t err) {
  if (err < ERR_COUNT)
    {
      or_active_errors[err] = true;
      or_error_triggered[err] = true; // permanent trace
    }
}

/**
 * @brief Clear an active error.
 * 
 * @param err The error to clear.
 */
void or_clear_error(or_error_t err) {
  if (err < ERR_COUNT)
    or_active_errors[err] = false;
}


/**
 * @brief Evaluates the appropriate safety level based on current errors.
 *        Only escalation is allowed (no automatic downgrade).
 * 
 * @return The new safety state (e.g. OR_STATE_HOLD)
 */
uint16_t or_evaluate_safety_state(void) {
  uint16_t current_safety_state = or_state.safety_mode;

  // Automatic INIT → IDLE
  if (current_safety_state == OR_STATE_INIT) {
    //Stay in INIT mode 8 seconds (enough to let the actuator boot properly and flush eventual remaining data in CAN buffer)
    if (or_state.encoders_init && ST2S(chVTGetSystemTimeX()) > 8) {
      return OR_STATE_IDLE;
    }
    return OR_STATE_INIT;
  }

  // Automatic IDLE → ENABLE only if no active errors
  if (current_safety_state == OR_STATE_IDLE) {
    if (or_compute_max_error_level() == ERR_LEVEL_NONE) {
      return OR_STATE_ENABLE;
    }
    return OR_STATE_IDLE;
  }

  // Escalation from ENABLE → HOLD → BRAKE → ESTOP
  
  or_error_level_t severity = or_compute_max_error_level(); 
  uint16_t new_safety_state = current_safety_state; //new safety state starts as-is and will be changed if needed
  
  switch (severity)
  {
    case ERR_LEVEL_ESTOP:
      new_safety_state = OR_STATE_ESTOP;
      break;
    case ERR_LEVEL_BRAKE:
      if (current_safety_state < OR_STATE_BRAKE) { //only escalate
        new_safety_state = OR_STATE_BRAKE;
      }
      break;
    case ERR_LEVEL_HOLD:
      if (current_safety_state < OR_STATE_HOLD) { //only escalate
        new_safety_state = OR_STATE_HOLD;
      } //TODO: allow returning to ENABLE / Position control?
      break;
    case ERR_LEVEL_WARNING: //do not change safety state
      break;
    case ERR_LEVEL_NONE: //do not change safety state
      break;
    case ERR_LEVEL_COUNT:
      break;
  }

  return new_safety_state;
}

/**
 * @brief Set the current safety mode in the orthopus state word.
 *
 * This function clears the existing safety mode bits and sets the new mode
 * using the defined OR_STATE_MSK. It ensures mutually exclusive safety states.
 *
 * @param mode  The new safety mode to apply (e.g. OR_STATE_ENABLE).
 */
void or_set_safety_mode(uint32_t mode)
{
  //check if we are switching to ENABLE and trigger release if so
  if (mode == OR_STATE_ENABLE && or_state.control_mode == OR_CTRL_MODE_OFF && or_state.safety_mode != OR_STATE_ENABLE)
  {
    release_on_enable = true;
  }
  or_state.safety_mode = mode;
}

/**
 * @brief Set the current control mode in the orthopus control word.
 *
 * This function clears the existing control mode bits and sets the new mode
 * using the defined OR_CTRL_MODE_MSK. It sets only one control mode at a time.
 * Also updates the communication structure to ensure synchronization.
 * When called from terminal commands, disables timeout checking.
 *
 * @param mode  The new control mode to apply (e.g. OR_CTRL_MODE_POS).
 */
void or_set_control_mode(uint32_t mode)
{
  or_state.control_mode = mode;
  // Update communication control word to maintain synchronization
  or_comm.ctrl->word = (or_comm.ctrl->word & ~OR_CTRL_MODE_MSK) | mode;
  // Disable timeout checking for terminal-initiated commands (except OFF mode)
  or_state.terminal_timeout_disable = (mode != OR_CTRL_MODE_OFF);
}

/**
 * @brief Synchronize active error flags with the orthopus state word.
 *
 * This function sets or clears error bits in or_comm.state->word
 * based on the content of or_active_errors[]. It ensures that the state
 * word reflects the current error status precisely.
 */
void or_sync_error_flags(void)
{
  // Clear all error bits managed here
  or_comm.state->word &= ~(OR_STATE_ERR_POS_STEP |
                                 OR_STATE_ERR_VEL_STEP |
                                 OR_STATE_ERR_TRQ_STEP);

  // Set error bits based on active error status
  if (or_active_errors[ERR_POS_STEP])
    or_comm.state->word |= OR_STATE_ERR_POS_STEP;

  if (or_active_errors[ERR_VEL_STEP])
    or_comm.state->word |= OR_STATE_ERR_VEL_STEP;

  if (or_active_errors[ERR_TRQ_STEP])
    or_comm.state->word |= OR_STATE_ERR_TRQ_STEP;

  // Check for any other active errors
  for (int i = 0; i < ERR_COUNT; ++i) {
    if (i != ERR_POS_STEP && i != ERR_VEL_STEP && i != ERR_TRQ_STEP && or_active_errors[i]) {
      or_comm.state->word |= OR_STATE_ERR_OTHER;
      break;
    }
  }
}

/**
 * Input shaper for position control commands
 * 
 * Implements a simple input shaper that splits the command into two parts:
 * - A1 fraction sent immediately
 * - (1-A1) fraction sent after a delay T1
 * 
 * Special handling for angular wraparound to prevent erratic behavior when crossing 0°/360°
 * 
 * @param input_command The raw position command to be shaped
 * @return float - The shaped position command
 */
float or_input_shaper_pos(float input_command)
{
  if (!or_conf.input_shaper_enable) {
    pos_input_shaper_was_enabled = false; // Track that input shaper is disabled
    return input_command; // Pass-through if disabled
  }

  // Check if input shaper was just enabled (transition from disabled to enabled)
  bool just_enabled = !pos_input_shaper_was_enabled && or_conf.input_shaper_enable;
  pos_input_shaper_was_enabled = true; // Update state for next call

  // Update A1 factor from config
  or_state.input_shaper_A1 = or_conf.input_shaper_A1;
  
  // Calculate delay in samples based on configured delay T1 (in ms) and loop rate
  or_state.input_shaper_delay_samples = (int)((or_conf.input_shaper_T1 / 1000.0f) * or_conf.perf_rate_hz);
  
  // Limit delay samples to buffer size
  if (or_state.input_shaper_delay_samples >= 1000) {
    or_state.input_shaper_delay_samples = 999;
  }
  if (or_state.input_shaper_delay_samples < 1) {
    or_state.input_shaper_delay_samples = 1;
  }

  // Warm up buffer if input shaper was just enabled
  // This prevents unwanted movements when switching from disabled to enabled
  if (just_enabled) {
    for (int i = 0; i < 1000; i++) {
      or_state.input_shaper_buffer_pos[i] = input_command;
    }
  }

  // Shift buffer to make room for new value (move all values one position right)
  for (int i = 999; i > 0; i--) {
    or_state.input_shaper_buffer_pos[i] = or_state.input_shaper_buffer_pos[i-1];
  }
  
  // Store current command at index 0 (newest)
  or_state.input_shaper_buffer_pos[0] = input_command;
  
  // Get delayed command (older value from buffer)
  float delayed_command = or_state.input_shaper_buffer_pos[or_state.input_shaper_delay_samples];
  
  // Handle angular wraparound for position commands
  // Calculate the shortest angular distance between current and delayed commands
  float angle_diff = fmodf((input_command - delayed_command + 540.0f), 360.0f) - 180.0f;
  
  // Calculate shaped output using the corrected angular difference
  // This prevents jumps when crossing 0°/360° boundary
  float shaped_command = delayed_command + or_state.input_shaper_A1 * angle_diff;
  
  // Normalize result to keep it in reasonable range
  shaped_command = fmodf(shaped_command, 360.0f);
  if (shaped_command < 0.0f) {
    shaped_command += 360.0f;
  }
  
  return shaped_command;
}

/**
 * Input shaper for velocity control commands
 * 
 * Implements a simple input shaper that splits the command into two parts:
 * - A1 fraction sent immediately
 * - (1-A1) fraction sent after a delay T1
 * 
 * @param input_command The raw velocity command to be shaped
 * @return float - The shaped velocity command
 */
float or_input_shaper_vel(float input_command)
{
  if (!or_conf.input_shaper_enable) {
    return input_command; // Pass-through if disabled
  }

  // Update A1 factor from config
  or_state.input_shaper_A1 = or_conf.input_shaper_A1;
  
  // Calculate delay in samples based on configured delay T1 (in ms) and loop rate
  or_state.input_shaper_delay_samples = (int)((or_conf.input_shaper_T1 / 1000.0f) * or_conf.perf_rate_hz);
  
  // Limit delay samples to buffer size
  if (or_state.input_shaper_delay_samples >= 1000) {
    or_state.input_shaper_delay_samples = 999;
  }
  if (or_state.input_shaper_delay_samples < 1) {
    or_state.input_shaper_delay_samples = 1;
  }

  // Shift buffer to make room for new value (move all values one position right)
  for (int i = 999; i > 0; i--) {
    or_state.input_shaper_buffer_vel[i] = or_state.input_shaper_buffer_vel[i-1];
  }
  
  // Store current command at index 0 (newest)
  or_state.input_shaper_buffer_vel[0] = input_command;
  
  // Get delayed command (older value from buffer)
  float delayed_command = or_state.input_shaper_buffer_vel[or_state.input_shaper_delay_samples];
  
  // Calculate shaped output: A1 * current + (1-A1) * delayed
  float shaped_command = or_state.input_shaper_A1 * input_command + (1.0f - or_state.input_shaper_A1) * delayed_command;
  
  return shaped_command;
}

/**
 * Input shaper for torque control commands
 * 
 * Implements a simple input shaper that splits the command into two parts:
 * - A1 fraction sent immediately
 * - (1-A1) fraction sent after a delay T1
 * 
 * @param input_command The raw torque command to be shaped
 * @return float - The shaped torque command
 */
float or_input_shaper_trq(float input_command)
{
  if (!or_conf.input_shaper_enable) {
    return input_command; // Pass-through if disabled
  }

  // Update A1 factor from config
  or_state.input_shaper_A1 = or_conf.input_shaper_A1;
  
  // Calculate delay in samples based on configured delay T1 (in ms) and loop rate
  or_state.input_shaper_delay_samples = (int)((or_conf.input_shaper_T1 / 1000.0f) * or_conf.perf_rate_hz);
  
  // Limit delay samples to buffer size
  if (or_state.input_shaper_delay_samples >= 1000) {
    or_state.input_shaper_delay_samples = 999;
  }
  if (or_state.input_shaper_delay_samples < 1) {
    or_state.input_shaper_delay_samples = 1;
  }

  // Shift buffer to make room for new value (move all values one position right)
  for (int i = 999; i > 0; i--) {
    or_state.input_shaper_buffer_trq[i] = or_state.input_shaper_buffer_trq[i-1];
  }
  
  // Store current command at index 0 (newest)
  or_state.input_shaper_buffer_trq[0] = input_command;
  
  // Get delayed command (older value from buffer)
  float delayed_command = or_state.input_shaper_buffer_trq[or_state.input_shaper_delay_samples];
  
  // Calculate shaped output: A1 * current + (1-A1) * delayed
  float shaped_command = or_state.input_shaper_A1 * input_command + (1.0f - or_state.input_shaper_A1) * delayed_command;
  
  return shaped_command;
}