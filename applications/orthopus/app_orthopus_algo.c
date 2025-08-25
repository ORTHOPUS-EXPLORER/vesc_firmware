#include "app_orthopus.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***
#include "mc_interface.h"
#include <math.h> //for atanf function and fmodf function

volatile bool orthopus_thread_stop = true;
volatile bool orthopus_thread_running = false;

//const int loop_rate = 2000; //loop rate in Hz

volatile or_state_t or_state =
{
  .pos_multiturn_now = 0.0,
  .speed_now = 0.0,
  .enc_pos   = 0.0,
  .adc3_zero = 0.0,
  .turn_now = 0,
  .ext_torque_setpoint = 0,
  .ext_pos_setpoint = 0,
  .encoders_init = false
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

THD_FUNCTION(orthopus_thread, arg) 
{
	(void)arg;

	chRegSetThreadName("OrthopusTh");

  size_t encoder_wait=10;
  do
  {
    enc_as504x_routine(&encoder_cfg_as504x);
    chThdSleepMilliseconds(1);
    if(encoder_wait && !(--encoder_wait))
      break;
  } while(!encoder_cfg_as504x.state.sensor_diag.is_connected);

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
  // Clear the delay buffer
  for (int i = 0; i < 1000; i++) {
    or_state.input_shaper_buffer[i] = 0.0f;
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
    //TODO: take into account current speed to compare raw value with next 
                                      //expected value instead of previous value

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

    //TODO: speed now computed at higher freq
    //or_state.speed_now = mc_interface_get_rpm() / mc_interface_get_configuration()->p_pid_ang_div;

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
        //TODO: understand why the commands_printf induces fail (disconnect from VESC_tool + bricked controller sometimes)
        /*commands_printf("torque zero done: or_state.adc3_zero: % 7.3f", 
                                                    (double)or_state.adc3_zero);*/
        ninitadc = 0;
        or_conf.ctrl_torquezero = or_state.adc3_zero;
        //commands_printf("save config to store in EEPROM");
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

      /* Apply motor direction inversion to torque if needed */
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
        or_comm.state->word = (or_comm.state->word &= ~OR_SAFETY_MSK) | OR_SAFETY_ESTOP;
      } 
      else 
      {
        /* -------------------------------------------------------------------------- */
        /*                              Main control loop                             */
        /* -------------------------------------------------------------------------- */

        or_set_safety_mode(or_evaluate_safety_state());

        if(or_comm.process_ctrl && !or_conf.simu_mode) //TODO: deal with those cases properly
        {
          switch(or_comm.state->word & OR_SAFETY_MSK)
          {
            case OR_SAFETY_INIT:
            {
              hold_initialized = false;
              break;
            }
            case OR_SAFETY_IDLE:
            {
              mc_interface_release_motor();
              hold_initialized = false;
              break;
            }
            case OR_SAFETY_ENABLE:
            {
              or_comm.state->word &= ~OR_STATE_MODE_MSK;                       // Clear mode
              // Modes are currently exclusive. Refactor this switch for mode fine-grained mode control
              switch(or_comm.ctrl->word & OR_CTRL_MODE_MSK) 
              {
                case OR_CTRL_MODE_POS:
                {
                  or_comm.state->word |= OR_STATE_MODE_POS; // Set mode 

                  // TODO: tunable max position error
                  if ((fabs(fmod((or_comm.ctrl->pos - or_comm.state->pos + 540),360) - 180) >= (double)or_conf.safety_max_q_error) && !or_conf.safety_track_disable)
                  {
                    or_raise_error(ERR_POS_STEP); // Set error flag
                  } else {
                    if (!or_active_errors[ERR_POS_STEP] || or_conf.auto_clear_errors) //if 
                    {
                      mc_interface_set_pid_pos(RAD2DEG_f(or_comm.ctrl->pos));
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
                  if (((ST2US2(chVTGetSystemTimeX() - or_comm.last_update) > 10000) && or_comm.ctrl->vel != 0.0) && !or_conf.safety_timeout_disable) //raise error if command does not ensure at least 100Hz
                  {
                    or_raise_error(ERR_CAN_TIMEOUT);
                    break; // Stop executing velocity control when timeout occurs
                  }
                  or_comm.state->word |= OR_STATE_MODE_VEL; // Set mode
                  mc_interface_set_pid_speed(mc_interface_get_configuration()->p_pid_ang_div*RADPS2RPM_f(or_comm.ctrl->vel));
                  break;
                }
                case OR_CTRL_MODE_TRQ :
                {
                  if (((ST2US2(chVTGetSystemTimeX() - or_comm.last_update) > 10000) && !or_conf.safety_timeout_disable)) //raise error if command does not ensure at least 100Hz
                  {
                    or_raise_error(ERR_CAN_TIMEOUT);
                    break; // Stop executing torque control when timeout occurs
                  }
                  // that state word does not contain OR_STATE_ERR_TRQ_STEP
                  if (or_comm.state->word & OR_STATE_ERR_TRQ_STEP)
                  {
                    or_state.ext_torque_setpoint = 0.0;
                    //or_estop(); //TODO remove - handled by state machine
                  }
                  else
                  {
                    or_comm.state->word |= OR_STATE_MODE_TRQ; // Set mode
                    or_state.ext_torque_setpoint = or_comm.ctrl->trq;
                    if (or_conf.limits_enable_reaction && or_conf.limits_enable)
                      or_limits_reaction();
                    or_interface_torquecontrol();
                  }
                  break;
                }
                case OR_CTRL_MODE_IMP : //Impedance mode: available for later
                {
                  if ((ST2US2(chVTGetSystemTimeX() - or_comm.last_update) > 10000) && !or_conf.safety_timeout_disable) //raise error if command does not ensure at least 100Hz
                  {
                    or_raise_error(ERR_CAN_TIMEOUT);
                    break; // Stop executing impedance control when timeout occurs
                  }
                  or_comm.state->word |= OR_STATE_MODE_IMP; // Set mode
                  or_state.ext_pos_setpoint = or_comm.ctrl->pos;
                  or_state.ext_vel_setpoint = or_comm.ctrl->vel;
                  or_state.ext_torque_setpoint = or_comm.ctrl->trq;
                  break;
                }
                case OR_CTRL_MODE_CST : //Cusom mode: TOODO
                {
                  if ((ST2US2(chVTGetSystemTimeX() - or_comm.last_update) > 10000) && !or_conf.safety_timeout_disable) //raise error if command does not ensure at least 100Hz
                  {
                    or_raise_error(ERR_CAN_TIMEOUT);
                    break; // Stop executing custom control when timeout occurs
                  }
                  or_comm.state->word |= OR_STATE_MODE_CST; // Set mode
                  or_state.ext_pos_setpoint = or_comm.ctrl->pos;
                  or_state.ext_vel_setpoint = or_comm.ctrl->vel;
                  or_state.ext_torque_setpoint = or_comm.ctrl->trq;
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
            case OR_SAFETY_HOLD:
            {
              // Initialize hold by getting the current position as hold position
              if (!hold_initialized)
              {
                hold_position = or_comm.state->pos;
                hold_initialized = true;
              }

              // compute the actual error
              float error = fabs(fmod((hold_position - or_comm.state->pos + 540), 360) - 180);

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
                float pos_error = fabs(fmod((or_comm.ctrl->pos - or_comm.state->pos + 540), 360) - 180);

                if (or_conf.auto_clear_errors && or_active_errors[ERR_POS_STEP] &&
                    pos_error < 0.1*or_conf.safety_max_q_error && (or_comm.ctrl->word & OR_CTRL_MODE_MSK) == OR_CTRL_MODE_POS) //reset if auto_reset AND error is less than 10% the max allowed value. TODO: better definition of the treshold?
                {
                  or_clear_error(ERR_POS_STEP);
                  or_set_safety_mode(OR_SAFETY_ENABLE);
                }
              }

              break;
            }
            case OR_SAFETY_BRAKE:
            {
              mc_interface_set_brake_current(3); //brake at 3 amps - TODO: tunable
              hold_initialized = false;
              break;
            }
            case OR_SAFETY_ESTOP:
            {
              or_estop();
              hold_initialized = false;
              break;
            }
            default:
            {
              or_estop();
              hold_initialized = false;
              break; // trigger hold if unknown ?
            }
          }
        }
      }
    }
    
/* -------------------------------------------------------------------------- */
/*                          Execution time management                         */
/* -------------------------------------------------------------------------- */
    //compute and control loop time TODO: clean
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
  //TODO write clear control law bloc diagram
  or_state.torque_err = or_state.ext_torque_setpoint
                        - or_state.torque_now;
  //add stiffness action
  or_state.torque_err += or_conf.ctrl_stiffness
                *(or_state.ext_pos_setpoint-or_state.pos_multiturn_now);
  // add damping action
  or_state.torque_err -= or_conf.ctrl_damping*or_state.speed_now;
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
  float shaped_command = or_input_shaper(or_state.ctrl_command);
  mc_interface_set_current_rel(shaped_command);
  //}    //TODO: check limits after last ctrl_command computation and set to
                                                  //zero if out of limits?
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
  // ------------------ new command received: ------------------------//

  //compare or_comm.ctrl_prev and or_comm.ctrl to detect changes in word , pos, trq or vel
  if (or_comm.ctrl != or_comm.ctrl_prev)
  {
    if (or_comm.ctrl->word != or_comm.ctrl_prev->word) //detect change of control word
    {
      //TODO: manage modes switch
      //clear error if word goes from anything to 0x0000
      if (or_comm.ctrl->word == OR_STATE_MODE_OFF)
      {
        or_clear_error(ERR_POS_STEP); // Clear error //TODO: manage error clear
        if (or_comm.ctrl->trq == 0.0)
          or_clear_error(ERR_TRQ_STEP); // Clear error
        if (or_comm.ctrl->vel == 0.0)
          or_clear_error(ERR_VEL_STEP); // Clear error
        
        //setting control word OFF resets the safety mode to ENABLE if not critical
        if ((or_comm.state->word & OR_SAFETY_MSK) <= OR_SAFETY_HOLD)
        {
          or_set_safety_mode(OR_SAFETY_ENABLE);
        }

        mc_interface_release_motor(); //release motor last command -> allows control from VESC
      }
    }
    
    if ((fabs(or_comm.ctrl->trq - or_comm.ctrl_prev->trq) > 5) && !or_conf.safety_track_disable) //TODO: parametrable max torque command step
    {
      or_raise_error(ERR_TRQ_STEP); // Set error flag
    }

    if ((fabs(or_comm.ctrl->vel - or_comm.ctrl_prev->vel) > 5) && !or_conf.safety_track_disable)
    {
      or_raise_error(ERR_VEL_STEP); // Set error flag
    }

    or_comm.ctrl_prev->word = or_comm.ctrl->word; //save previous ctrl word
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
 * @return The new safety state (e.g. OR_SAFETY_HOLD)
 */
uint16_t or_evaluate_safety_state(void) {
  uint16_t current_safety_state = or_comm.state->word & OR_SAFETY_MSK;

  // Automatic INIT → IDLE
  if (current_safety_state == OR_SAFETY_INIT) {
    //Stay in INIT mode 8 seconds (enough to let the actuator boot properly and flush eventual remaining data in CAN buffer)
    if (or_state.encoders_init && ST2S(chVTGetSystemTimeX()) > 8) {
      return OR_SAFETY_IDLE;
    }
    return OR_SAFETY_INIT;
  }

  // Automatic IDLE → ENABLE only if no active errors
  if (current_safety_state == OR_SAFETY_IDLE) {
    if (or_compute_max_error_level() == ERR_LEVEL_NONE) {
      return OR_SAFETY_ENABLE;
    }
    return OR_SAFETY_IDLE;
  }

  // Escalation from ENABLE → HOLD → BRAKE → ESTOP
  
  or_error_level_t severity = or_compute_max_error_level(); 
  uint16_t new_safety_state = current_safety_state; //new safety state starts as-is and will be changed if needed
  
  switch (severity)
  {
    case ERR_LEVEL_ESTOP:
      new_safety_state = OR_SAFETY_ESTOP;
      break;
    case ERR_LEVEL_BRAKE:
      if (current_safety_state < OR_SAFETY_BRAKE) { //only escalate
        new_safety_state = OR_SAFETY_BRAKE;
      }
      break;
    case ERR_LEVEL_HOLD:
      if (current_safety_state < OR_SAFETY_HOLD) { //only escalate
        new_safety_state = OR_SAFETY_HOLD;
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
 * using the defined OR_SAFETY_MSK. It ensures mutually exclusive safety states.
 *
 * @param mode  The new safety mode to apply (e.g. OR_SAFETY_ENABLE).
 */
void or_set_safety_mode(uint32_t mode)
{
  //check if we are swithcing to ENABLE and trigger release if so
  if (mode == OR_SAFETY_ENABLE && (or_comm.ctrl->word & OR_CTRL_MODE_MSK) == OR_CTRL_MODE_OFF && (or_comm.state->word &= OR_SAFETY_MSK) != OR_SAFETY_ENABLE)
  {
  release_on_enable = true;
  }
  or_comm.state->word &= ~OR_SAFETY_MSK; // Clear current safety bits
  or_comm.state->word |= mode; // Set new safety mode
}

/**
 * @brief Set the current control mode in the orthopus control word.
 *
 * This function clears the existing control mode bits and sets the new mode
 * using the defined OR_CTRL_MODE_MSK. It sets only one control mode at a time.
 *
 * @param mode  The new control mode to apply (e.g. OR_CTRL_MODE_POS).
 */
void or_set_control_mode(uint32_t mode)
{
  or_comm.ctrl->word &= ~OR_CTRL_MODE_MSK; // Clear current control bits
  or_comm.ctrl->word |= mode; // Set new control mode
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
 * Input shaper for vibration reduction
 * 
 * Implements a simple input shaper that splits the command into two parts:
 * - A1 fraction sent immediately
 * - (1-A1) fraction sent after a delay T1
 *
 * @param input_command
 * The raw control command to be shaped
 *
 * @return
 * float - The shaped control command
 */
float or_input_shaper(float input_command)
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
    or_state.input_shaper_buffer[i] = or_state.input_shaper_buffer[i-1];
  }
  
  // Store current command at index 0 (newest)
  or_state.input_shaper_buffer[0] = input_command;
  
  // Get delayed command (older value from buffer)
  float delayed_command = or_state.input_shaper_buffer[or_state.input_shaper_delay_samples];
  
  // Calculate shaped output: A1 * current + (1-A1) * delayed
  float shaped_command = or_state.input_shaper_A1 * input_command + (1.0f - or_state.input_shaper_A1) * delayed_command;
  
  return shaped_command;
}