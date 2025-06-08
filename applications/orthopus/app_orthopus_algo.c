#include "app_orthopus.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***
#include "mc_interface.h"
//#include <math.h> //for atanf function

volatile bool orthopus_thread_stop = true;
volatile bool orthopus_thread_running = false;

//const int loop_rate = 2000; //loop rate in Hz

volatile orthopus_state_t or_state =
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
unsigned long int nsample = 0;
float pid_pos_now = 0;
float pid_pos_last = 0;
systime_t time_now, time_last, time_start, time_end;
systime_t time_lasterrprint;
int ninitadc = 0;
bool or_active_errors[ERR_COUNT] = { false }; //array tracking all errors state
bool or_error_triggered[ERR_COUNT]; // true = triggered at least once since startup/reset

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
  orthopus_set_joint_offset(v, false);

  pid_pos_now = mc_interface_get_pid_pos_now();
  pid_pos_last = pid_pos_now;
  if ( pid_pos_now > 180.0)
  {
    or_state.turn_now = -1;
  }

  if (orthopus_read_encoder() > 180.0)
  {
    or_state.enc_turn = -1;
  }
  get_fw_version_cnt = 0;
  or_state.perf_max_period = -1; //init max period
  or_state.perf_min_period = -1; //init min period
  orthopus_thread_running = true;
  time_now = chVTGetSystemTimeX();
  time_lasterrprint = chVTGetSystemTimeX();
  time_last = time_now;
  //check if torquezero set in config
  if (or_conf.ctrl_torquezero != 0.0 && or_conf.signature == ORTHOPUS_CONFIG_T_SIGNATURE)
  {
      or_state.adc3_init = true;
      or_state.adc3_zero = or_conf.ctrl_torquezero;
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

    or_state.enc_pos = orthopus_read_encoder();
    //TODO: take into account current speed to compare raw value with next 
                                      //expected value instead of previous value

    /* ------------------------------- Count turns ------------------------------ */
    //Output encoder
    if (or_state.enc_pos - enc_pos_last < -350.0)
      ++or_state.enc_turn;
    else if (or_state.enc_pos - enc_pos_last > 350.0)
      --or_state.enc_turn;
    or_state.enc_pos_multiturn = 0.1*(or_state.enc_pos
                                              + 360.0*or_state.enc_turn)
                                        + 0.9*or_state.enc_pos_multiturn;
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

    //TODO: speed now computed at higher freq
    or_state.speed_now = mc_interface_get_rpm() / mc_interface_get_configuration()->p_pid_ang_div;

    /* --------------------------------- Limits --------------------------------- */
    if (or_conf.limits_enable)
      orthopus_limits();

    /* ---------------------------- Sample adc3 value --------------------------- */
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

      /* ------------------------------ Control plot ------------------------------ */
      if (or_state.ctrl_plot){
        ++nsample;
        orthopus_plot_impedance(nsample);
      }

      /* ------------------------------ Safety / modes switch --------------------- */
      //compare orthopus_comm.ctrl_prev and orthopus_comm.ctrl to detect changes in word , pos, trq or vel
      if (orthopus_comm.ctrl != orthopus_comm.ctrl_prev)
      {
        if (orthopus_comm.ctrl->word != orthopus_comm.ctrl_prev->word)
        {
          //TODO: manage modes switch
          //clear error if word goes from anything to 0x0000
          if (orthopus_comm.ctrl->word == 0x0000)
          {
            orthopus_comm.state->word &= ~ORTHOPUS_STATE_ERR_POS_STEP; // Clear error //TODO: manage error clear
            if (orthopus_comm.ctrl->trq == 0.0)
              orthopus_comm.state->word &= ~ORTHOPUS_STATE_ERR_TRQ_STEP; // Clear error
            if (orthopus_comm.ctrl->vel == 0.0)
              orthopus_comm.state->word &= ~ORTHOPUS_STATE_ERR_VEL_STEP; // Clear error
          }
        }
        if (fabs(orthopus_comm.ctrl->trq - orthopus_comm.ctrl_prev->trq) > 10) //TODO: parametrable max torque command step
        {
          orthopus_comm.state->word |= ORTHOPUS_STATE_ERR_TRQ_STEP; // Set error flag
        }
        orthopus_comm.ctrl_prev->word = orthopus_comm.ctrl->word; //save previous ctrl word
      } 

      if (!orthopus_safety())
      {
        //if orthopus_safety failed to handle modes, fallback to ESTOP (should never happend)
        orthopus_comm.state->word = (orthopus_comm.state->word &= ~ORTHOPUS_SAFETY_MSK) | ORTHOPUS_SAFETY_ESTOP;
      } 
      else 
      {
        /* -------------------------------------------------------------------------- */
        /*                              Main control loop                             */
        /* -------------------------------------------------------------------------- */

        orthopus_comm.state->word = (orthopus_comm.state->word & ~ORTHOPUS_SAFETY_MSK) | evaluate_safety_state();

        if(orthopus_comm.process_ctrl && !or_conf.simu_mode) //TODO: deal with those cases properly
        {
          switch(orthopus_comm.state->word & ORTHOPUS_SAFETY_MSK)
          {
            case ORTHOPUS_SAFETY_INIT:
            {
              break;
            }
            case ORTHOPUS_SAFETY_IDLE:
            {
              break;
            }
            case ORTHOPUS_SAFETY_ENABLE:
            {
              orthopus_comm.state->word &= ~ORTHOPUS_STATE_MODE_MSK;                       // Clear mode
              // Modes are currently exclusive. Refactor this switch for mode fine-grained mode control
              switch(orthopus_comm.ctrl->word & ORTHOPUS_CTRL_MODE_MSK) 
              {
                case ORTHOPUS_CTRL_MODE_POS:
                {
                  orthopus_comm.state->word |= ORTHOPUS_STATE_MODE_POS; // Set mode 

                  // TODO: tunable max position error
                  if (fabs(fmod((orthopus_comm.ctrl->pos - orthopus_comm.state->pos + 540),360) - 180) >= (double)or_conf.safety_max_q_error)
                  {
                    raise_error(ERR_POS_STEP); // Set error flag
                  } else {
                    if (!or_active_errors[ERR_POS_STEP] || or_conf.auto_clear_errors) //if 
                    {
                      mc_interface_set_pid_pos(orthopus_comm.ctrl->pos);
                    }

                    if (or_conf.auto_clear_errors)
                    {
                      clear_error(ERR_POS_STEP); // Clear error
                    }
                    or_state.ctrl_enable = false; //todo: proper management of modes swhitch
                  }
                  break;
                }
                case ORTHOPUS_CTRL_MODE_VEL:
                {
                  orthopus_comm.state->word |= ORTHOPUS_STATE_MODE_VEL; // Set mode
                  mc_interface_set_pid_speed(mc_interface_get_configuration()->p_pid_ang_div*RADPS2RPM_f(orthopus_comm.ctrl->vel)/10); //TODO: check why factor 10
                  or_state.ctrl_enable = false; //todo: proper management of modes swhitch
                  break;
                }
                case ORTHOPUS_CTRL_MODE_TRQ :
                {
                  // that state word does not contain ORTHOPUS_STATE_ERR_TRQ_STEP
                  if (orthopus_comm.state->word & ORTHOPUS_STATE_ERR_TRQ_STEP)
                  {
                    or_state.ext_torque_setpoint = 0.0;
                    orthopus_estop();
                  }
                  else
                  {
                    orthopus_comm.state->word |= ORTHOPUS_STATE_MODE_TRQ; // Set mode
                    or_state.ext_torque_setpoint = orthopus_comm.ctrl->trq;
                    or_state.ctrl_enable = true;
                  }
                  break;
                }
                case ORTHOPUS_CTRL_MODE_IMP : //Impedance mode: available for later
                {
                  orthopus_comm.state->word |= ORTHOPUS_STATE_MODE_IMP; // Set mode
                  or_state.ctrl_enable = true;
                  or_state.ext_pos_setpoint = orthopus_comm.ctrl->pos;
                  or_state.ext_vel_setpoint = orthopus_comm.ctrl->vel;
                  or_state.ext_torque_setpoint = orthopus_comm.ctrl->trq;
                  break;
                }
                case ORTHOPUS_CTRL_MODE_CST : //Cusom mode: TOODO
                {
                  orthopus_comm.state->word |= ORTHOPUS_STATE_MODE_CST; // Set mode
                  or_state.ctrl_enable = false; //TODO: enable custom control mode
                  or_state.ext_pos_setpoint = orthopus_comm.ctrl->pos;
                  or_state.ext_vel_setpoint = orthopus_comm.ctrl->vel;
                  or_state.ext_torque_setpoint = orthopus_comm.ctrl->trq;
                  break;
                }
                default:
                  break;
              }
              break;
            }
            case ORTHOPUS_SAFETY_HOLD:
            {
              break;
            }
            case ORTHOPUS_SAFETY_BRAKE:
            {
              break;
            }
            case ORTHOPUS_SAFETY_ESTOP:
            {
              break;
            }
            default:
              break; // trigger hold if unknown ?
          }
        }

        /* -------------------------------------------------------------------------- */
        /*                              Impedance control                             */
        /* -------------------------------------------------------------------------- */
        if (or_state.ctrl_enable)
        {
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
          mc_interface_set_current_rel(or_state.ctrl_command);
        }    //TODO: check limits after last ctrl_command computation and set to
                                                        //zero if out of limits?
      }
    }
    
/* -------------------------------------------------------------------------- */
/*                          Execution time management                         */
/* -------------------------------------------------------------------------- */
    //compute and control loop time TODO: clean
/* ---------------------------- Performances plot --------------------------- */
    if (or_state.perf_plot){
      ++nsample;
      orthopus_plot_cycletime(nsample);
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
void orthopus_pwm_callback(void)
{
	// Called for every control iteration in interrupt context.
  //Sample torque sensor ADC at high frequency
  or_state.adc3_filt = or_conf.torque_filter_const*ADC_VOLTS(ADC_IND_EXT3)
                     + (1-or_conf.torque_filter_const)*or_state.adc3_filt;
  /*or_state.speed_now = or_conf.speed_filter_const*(mc_interface_get_rpm() / mc_interface_get_configuration()->p_pid_ang_div)
                     + (1-or_conf.speed_filter_const)*or_state.speed_now;*/
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
void orthopus_estop(void)
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
bool orthopus_safety(void)
{
  //check indicators
  if (or_state.nid1 > 50 ) {
    commands_printf("estop: too many identical !=0 ctrl_command detected");
    or_state.nid1 = 0;
    return false;
  } 
  /* else if (fabsf(or_state.enc_pos_filter_multiturn-or_state.pos_multiturn_now) //TODO debug: o_offset encoder causes vesc reboot when activated
                                                     > or_conf.encoder_max_diff)
  {
    if (ST2S(chVTGetSystemTimeX()-time_lasterrprint) > 2) 
    { //Print an error message every 2 seconds
      time_lasterrprint = chVTGetSystemTimeX();
      commands_printf("estop: Error: unconsistent sincos/encoder position");
    }
    return false;
  } */ 
  return true;
}

/**
 * Limits management: stop actuator if exceeding defined limits
 * (position, speed, etc.)
 *
 * @param
 * void
 *
 * @return
 * void
 */
void orthopus_limits(void)
{
  //check position limits
  or_state.limit_reaction = 0;
  if (   (or_state.pos_multiturn_now > or_conf.limits_pos_max)
      || (or_state.pos_multiturn_now < or_conf.limits_pos_min))
  {
    orthopus_estop();
    or_state.ctrl_enable = false;
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
            && (!or_state.ctrl_enable)
          )
  {
    orthopus_estop();
  }
  if ( (or_state.ctrl_enable)
       &&
       (or_state.pos_multiturn_now 
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
  if ( (or_state.ctrl_enable)
       &&
       (or_state.pos_multiturn_now
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
  if ( (or_state.ctrl_enable) 
       &&
       (or_state.pos_multiturn_now
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
  if ( (or_state.ctrl_enable)
       &&
       (or_state.pos_multiturn_now
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
void orthopus_plot_encoder_filtering(int ns)
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
void orthopus_plot_cycletime(int ns)
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
void orthopus_plot_impedance(int ns)
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
or_error_level_t get_error_severity(or_error_t err) {
  switch (err) {
    case ERR_POS_STEP:
    case ERR_TRQ_STEP:
      return ERR_LEVEL_HOLD;

    case ERR_VEL_STEP:
      return ERR_LEVEL_BRAKE;

    default:
      return ERR_LEVEL_WARNING;
  }
}

/**
 * @brief Compute the maximum severity level among all active errors.
 * 
 * @return The highest severity level of any currently active error.
 */
or_error_level_t compute_max_error_level(void) {
  or_error_level_t max_level = ERR_LEVEL_NONE;

  for (int i = 0; i < ERR_COUNT; ++i) {
    if (or_active_errors[i]) {
      or_error_level_t level = get_error_severity((or_error_t)i);
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
void raise_error(or_error_t err) {
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
void clear_error(or_error_t err) {
  if (err < ERR_COUNT)
    or_active_errors[err] = false;
}


/**
 * @brief Evaluates the appropriate safety level based on current errors.
 *        Only escalation is allowed (no automatic downgrade).
 * 
 * @return The new safety state (e.g. ORTHOPUS_SAFETY_HOLD)
 */
uint16_t evaluate_safety_state(void) {
  uint16_t current_safety_state = orthopus_comm.state->word & ORTHOPUS_SAFETY_MSK;

  // Automatic INIT → IDLE
  if (current_safety_state == ORTHOPUS_SAFETY_INIT) {
    if (or_state.encoders_init /* && other init flags */) {
      return ORTHOPUS_SAFETY_IDLE;
    }
    return ORTHOPUS_SAFETY_INIT;
  }

  // Automatic IDLE → ENABLE only if no active errors
  if (current_safety_state == ORTHOPUS_SAFETY_IDLE) {
    if (compute_max_error_level() == ERR_LEVEL_NONE) {
      return ORTHOPUS_SAFETY_ENABLE;
    }
    return ORTHOPUS_SAFETY_IDLE;
  }

  // Escalation from ENABLE → HOLD → BRAKE → ESTOP
  
  or_error_level_t severity = compute_max_error_level(); 
  uint16_t new_safety_state = current_safety_state; //new safety state starts as-is and will be changed if needed
  
  switch (severity)
  {
    case ERR_LEVEL_ESTOP:
      new_safety_state = ORTHOPUS_SAFETY_ESTOP;
      break;
    case ERR_LEVEL_BRAKE:
      if (current_safety_state < ORTHOPUS_SAFETY_BRAKE) { //only escalate
        new_safety_state = ORTHOPUS_SAFETY_BRAKE;
      }
      break;
    case ERR_LEVEL_HOLD:
      if (current_safety_state < ORTHOPUS_SAFETY_HOLD) { //only escalate
        new_safety_state = ORTHOPUS_SAFETY_HOLD;
      } //TODO: allow returning to ENABLE / Position control?
      break;
    case ERR_LEVEL_WARNING: //do not change safety state
      break;
    case ERR_LEVEL_NONE: //do not change safety state
      break;
  }

  return new_safety_state;
}