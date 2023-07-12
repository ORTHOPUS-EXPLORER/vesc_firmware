#include "app_orthopus.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***
//#include <math.h> //for atanf function

static volatile bool orthopus_thread_stop = true;
static volatile bool orthopus_thread_running = false;

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

//const int loop_rate = 2000; //loop rate in Hz

extern volatile orthopus_state_t orthopus_state =
{
  .pos_multiturn_now = 0.0,
  .enc_pos_filter = 0.0,
  .speed_now = 0.0,
  .enc_pos   = 0.0,
  .ADC3zero = 0.0,
  .turn_now = 0,
  .ext_torque_setpoint = 0,
  .ext_pos_setpoint = 0
};


int get_fw_version_cnt;
float enc_pos_filter_last = 0.0;
int nb_enc_filter_error = 0;
int last_nb_enc_filter_error = 0;
unsigned long int nsample = 0;
float pid_pos_now = 0;
float pid_pos_last = 0;
static systime_t time_now, time_last, time_start, time_end;
//static int time_now, time_last, time_start, time_end;
int ninitadc = 0;

static THD_FUNCTION(orthopus_thread, arg) {
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
  if ( pid_pos_now > 180)
  {
    orthopus_state.turn_now = -1;
  }

  get_fw_version_cnt = 0;
  orthopus_state.maxperiod = -1; //init max period
  orthopus_state.minperiod = -1; //init min period
  orthopus_thread_running = true;
  time_now = chVTGetSystemTimeX();
  time_last = time_now;
  //check if torquezero set in config
  if (orthopus_config.Torquezero != 0.0 && orthopus_config.orthopus_config_set){
      orthopus_state.ADC3init = true;
      orthopus_state.ADC3zero = orthopus_config.Torquezero;
  }
	for(;;)
  {
    time_start = chVTGetSystemTimeX();
		// Check if it is time to stop.
		if (orthopus_thread_stop) {
			orthopus_thread_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

    // Read AMS
    enc_as504x_routine(&encoder_cfg_as504x);

		// Run your logic here. A lot of functionality is available in mc_interface.h.

    // You can call  functions such as:
    // - mc_interface_set_duty()
    // - mc_interface_set_pid_speed()
    // - mc_interface_set_pid_pos()


    // ENCODER EMI NOISE FILTERING
    orthopus_state.enc_pos = orthopus_read_encoder();
    //TODO: take into account current speed to compare raw value with next expected value instead of previous value
    if (orthopus_config.encoder_filter_enable)
    {
      last_nb_enc_filter_error = nb_enc_filter_error;
      if ( ( (float)fabs(orthopus_state.enc_pos - enc_pos_filter_last) >
             ( orthopus_config.encoder_filter_anglestep *
               (1 + orthopus_config.encoder_filter_error_gain*nb_enc_filter_error)
             )
           )
           && (fabs(orthopus_state.enc_pos - enc_pos_filter_last) < 350)
           && (nb_enc_filter_error < 5)
         )
      {
        ++nb_enc_filter_error;
        orthopus_state.enc_pos_filter = enc_pos_filter_last;
      }
      else
      {
        orthopus_state.enc_pos_filter = orthopus_state.enc_pos;
        nb_enc_filter_error = 0;
      }

      if (fabs(orthopus_state.enc_pos - enc_pos_filter_last) > 350)
        nb_enc_filter_error = 0;
      ++nsample;

      if (orthopus_config.encoder_filter_plot_enable)//debug encoder filter
        orthopus_plot_encoder_filtering(nsample);
    }
    else
    {
      orthopus_state.enc_pos_filter = orthopus_state.enc_pos;
      nb_enc_filter_error = 0;
    }
    enc_pos_filter_last = orthopus_state.enc_pos_filter;
    // Compute encoder position multiturn
    pid_pos_now = mc_interface_get_pid_pos_now();
    if (pid_pos_now - pid_pos_last < -350)
      ++orthopus_state.turn_now;
    else if (pid_pos_now - pid_pos_last > 350)
      --orthopus_state.turn_now;

    orthopus_state.pos_multiturn_now = pid_pos_now + 360*orthopus_state.turn_now;
    pid_pos_last = pid_pos_now;
    orthopus_state.speed_now = mc_interface_get_rpm()/orthopus_config.angle_division;
    // Check coherence between rotor position (sin/cos) and encoder position
    //TODO
    if (orthopus_config.limits_enable)
      orthopus_limits();

    //Impedance control
    orthopus_state.ADC3val = ADC_VOLTS(ADC_IND_EXT3);
    if (!orthopus_state.ADC3init) //init ADC Zero
    {
      ++ninitadc;
      orthopus_state.ADC3zero += orthopus_state.ADC3val/500.0;
      if (ninitadc == 500)
      {
        orthopus_state.ADC3init = true;
        commands_printf("torque zero done: orthopus_state.ADC3zero: % 7.3f", (double)orthopus_state.ADC3zero);
        ninitadc = 0; ///TODO: debug
      }
    } else {
      orthopus_state.Torque = (1-orthopus_config.torque_filter_const)*orthopus_state.Torque
                              + orthopus_config.torque_filter_const*orthopus_config.Torquegain*(orthopus_state.ADC3val-orthopus_state.ADC3zero);
                              //TODO: low lag low pass filter 
      if (orthopus_state.ctrl_plot)
        orthopus_plot_impedance(nsample);
      //TODO: control loop
      if (orthopus_state.ctrl_enable)
      {
        orthopus_state.torqueerror = orthopus_state.ext_torque_setpoint-orthopus_state.Torque;
        //add stiffness action
        orthopus_state.torqueerror -= orthopus_config.ctrl_stiffness*(orthopus_state.ext_pos_setpoint-orthopus_state.pos_multiturn_now);
        // add damping action
        orthopus_state.torqueerror += orthopus_config.ctrl_damping*orthopus_state.speed_now;
        //add limits action
        orthopus_state.torqueerror -= orthopus_state.limitreaction;
        if (orthopus_config.deadzone)
        {
          //orthopus_state.ctrl_command = -orthopus_config.ctrl_kp * (orthopus_state.ext_torque_setpoint-orthopus_state.Torque);
          //orthopus_state.ctrl_command = orthopus_state.ctrl_command - atanf(orthopus_state.ctrl_command*orthopus_config.a)/orthopus_config.a;
          orthopus_state.ctrl_command = -orthopus_config.ctrl_kp * (orthopus_state.torqueerror);
          orthopus_state.ctrl_command = orthopus_state.ctrl_command - atanf(orthopus_state.ctrl_command*orthopus_config.a)/orthopus_config.a;
        } else {
          //orthopus_state.ctrl_command = -orthopus_config.ctrl_kp * (orthopus_state.ext_torque_setpoint-orthopus_state.Torque);
          orthopus_state.ctrl_command = -orthopus_config.ctrl_kp * (orthopus_state.torqueerror);
        }
        //add stiffness action
        //orthopus_state.ctrl_command += orthopus_config.ctrl_stiffness*(orthopus_state.ext_pos_setpoint-orthopus_state.pos_multiturn_now);
        //add limit action
        //orthopus_state.ctrl_command += orthopus_state.limitreaction;
        mc_interface_set_current_off_delay(0.1); //prevent disabling motor if torque request is 0
        mc_interface_set_current_rel(orthopus_state.ctrl_command);
      } //TODO: check limits after last ctrl_command computation and set to zero if out of limits?
    }
    
    
    //compute and control loop time TODO: clean
    if (orthopus_state.perfplot)
      orthopus_plot_cycletime(nsample);
    time_now = chVTGetSystemTimeX();
    orthopus_state.time_diff = ST2US2(time_now - time_last);
    orthopus_state.time_diff_filt = 0.99*orthopus_state.time_diff_filt
                                  + 0.01*(orthopus_state.time_diff
                                  + orthopus_state.time_lag_compensation); //simple filter
    orthopus_state.time_lag_filt = orthopus_state.time_diff_filt - 1000000.0*1.0/orthopus_config.rate_hz;
    if (orthopus_state.time_diff > orthopus_state.maxperiod)
      orthopus_state.maxperiod = orthopus_state.time_diff;
    if (orthopus_state.time_diff < orthopus_state.minperiod)
      orthopus_state.minperiod = orthopus_state.time_diff;
    time_end = chVTGetSystemTimeX();
    orthopus_state.exectime = time_end - time_start;
    if (orthopus_config.perf_compensateexectime)
      chThdSleepMicroseconds(1000000.0*1.0/orthopus_config.rate_hz-ST2US2(orthopus_state.exectime));
    else 
      chThdSleepMicroseconds(1000000.0*1.0/orthopus_config.rate_hz);
    time_last = time_now;
	}
}

// Called in mc_interface.c:1913, in mc_interface_mc_timer_isr()
static void orthopus_pwm_callback(void)
{
	// Called for every control iteration in interrupt context.

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
static void orthopus_estop(void)
{
  mc_interface_release_motor();   //disable motor
  mc_interface_ignore_input(100);  // disable new inputs for at least 1 cycle (100ms)
}


/**
 * Limits management: stop actuator if exceeding defined limits (position, speed, etc.)
 *
 * @param
 * void
 *
 * @return
 * void
 */
static void orthopus_limits(void)
{
  //check position limits
  orthopus_state.limitreaction = 0;
  if ((orthopus_state.pos_multiturn_now > orthopus_config.limits_pos_max) || (orthopus_state.pos_multiturn_now < orthopus_config.limits_pos_min))
  {
    orthopus_estop();
  }
  else if ( (
            (orthopus_state.speed_now > orthopus_config.limits_reach_speed
              && (orthopus_state.pos_multiturn_now > orthopus_config.limits_pos_max - orthopus_config.limits_reach_angle)
            )
            ||
            ( orthopus_state.speed_now < -orthopus_config.limits_reach_speed
              && (orthopus_state.pos_multiturn_now < orthopus_config.limits_pos_min + orthopus_config.limits_reach_angle)
            )
            )
            && (!orthopus_state.ctrl_enable)
          )
  {
    orthopus_estop();
  }
  if ((orthopus_state.ctrl_enable) && (orthopus_state.pos_multiturn_now < orthopus_config.limits_pos_min + orthopus_config.limits_reach_angle)){
    orthopus_state.limitreaction = orthopus_config.limits_kp*pow((orthopus_state.pos_multiturn_now-(orthopus_config.limits_pos_min + orthopus_config.limits_reach_angle)),orthopus_config.limits_powp);
    /*if (orthopus_state.speed_now < 0) //add damping, only in the direction of the limit to avoid sticking effect
    {
      orthopus_state.limitreaction += -orthopus_config.limits_kd*pow(orthopus_state.speed_now,orthopus_config.limits_powd);
    }*/
  }
  if ((orthopus_state.ctrl_enable) && (orthopus_state.pos_multiturn_now > orthopus_config.limits_pos_max - orthopus_config.limits_reach_angle)) { //-3 adds a zone before reach angle in which we add a friction
    orthopus_state.limitreaction = -orthopus_config.limits_kp*pow((orthopus_state.pos_multiturn_now-(orthopus_config.limits_pos_max - orthopus_config.limits_reach_angle)),orthopus_config.limits_powp);
    /*if (orthopus_state.speed_now > 0) //add damping, only in the direction of the limit to avoid sticking effect
    {
      orthopus_state.limitreaction += -orthopus_config.limits_kd*pow(orthopus_state.speed_now,orthopus_config.limits_powd);
    }*/
  }
  if ((orthopus_state.ctrl_enable) && (orthopus_state.pos_multiturn_now < orthopus_config.limits_pos_min + orthopus_config.limits_reach_angle + orthopus_config.limits_damp_reachangle) && (orthopus_state.speed_now < 0)) //add damping, only in the direction of the limit to avoid sticking effect
  {
    orthopus_state.limitreaction += -orthopus_config.limits_kd*pow(orthopus_state.speed_now,orthopus_config.limits_powd);
  }
  if ((orthopus_state.ctrl_enable) && (orthopus_state.pos_multiturn_now > orthopus_config.limits_pos_max - orthopus_config.limits_reach_angle - orthopus_config.limits_damp_reachangle) && (orthopus_state.speed_now > 0)) //add damping, only in the direction of the limit to avoid sticking effect
  {
    orthopus_state.limitreaction += -orthopus_config.limits_kd*pow(orthopus_state.speed_now,orthopus_config.limits_powd);
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
static void orthopus_plot_encoder_filtering(int ns)
{
  bool plot_started=true;
  if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
    get_fw_version_cnt = commands_get_fw_version_sent_cnt();
    plot_started = false;
  }
  if (!plot_started) {
    plot_started = true;
    commands_init_plot("time", "angle");
    commands_plot_add_graph("enc_pos_filter");
    commands_plot_add_graph("enc_pos");
    commands_plot_add_graph("last_nb_enc_filter_error");
    commands_plot_add_graph("enc_pos_filter_last");
  }
  commands_plot_set_graph(0);
  commands_send_plot_points(ns, orthopus_state.enc_pos_filter);
  commands_plot_set_graph(1);
  commands_send_plot_points(ns, orthopus_state.enc_pos);
  commands_plot_set_graph(2);
  commands_send_plot_points(ns, last_nb_enc_filter_error);
  commands_plot_set_graph(3);
  commands_send_plot_points(ns, enc_pos_filter_last);
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
static void orthopus_plot_cycletime(int ns)
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
  commands_send_plot_points(ns, orthopus_state.time_diff*1.0);
  commands_plot_set_graph(1);
  commands_send_plot_points(ns, orthopus_state.time_diff_filt*1.0);
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
static void orthopus_plot_impedance(int ns)
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
    commands_plot_add_graph("Torque");
    commands_plot_add_graph("Controlout");
  }
  commands_plot_set_graph(0);
  commands_send_plot_points(ns, orthopus_state.ADC3val*1.0);
  commands_plot_set_graph(1);
  commands_send_plot_points(ns, orthopus_state.Torque*1.0);
  commands_plot_set_graph(2);
  commands_send_plot_points(ns, orthopus_state.ctrl_command*1.0);
}