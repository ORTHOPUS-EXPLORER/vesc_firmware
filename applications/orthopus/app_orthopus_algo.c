#include "app_orthopus.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***

static volatile bool orthopus_thread_stop = true;
static volatile bool orthopus_thread_running = false;

const int loop_rate = 2000; //loop rate in Hz

static volatile orthopus_state_t orthopus_state =
{
  .pos_multiturn_now = 0.0,
  .enc_pos_filter = 0.0,
  .speed_now = 0.0
};

//AMS filter variables
static volatile float enc_pos = 0.0;
static volatile float enc_pos_filter_last = 0.0;
static volatile int nb_enc_filter_error = 0;
static volatile int last_nb_enc_filter_error = 0;
static volatile unsigned long int nsample = 0;
static volatile float pid_pos_now = 0;
static volatile float pid_pos_last = 0;
static volatile int turn_now = 0;

/*float app_orthopus_get_enc_pos_filtered(void) {
	return orthopus_state.enc_pos_filter;
}*/

/*float app_orthopus_get_pos_multiturn(void) {
	return orthopus_state.pos_multiturn_now;
}*/

/*void app_orthopus_set_filter_anglestep(float v) {
	filter_anglestep = v;
  commands_printf("Actual angle step: % 7.3f", (double)filter_anglestep);
}*/

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
    turn_now = -1;
  }

  int get_fw_version_cnt = 0;
  orthopus_thread_running = true;
	for(;;)
  {
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
    enc_pos = orthopus_read_encoder();
    //TODO: take into account current speed to compare raw value with next expected value instead of previous value
    if (orthopus_config.encoder_filter_enable)
    {
      last_nb_enc_filter_error = nb_enc_filter_error;
      if ( ((float)fabs(enc_pos - enc_pos_filter_last) > (orthopus_config.encoder_filter_anglestep * (1 + orthopus_config.encoder_filter_error_gain*nb_enc_filter_error))) && (fabs(enc_pos - enc_pos_filter_last) < 350) && (nb_enc_filter_error < 5))
      {
        nb_enc_filter_error += 1;
        orthopus_state.enc_pos_filter = enc_pos_filter_last;
      } else {
        orthopus_state.enc_pos_filter = enc_pos;
        nb_enc_filter_error = 0;
      }

      if (fabs(enc_pos - enc_pos_filter_last) > 350)
      {
        nb_enc_filter_error = 0;
      }
      nsample += 1;

      if (orthopus_config.encoder_filter_plot_enable)//debug encoder filter
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
        commands_send_plot_points(nsample, orthopus_state.enc_pos_filter);
        commands_plot_set_graph(1);
        commands_send_plot_points(nsample, enc_pos);
        commands_plot_set_graph(2);
        commands_send_plot_points(nsample, last_nb_enc_filter_error);
        commands_plot_set_graph(3);
        commands_send_plot_points(nsample, enc_pos_filter_last);
      }
      enc_pos_filter_last = orthopus_state.enc_pos_filter;
    } else {
      orthopus_state.enc_pos_filter = enc_pos;
      enc_pos_filter_last = orthopus_state.enc_pos_filter;
      nb_enc_filter_error = 0;
    }
    // Compute encoder position multiturn
    pid_pos_now = mc_interface_get_pid_pos_now();
    if (pid_pos_now - pid_pos_last < -350)
    {
      turn_now += 1;
    } else if (pid_pos_now - pid_pos_last > 350)
    {
      turn_now -= 1;
    }
    orthopus_state.pos_multiturn_now = pid_pos_now + 360*turn_now;
    pid_pos_last = pid_pos_now;
    orthopus_state.speed_now = mc_interface_get_rpm()/orthopus_config.angle_division; // TODO compute actual speed
    // Check coherence between rotor position (sin/cos) and encoder position
    //TODO
    // Watch axis software limits
    if (orthopus_config.limits_enable)
    {
      //check position limits
      if ((orthopus_state.pos_multiturn_now > orthopus_config.limits_pos_max) || (orthopus_state.pos_multiturn_now < orthopus_config.limits_pos_min))
      {
        mc_interface_release_motor();   //disable motor
        mc_interface_ignore_input(100);  // disable new inputs for at least 1 cycle (100ms)
        //chThdSleepMilliseconds(10);     //sleep 10ms
      //TODO all in the same if or create estop fuction
      } else if ((orthopus_state.speed_now > orthopus_config.limits_reach_speed
                  && (orthopus_state.pos_multiturn_now > orthopus_config.limits_pos_max - orthopus_config.limits_reach_angle))
                  ||
                  ( orthopus_state.speed_now < -orthopus_config.limits_reach_speed
                  && (orthopus_state.pos_multiturn_now < orthopus_config.limits_pos_min + orthopus_config.limits_reach_angle)))
      {
        mc_interface_release_motor();   //disable motor
        mc_interface_ignore_input(100);  // disable new inputs for at least 1 cycle (100ms)
        //chThdSleepMilliseconds(10);     //sleep 10ms
      }

    }

    chThdSleepMicroseconds(1000000*1/loop_rate);

    // Couldn't figure out how this works
    //float samples[10]={12.34,56.78};
    //commands_send_experiment_samples(samples, 10);
	}
}

// Called in mc_interface.c:1913, in mc_interface_mc_timer_isr()
static void orthopus_pwm_callback(void)
{
	// Called for every control iteration in interrupt context.

}
