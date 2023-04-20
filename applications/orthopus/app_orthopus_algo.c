#include "app_orthopus.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***

static volatile bool orthopus_thread_stop = true;
static volatile bool orthopus_thread_running = false;

const int loop_rate = 2000; //loop rate in Hz

//AMS filter variables
static volatile float enc_pos_raw = 0.0;
static volatile float enc_pos_filter = 0.0;
static volatile float enc_last_pos_filter = 0.0;
static volatile int nb_enc_filter_error = 0;
static volatile int last_nb_enc_filter_error = 0;
static volatile unsigned long int nsample = 0;
static volatile float actual_pid_pos = 0;
static volatile float last_pid_pos = 0;
static volatile float actual_pos_multiturn = 0;
static volatile int actual_turn = 0;

float app_orthopus_get_enc_pos_filtered(void) {
	return enc_pos_filter;
}

float app_orthopus_get_pos_multiturn(void) {
	return actual_pos_multiturn;
}

/*void app_orthopus_set_filter_anglestep(float v) {
	filter_anglestep = v;
  commands_printf("Actual angle step: % 7.3f", (double)filter_anglestep);
}*/

static THD_FUNCTION(orthopus_thread, arg) {
	(void)arg;

	chRegSetThreadName("OrthopusTh");

	orthopus_thread_running = true;

  size_t encoder_wait=10;
  do
  {
    enc_as504x_read_angle(&encoder_cfg_as504x);
    chThdSleepMilliseconds(1);
    if(encoder_wait && !(--encoder_wait))
      break;
  } while(!encoder_cfg_as504x.state.sensor_diag.is_connected);

  float v = 0;
  orthopus_set_joint_offset(v, false);

  actual_pid_pos = mc_interface_get_pid_pos_now();
  last_pid_pos = actual_pid_pos;
  if ( actual_pid_pos > 180)
  {
    actual_turn = -1;
  }

  int get_fw_version_cnt = 0;
	for(;;) {
		// Check if it is time to stop.
		if (orthopus_thread_stop) {
			orthopus_thread_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.

    // You can call  functions such as:
    // - mc_interface_set_duty()
    // - mc_interface_set_pid_speed()
    // - mc_interface_set_pid_pos()

    
    // ENCODER EMI NOISE FILTERING
    enc_pos_raw = orthopus_read_encoder();
    //TODO: take into account current speed to copare raw value with next expected value instead of previous value
    if (orthopus_config.encoder_filter_enable)
    {
      last_nb_enc_filter_error = nb_enc_filter_error;
      if ( ((float)fabs(enc_pos_raw - enc_last_pos_filter) > (orthopus_config.encoder_filter_anglestep * (1 + nb_enc_filter_error))) && (fabs(enc_pos_raw - enc_last_pos_filter) < 350) && (nb_enc_filter_error < 5))
      {
        nb_enc_filter_error += 1;
        enc_pos_filter = enc_last_pos_filter;
      } else {
        enc_pos_filter = enc_pos_raw;
        nb_enc_filter_error = 0;
      }

      if (fabs(enc_pos_raw - enc_last_pos_filter) > 350)
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
          commands_plot_add_graph("enc_pos_raw");
          commands_plot_add_graph("last_nb_enc_filter_error");
          commands_plot_add_graph("enc_last_pos_filter");
        }
        commands_plot_set_graph(0);
        commands_send_plot_points(nsample, enc_pos_filter);
        commands_plot_set_graph(1);
        commands_send_plot_points(nsample, enc_pos_raw);
        commands_plot_set_graph(2);
        commands_send_plot_points(nsample, last_nb_enc_filter_error);
        commands_plot_set_graph(3);
        commands_send_plot_points(nsample, enc_last_pos_filter);
      }
      enc_last_pos_filter = enc_pos_filter;
    } else {
      enc_pos_filter = enc_pos_raw;
      enc_last_pos_filter = enc_pos_filter;
      nb_enc_filter_error = 0;
    }
    // Compute encoder position multiturn
    actual_pid_pos = mc_interface_get_pid_pos_now();
    if (actual_pid_pos - last_pid_pos < -350)
    {
      actual_turn += 1;
    } else if (actual_pid_pos - last_pid_pos > 350)
    {
      actual_turn -= 1;
    }
    actual_pos_multiturn = actual_pid_pos + 360*actual_turn;
    last_pid_pos = actual_pid_pos;
    // Check coherence between rotor position (sin/cos) and encoder position
    //TODO
    // Watch axis software limits
    if (orthopus_config.limits_enable)
    {
      if ((actual_pos_multiturn > orthopus_config.limits_pos_max) || (actual_pos_multiturn < orthopus_config.limits_pos_min))
      {
        //commands_printf("Warning actuator outside position limits.");
        //mc_interface_fault_stop(FAULT_CODE_NONE, false, false); //NOT WORKING (getting stuck with last setpoint). //TODO: manage proper error handling
      }
    }

    chThdSleepMicroseconds(1000000*1/loop_rate);

    // Use commands_get_fw_version_sent_cnt() to guess if we're (re?)connected to a GUI
    /*
    bool plot_started=true;
		if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
			get_fw_version_cnt = commands_get_fw_version_sent_cnt();
			plot_started = false;
		}

    // Init the plot in the APP from here.
    if (!plot_started) {
      plot_started = true;
      commands_init_plot("X", "Y");
      commands_plot_add_graph("myPlot");
    }

    commands_plot_set_graph(0);
    commands_send_plot_points(enc_as504x_read_angle(&encoder_cfg_as504x), orthopus_read_encoder());
    */

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
