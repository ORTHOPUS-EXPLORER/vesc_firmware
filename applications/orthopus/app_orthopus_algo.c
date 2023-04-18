#include "app_orthopus.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***

static volatile bool orthopus_thread_stop = true;
static volatile bool orthopus_thread_running = false;

static THD_FUNCTION(orthopus_thread, arg) {
	(void)arg;

	chRegSetThreadName("OrthopusTh");

	orthopus_thread_running = true;

  size_t encoder_wait=10;
  do
  {
    enc_as504x_read_angle(&encoder_cfg_as504x);
    chThdSleepMilliseconds(100);
    if(encoder_wait && !(--encoder_wait))
      break;
  } while(!encoder_cfg_as504x.state.sensor_diag.is_connected);

  float v = 0;
  orthopus_set_joint_offset(v, false);

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

		chThdSleepMilliseconds(10);

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
