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
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "encoder/enc_as504x.h"
#include "encoder/enc_sincos.h"
#include "encoder/encoder_cfg.h" // For encoder_cfg_***
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void pwm_callback(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

static size_t init_delay = 10;
static float init_v = 0;

static float orthopus_enc_read_deg(void)
{
  return enc_sincos_read_deg(&encoder_cfg_sincos);
        // AS504x_LAST_ANGLE(&encoder_cfg_as504x)
}

static bool orthopus_enc_fault(void)
{
  return false;
}

static const char* orthopus_enc_print_info(void)
{
  static char b[512];
  sprintf(b , "AMS: % 7.3f SINCOS: % 7.3f Offset: %7.3f"
            , (double)AS504x_LAST_ANGLE(&encoder_cfg_as504x)
            , (double)enc_sincos_read_deg(&encoder_cfg_sincos)
            , (double)init_v
         );
  return b;
}

static bool orthopus_enc_init(void)
{
  commands_printf("EncInit()");
  SENSOR_PORT_3V3();
    // const cast
  volatile mc_configuration* conf = (volatile mc_configuration*)mc_interface_get_configuration();
  encoder_cfg_sincos.s_gain = 1.0 / conf->m_encoder_sin_amp;
  encoder_cfg_sincos.s_offset = conf->m_encoder_sin_offset;
  encoder_cfg_sincos.c_gain = 1.0 /conf->m_encoder_cos_amp;
  encoder_cfg_sincos.c_offset =  conf->m_encoder_cos_offset;
  encoder_cfg_sincos.filter_constant = conf->m_encoder_sincos_filter_constant;
  sincosf(DEG2RAD_f(conf->m_encoder_sincos_phase_correction), &encoder_cfg_sincos.sph, &encoder_cfg_sincos.cph);

  return enc_sincos_init(&encoder_cfg_sincos) && enc_as504x_init(&encoder_cfg_as504x);
}

static void orthopus_enc_deinit(void)
{
  commands_printf("EncDeinit()");
  enc_as504x_deinit(&encoder_cfg_as504x);
  enc_sincos_deinit(&encoder_cfg_sincos);
}

static void orthopus_init_offset(const float v);

static void orthopus_enc_routine(void)
{
  enc_as504x_routine(&encoder_cfg_as504x);
  if(init_delay && !(--init_delay))
  {
    orthopus_init_offset(AS504x_LAST_ANGLE(&encoder_cfg_as504x));
  }
}

static void orthopus_init_offset(const float v)
{
  init_delay = 0;
  init_v = v;
  mc_interface_update_pid_pos_offset(v, false);
}

static void orthopus_init_offset_cmd(int argc, const char **argv)
{
  float v = AS504x_LAST_ANGLE(&encoder_cfg_as504x);
	if (argc == 2) {
		sscanf(argv[1], "%f", &v);
  }
	commands_printf("Init Pos PID Offset with joint offset: %f", (double)v);
  orthopus_init_offset(v);
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

  commands_printf("AppStart()");

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

  encoder_set_custom_callbacks(
    &orthopus_enc_init,
    &orthopus_enc_deinit,
    &orthopus_enc_routine,
    &orthopus_enc_read_deg,
    &orthopus_enc_fault,
    &orthopus_enc_print_info
  );
  // const cast
  mc_configuration* conf = (mc_configuration*)mc_interface_get_configuration();
  // Force re-init custom encoder
  if(conf->m_sensor_port_mode == SENSOR_PORT_MODE_CUSTOM_ENCODER)
  {
    encoder_init(conf);
  }

  terminal_register_command_callback(
    "o_init_offset",
    "[Orthopus] Initialize Pos PID offset with current AMS (or forced) value",
    "[d]",
    orthopus_init_offset_cmd
  );
}


// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(orthopus_init_offset_cmd);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom");

	is_running = true;

	// Example of using the experiment plot
//	chThdSleepMilliseconds(8000);
//	commands_init_plot("Sample", "Voltage");
//	commands_plot_add_graph("Temp Fet");
//	commands_plot_add_graph("Input Voltage");
//	float samp = 0.0;
//
//	for(;;) {
//		commands_plot_set_graph(0);
//		commands_send_plot_points(samp, mc_interface_temp_fet_filtered());
//		commands_plot_set_graph(1);
//		commands_send_plot_points(samp, GET_INPUT_VOLTAGE());
//		samp++;
//		chThdSleepMilliseconds(10);
//	}

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.

		chThdSleepMilliseconds(10);
	}
}

static void pwm_callback(void) {
	// Called for every control iteration in interrupt context.
}
