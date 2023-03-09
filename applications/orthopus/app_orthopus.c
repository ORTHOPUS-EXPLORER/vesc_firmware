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
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Private functions
static void my_pwm_callback(void);

// Private variables

static void orthopus_init_offset_cmd(int argc, const char **argv)
{
  float v = 0;
	if (argc == 2)
  {
		sscanf(argv[1], "%f", &v);
  }
  else
  {
    size_t i = 0;
    enc_as504x_read_angle(&encoder_cfg_as504x); // Bypass first null value
    for(i=0;i<3;i++)
      v += enc_as504x_read_angle(&encoder_cfg_as504x)/3;
  }
	commands_printf("Init Pos PID Offset with joint offset: %f", (double)v);
  mc_interface_update_pid_pos_offset(v, false);
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
  commands_printf("AppStart()");

  if(!enc_as504x_init(&encoder_cfg_as504x))
    commands_printf("AMS init failed");

  // Re-init ADC_Ext3 since enc_as504x_init() may have overwritten it...
	// Use SCL/MOSI/TX as ADC_EXT3.
	//   Header pin is tied to PA7 (AIN7) and PB10.
	//   Set PA7 as Analog input and PB10 in Hi-Z.
	palSetPadMode(GPIOA,  7, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 10, PAL_MODE_INPUT);

  // Hard-RT context
	mc_interface_set_pwm_callback(my_pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

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

	chRegSetThreadName("AppCustomTh");

	is_running = true;

	chThdSleepMilliseconds(1);
  orthopus_init_offset_cmd(0, NULL);

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

    // You can call  functions such as:
    // - mc_interface_set_duty()
    // - mc_interface_set_pid_speed()
    // - mc_interface_set_pid_pos()

		chThdSleepMilliseconds(10);
	}
}

// Called in mc_interface.c:1913, in mc_interface_mc_timer_isr()
static void my_pwm_callback(void) {
	// Called for every control iteration in interrupt context.

}
