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
static void terminal_test(int argc, const char **argv);
static void orthopus_init_sincos_cmd(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

static float enc_v=0;

static float orthopus_enc_read_deg(void)
{
  return enc_v;
}
static bool orthopus_enc_fault(void)
{
  return false;
}

static const char *enc_info = "Helloooo";
static char* orthopus_enc_print_info(void)
{
  return enc_info;
}

static bool orthopus_enc_init(void)
{
  commands_printf("EncInit()");
  enc_v = 17.9;
  return true;
}

static bool orthopus_enc_deinit(void)
{
  commands_printf("EncDeinit()");

}

static void orthopus_init_sincos(const float v)
{
  commands_printf("EncInitSinCos()");

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
    &orthopus_enc_read_deg,
    &orthopus_enc_fault,
    &orthopus_enc_print_info
  );

  terminal_register_command_callback(
    "o_init_sincos",
    "[Orthopus] Initialize SINCOS sensor with AS504x (or force value)",
    "[d]",
    orthopus_init_sincos_cmd
  );

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"custom_cmd",
			"Print the number d",
			"[d]",
			terminal_test);

  enc_v = 12.34;
}

static void orthopus_init_sincos_cmd(int argc, const char **argv) {

	if (argc == 2) {
		float v = -1;
		sscanf(argv[1], "%f", &v);
    enc_v = v;

		commands_printf("You have entered %f", v);
  }
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(terminal_test);

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

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		commands_printf("You have entered %d", d);

		// For example, read the ADC inputs on the COMM header.
		commands_printf("ADC1: %.2f V ADC2: %.2f V ADC3: %.2f",
				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2), (double)ADC_VOLTS(ADC_IND_EXT3));
	} else {
		commands_printf("This command requires one argument.\n");
	}
}
