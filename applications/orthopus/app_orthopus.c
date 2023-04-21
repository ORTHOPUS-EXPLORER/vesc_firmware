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
#include "lispif.h"
#include "lispbm.h"
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

#include "app_orthopus.h"

// Threads
static THD_WORKING_AREA(orthopus_thread_wa, 1024);

// Just make sure to pad to a 32bit aligned size.
// Eg: if you add an uint8_t param, add 3 bytes of padding after.
//                   uint16_t param     2
// uint8_t myparam; uint8_t[3] pad;
static orthopus_config_t orthopus_config =
{
  .encoder_offset = 0.0,
}; //init values to zero in case flash can't be read


static void orthopus_process_custom_app_data(unsigned char *rx_d, unsigned int len);
static void orthopus_process_custom_hw_data(unsigned char *rx_d, unsigned int len);

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
  commands_printf("OrthopusAppStart");

  SENSOR_PORT_3V3();
  if(!enc_as504x_init(&encoder_cfg_as504x))
    commands_printf("AMS init failed");

  // Re-init ADC_Ext3 since enc_as504x_init() may have overwritten it...
	// Use SCL/MOSI/TX as ADC_EXT3.
	//   Header pin is tied to PA7 (AIN7) and PB10.
	//   Set PA7 as Analog input and PB10 in Hi-Z.
	palSetPadMode(GPIOA,  7, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 10, PAL_MODE_INPUT);

  // Hard-RT context
	mc_interface_set_pwm_callback(orthopus_pwm_callback);

	orthopus_thread_stop = false;
	chThdCreateStatic(orthopus_thread_wa, sizeof(orthopus_thread_wa),
			NORMALPRIO, orthopus_thread, NULL);

  // Add shell commands
  orthopus_cmd_init();

  // Add LISP commands/symbols
  lispif_set_ext_load_callback(&orthopus_init_lisp);

  commands_set_app_data_handler(orthopus_process_custom_app_data);
  commands_set_hw_data_handler(orthopus_process_custom_hw_data);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
	mc_interface_set_pwm_callback(0);
	orthopus_cmd_deinit();

	orthopus_thread_stop = true;
	while (orthopus_thread_running)
		chThdSleepMilliseconds(1);
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;

  if(!orthopus_config_load(&orthopus_config))
     commands_printf("Orthopus_config_load failed");
  // if orthopus config not set, set default values
  if (!orthopus_config.orthopus_config_set)
  {
    orthopus_config.encoder_offset                  = 49.7; //for OR14B005 todo set to zero
    orthopus_config.encoder_filter_anglestep        = 0.25;
    orthopus_config.encoder_filter_enable           = true;
    orthopus_config.encoder_filter_plot_enable      = false;
    orthopus_config.limits_enable                   = false;
    orthopus_config.orthopus_config_set             = false;
    orthopus_config.limits_pos_max                  = 90.0;
    orthopus_config.limits_pos_min                  = -90.0;
    orthopus_config.angle_division                  = 700;
    orthopus_config.limits_reach_angle              = 15;
    orthopus_config.limits_reach_speed              = 2;
  }
}

static void orthopus_process_custom_app_data(unsigned char *rx_d, unsigned int len)
{
  (void)rx_d; (void)len;

  unsigned int olen=0;
  unsigned char tx_d[256];
  if(olen)
    commands_send_app_data(tx_d, olen);
}

// FIXME: Figure out how to send these packets from the APP or from CAN
static void orthopus_process_custom_hw_data(unsigned char *rx_d, unsigned int len)
{
  (void)rx_d; (void)len;

  unsigned int olen=0;
  unsigned char tx_d[256];
  if(olen)
    commands_send_hw_data(tx_d, olen);
}