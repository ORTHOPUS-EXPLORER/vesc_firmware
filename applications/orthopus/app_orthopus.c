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
#include "buffer.h"

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
static orthopus_config_t or_conf =
{
  .encoder_offset = 0.0,
}; //init values to zero in case flash can't be read


static orthopus_comm_t orthopus_comm =
{
  .st0 = {},
  .st1 = {},
  .state = &orthopus_comm.st0,
  .ctrl0 = { 
    .word = 0x0
  },
  .ctrl1 = { 
    .word = 0x0
  },
  .ctrl  = &orthopus_comm.ctrl0
};

static void orthopus_process_custom_app_data(unsigned char *rx_d, unsigned int len);
static void orthopus_process_custom_hw_data(unsigned char *rx_d, unsigned int len);

static bool orthopus_process_can_sid(uint32_t id, uint8_t *data, uint8_t len);
static bool orthopus_process_can_eid(uint32_t id, uint8_t *data, uint8_t len);

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
  commands_printf("OrthopusAppStart");

  // Load config from EEPROM
  if(!orthopus_config_load(&or_conf))
     commands_printf("Orthopus_config_load failed");
  // if orthopus config not set, set default values
  if (!or_conf.or_conf_set)
  {
    orthopus_config_reset(&or_conf);
  }

  // Init AMS sensor
  SENSOR_PORT_3V3();
  if(!enc_as504x_init(&encoder_cfg_as504x))
    commands_printf("AMS init failed");

  // Re-init ADC_Ext3 since enc_as504x_init() may have overwritten it...
	// Use SCL/MOSI/TX as ADC_EXT3.
	//   Header pin is tied to PA7 (AIN7) and PB10.
	//   Set PA7 as Analog input and PB10 in Hi-Z.
	palSetPadMode(GPIOA,  7, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 10, PAL_MODE_INPUT);

  // Add shell commands
  orthopus_cmd_init();

  // Add LISP commands/symbols
  lispif_add_ext_load_callback(&orthopus_init_lisp);

  // Custom Packets Handlers
  commands_set_app_data_handler(orthopus_process_custom_app_data);
  commands_set_hw_data_handler(orthopus_process_custom_hw_data);

  // Custom CAN handlers
  comm_can_set_sid_rx_callback(orthopus_process_can_sid);
  comm_can_set_eid_rx_callback(orthopus_process_can_eid);

 // Hard-RT context
	mc_interface_set_pwm_callback(orthopus_pwm_callback);

  // Custom thread
	orthopus_thread_stop = false;
	chThdCreateStatic(orthopus_thread_wa, sizeof(orthopus_thread_wa),
			NORMALPRIO+40, orthopus_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
  // RT Context
	mc_interface_set_pwm_callback(0);
  // Custom thread
	orthopus_thread_stop = true;
	while (orthopus_thread_running)
		chThdSleepMilliseconds(1);

  // CustomPacket handlers
  commands_set_app_data_handler(0);
  commands_set_hw_data_handler(0);
  // Custom CAN handlers
  comm_can_set_sid_rx_callback(0);
  comm_can_set_eid_rx_callback(0);

  // Commands
	orthopus_cmd_deinit();
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

void orthopus_comm_update_state(void);

static void orthopus_process_custom_app_data(unsigned char *rx_d, unsigned int len)
{
  // RX
  const size_t isize = sizeof(orthopus_comm_control_t)+2;
  if(len == isize && rx_d[0] == 0x70)
  {
    // Get the "free" buffer
    orthopus_comm_control_t* ctrl = orthopus_comm.ctrl  = orthopus_comm.ctrl  == &(orthopus_comm.ctrl1)
                                  ? &(orthopus_comm.ctrl0) 
                                  : &(orthopus_comm.ctrl1);
    long int ilen = 2;
    // Fill in some data from the received packet
    ctrl->word = buffer_get_uint32      (rx_d, &ilen);
    ctrl->pos  = buffer_get_float32_auto(rx_d, &ilen);
    ctrl->vel  = buffer_get_float32_auto(rx_d, &ilen);
    ctrl->trq  = buffer_get_float32_auto(rx_d, &ilen);
    // Activate
    orthopus_comm.ctrl = ctrl; // Swap !
  }

  // TX
  const size_t osize = sizeof(orthopus_comm_state_t)+2; 
  unsigned char tx_d[osize];
  long int olen=2;
  tx_d[0] = 0x12;
  tx_d[1] = 0x45;
  // Get the current buffer
  orthopus_comm_state_t* st = orthopus_comm.state;
  // Copy the data to te send buffer
  buffer_append_uint32      (tx_d, st->word, &olen);
  buffer_append_float32_auto(tx_d, st->pos,  &olen);
  buffer_append_float32_auto(tx_d, st->vel,  &olen);
  buffer_append_float32_auto(tx_d, st->trq,  &olen);
  buffer_append_float32_auto(tx_d, st->temp, &olen);
  buffer_append_float32_auto(tx_d, st->curr, &olen);
  commands_send_app_data(tx_d, osize);
}

static void orthopus_process_custom_hw_data(unsigned char *rx_d, unsigned int len)
{
  (void)rx_d; (void)len;

  unsigned int olen=0;
  unsigned char tx_d[256];
  tx_d[0] = 0x98;
  tx_d[1] = 0x42;
  olen = 2;
  if(olen)
    commands_send_hw_data(tx_d, olen);
}


static bool orthopus_process_can_sid(uint32_t id, uint8_t *data, uint8_t len)
{
  (void)id; (void)data; (void)len;
  //int32_t send_index = 0;
	//uint8_t buffer[8];
	//buffer_append_uint32(buffer, 0x1234567890, &send_index);
	//comm_can_send_buffer(id, buffer, send_index, 0);
  return false;
}

static bool orthopus_process_can_eid(uint32_t id, uint8_t *data, uint8_t len)
{
  (void)id; (void)data; (void)len;
  return false;
}