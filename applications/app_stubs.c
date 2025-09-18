/*
	Copyright 2025 VESC Project

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

/*
 * Application Stubs
 * 
 * This file provides dummy function implementations for disabled applications
 * to prevent linking errors and ensure compatibility when applications are
 * conditionally compiled out.
 */

#include "app.h"

// PPM Application Stubs
#if !APP_PPM_ENABLE
void app_ppm_start(void) {}
void app_ppm_stop(void) {}
float app_ppm_get_decoded_level(void) { return 0.0; }
void app_ppm_detach(bool detach) { (void)detach; }
void app_ppm_override(float val) { (void)val; }
void app_ppm_configure(ppm_config *conf) { (void)conf; }
#endif

// ADC Application Stubs
#if !APP_ADC_ENABLE
void app_adc_start(bool use_rx_tx) { (void)use_rx_tx; }
void app_adc_stop(void) {}
void app_adc_configure(adc_config *conf) { (void)conf; }
float app_adc_get_decoded_level(void) { return 0.0; }
float app_adc_get_voltage(void) { return 0.0; }
float app_adc_get_decoded_level2(void) { return 0.0; }
float app_adc_get_voltage2(void) { return 0.0; }
void app_adc_detach_adc(int detach) { (void)detach; }
void app_adc_adc1_override(float val) { (void)val; }
void app_adc_adc2_override(float val) { (void)val; }
void app_adc_detach_buttons(bool state) { (void)state; }
void app_adc_rev_override(bool state) { (void)state; }
void app_adc_cc_override(bool state) { (void)state; }
bool app_adc_range_ok(void) { return true; }
#endif

// UART Application Stubs
#if !APP_UART_ENABLE
void app_uartcomm_initialize(void) {}
void app_uartcomm_start(UART_PORT port_number) { (void)port_number; }
void app_uartcomm_stop(UART_PORT port_number) { (void)port_number; }
void app_uartcomm_configure(uint32_t baudrate, bool permanent_enabled, UART_PORT port_number) {
    (void)baudrate; (void)permanent_enabled; (void)port_number;
}
void app_uartcomm_send_packet(unsigned char *data, unsigned int len, UART_PORT port_number) {
    (void)data; (void)len; (void)port_number;
}
#endif

// Nunchuk Application Stubs
#if !APP_NUNCHUK_ENABLE
void app_nunchuk_start(void) {}
void app_nunchuk_stop(void) {}
void app_nunchuk_configure(chuk_config *conf) { (void)conf; }
float app_nunchuk_get_decoded_x(void) { return 0.0; }
float app_nunchuk_get_decoded_y(void) { return 0.0; }
bool app_nunchuk_get_bt_c(void) { return false; }
bool app_nunchuk_get_bt_z(void) { return false; }
bool app_nunchuk_get_is_rev(void) { return false; }
float app_nunchuk_get_update_age(void) { return 0.0; }
void app_nunchuk_update_output(chuck_data *data) { (void)data; }
#endif

// PAS Application Stubs
#if !APP_PAS_ENABLE
void app_pas_start(bool is_primary_output) { (void)is_primary_output; }
void app_pas_stop(void) {}
bool app_pas_is_running(void) { return false; }
void app_pas_configure(pas_config *conf) { (void)conf; }
float app_pas_get_current_target_rel(void) { return 0.0; }
float app_pas_get_pedal_rpm(void) { return 0.0; }
void app_pas_set_current_sub_scaling(float current_sub_scaling) { (void)current_sub_scaling; }
#endif

// Custom Application Stubs
#if !APP_CUSTOM_ENABLE
void app_custom_start(void) {}
void app_custom_stop(void) {}
void app_custom_configure(app_configuration *conf) { (void)conf; }
#endif
