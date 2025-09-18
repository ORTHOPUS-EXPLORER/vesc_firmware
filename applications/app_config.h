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

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/*
 * Application Enable/Disable Configuration
 * 
 * These defines are set by the build system (applications.mk) and control
 * which applications are compiled into the firmware. This significantly
 * reduces firmware size when unused applications are disabled.
 * 
 * To change the configuration, modify the settings in applications/applications.mk
 * 
 * Note: APP_CUSTOM (orthopus) should stay enabled if you're using orthopus applications.
 */

// Set default values if not defined by build system
#ifndef APP_PPM_ENABLE
#define APP_PPM_ENABLE          0  // PPM/RC input application
#endif

#ifndef APP_ADC_ENABLE  
#define APP_ADC_ENABLE          0  // Analog input application
#endif

#ifndef APP_UART_ENABLE
#define APP_UART_ENABLE         0  // UART communication application  
#endif

#ifndef APP_NUNCHUK_ENABLE
#define APP_NUNCHUK_ENABLE      0  // Wii Nunchuk input application
#endif

#ifndef APP_PAS_ENABLE
#define APP_PAS_ENABLE          0  // Pedal Assist System application
#endif

#ifndef APP_CUSTOM_ENABLE
#define APP_CUSTOM_ENABLE       1  // Custom applications (including orthopus)
#endif

#ifndef APP_CUSTOM_TEMPLATE_ENABLE
#define APP_CUSTOM_TEMPLATE_ENABLE  0  // Custom template for development
#endif

#ifndef APP_STEN_ENABLE
#define APP_STEN_ENABLE         0  // STEN specific application
#endif

#ifndef APP_DPV_ENABLE
#define APP_DPV_ENABLE          0  // DPV (Diver Propulsion Vehicle) application
#endif

#ifndef APP_SKYPUFF_ENABLE
#define APP_SKYPUFF_ENABLE      0  // Skypuff specific application
#endif

/*
 * Validation: At least one application should be enabled
 */
#if !APP_PPM_ENABLE && !APP_ADC_ENABLE && !APP_UART_ENABLE && \
    !APP_NUNCHUK_ENABLE && !APP_PAS_ENABLE && !APP_CUSTOM_ENABLE
#warning "No applications enabled! At least one application should be enabled."
#endif

#endif /* APP_CONFIG_H_ */
