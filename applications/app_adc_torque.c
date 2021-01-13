/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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
//

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

// Threads
static THD_FUNCTION(adc_torque_thread, arg);
static THD_WORKING_AREA(adc_torque_thread_wa, 1024);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float target = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool run_motor = false;


void app_adc_torque_configure(adc_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
}

void app_adc_torque_start(bool use_rx_tx) {
	use_rx_tx_as_buttons = use_rx_tx;
	stop_now = false;
	chThdCreateStatic(adc_torque_thread_wa, sizeof(adc_torque_thread_wa), NORMALPRIO, adc_torque_thread, NULL);
}

void app_adc_torque_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_adc_torque_get_decoded_level(void) {
	return decoded_level;
}

float app_adc_torque_get_voltage(void) {
	return read_voltage;
}


static THD_FUNCTION(adc_torque_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_ADC_TORQUE");

	// Set servo pin as an input with pullup
	if (use_rx_tx_as_buttons) {
		palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
		palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
	} else {
		palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
	}

	is_running = true;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		// Read the external ADC pin and convert the value to a voltage.
		float pwr = (float)ADC_Value[ADC_IND_EXT];
		read_voltage = ((float)ADC_Value[ADC_IND_EXT])/4905 * V_REG;

		// Optionally apply a mean value filter
		if (config.use_filter) {
			static float filter_buffer[FILTER_SAMPLES];
			static int filter_ptr = 0;

			filter_buffer[filter_ptr++] = read_voltage;
			if (filter_ptr >= FILTER_SAMPLES) {
				filter_ptr = 0;
			}

			read_voltage = 0.0;
			for (int i = 0;i < FILTER_SAMPLES;i++) {
				read_voltage += filter_buffer[i];
			}
			read_voltage /= FILTER_SAMPLES;
		}

		//Start and stop based on readings
		static int over_min_pulse_count = 0;
		static int under_min_pulse_count = 0;
		if(target >= MIN_TARGET){
			over_min_pulse_count ++;
			under_min_pulse_count = 0;
			if(over_min_pulse_count >= START_COUNT){
				run_motor = true;
			}
		}
		else{
			under_min_pulse_count++;
			over_min_pulse_count = 0;
			if(over_min_pulse_count >= STOP_COUNT){
				run_motor = false;
			}
		}

		if(!run_motor){

		}


		// Map the read voltage
		// switch (config.ctrl_type) {
		// Voltage start must be the 0 torque voltage
		if (read_voltage < config.voltage_center) {
			target = utils_map(pwr, config.voltage_start, config.voltage_center, 0.0, 0.5);
		} else {
			target = utils_map(pwr, config.voltage_center, config.voltage_end, 0.5, 1.0);
		}

		// Truncate the target output
		utils_truncate_number(&target, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage_inverted) {
			target = 1.0 - target;
		}

		decoded_level = target;

		//read_button_pins();

		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		// Apply deadband
		utils_deadband(&target, config.hyst, 1.0);

		// Apply throttle curve
		target = utils_throttle_curve(target, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time = fabsf(target) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, target, ramp_step);
			last_time = chVTGetSystemTimeX();
			target = pwr_ramp;
		}

		float current_rel = 0.0;
		bool current_mode = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();

		// Use the filtered and mapped voltage for control according to the configuration.
		//switch (config.ctrl_type) {
		current_mode = true;
		if ((target >= 0.0 && rpm_now >= 0.0) || (target < 0.0 && rpm_now < 0.0)) {
			current_rel = target;
		} else {
			current_rel = target;
		}

		if (fabsf(target) < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}


		// If safe start is enabled and the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());

			continue;
		}

		// Reset timeout
		timeout_reset();

		// If c is pressed and no throttle is used, maintain the current speed with PID control
		static bool was_pid = false;

		// Filter RPM to avoid glitches
		static float filter_buffer[RPM_FILTER_SAMPLES];
		static int filter_ptr = 0;
		filter_buffer[filter_ptr++] = mc_interface_get_rpm();
		if (filter_ptr >= RPM_FILTER_SAMPLES) {
			filter_ptr = 0;
		}

		float rpm_filtered = 0.0;
		for (int i = 0;i < RPM_FILTER_SAMPLES;i++) {
			rpm_filtered += filter_buffer[i];
		}
		rpm_filtered /= RPM_FILTER_SAMPLES;

		if (fabsf(target) < 0.001) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				was_pid = true;
				pid_rpm = rpm_filtered;
			}

			mc_interface_set_pid_speed(pid_rpm);

			continue;
		}

		was_pid = false;


		// if (current_mode) {
		float current_out = current_rel;
		mc_interface_set_current_rel(current_out);
	}
}

void read_button_pins(){
	// Read the button pins
	bool cc_button = false;
	bool rev_button = false;
	if (use_rx_tx_as_buttons) {
		cc_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);
		if (config.cc_button_inverted) {
			cc_button = !cc_button;
		}
		rev_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
		if (config.rev_button_inverted) {
			rev_button = !rev_button;
		}
	} else {
		// When only one button input is available, use it differently depending on the control mode
		if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON ||
				config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER ||
				config.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON ||
				config.ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON ||
				config.ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON) {
			rev_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
			if (config.rev_button_inverted) {
				rev_button = !rev_button;
			}
		} else {
			cc_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
			if (config.cc_button_inverted) {
				cc_button = !cc_button;
			}
		}
	}
}
