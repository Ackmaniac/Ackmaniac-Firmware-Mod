/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * app_nunchuk.c
 *
 *  Created on: 18 okt 2014
 *      Author: benjamin
 */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "commands.h"
#include "utils.h"
#include "timeout.h"
#include <string.h>
#include <math.h>
#include "led_external.h"
#include "datatypes.h"
#include "comm_can.h"

// Settings
#define OUTPUT_ITERATION_TIME_MS		1
#define MAX_CURR_DIFFERENCE				5.0
#define MAX_CAN_AGE						0.1
#define RPM_FILTER_SAMPLES				8

// Threads
static THD_FUNCTION(chuk_thread, arg);
static THD_WORKING_AREA(chuk_thread_wa, 1024);
static THD_FUNCTION(output_thread, arg);
static THD_WORKING_AREA(output_thread_wa, 1024);

// Private variables
static volatile bool is_running = false;
static volatile chuck_data chuck_d;
static volatile int chuck_error = 0;
static volatile chuk_config config;
static volatile throttle_config throt_config;

static volatile bool output_running = false;

static float px[5];
static float py[5];
static float nx[5];
static float ny[5];

float calculate_throttle_curve_chuk(float *x, float *y, float bezier_reduce_factor, float t) {
	
	float directSteps;
	if (t < x[1]){
	    directSteps = (y[1] / x[1] * t);
	} else if (t > x[3]) {
	    directSteps = ((y[4] - y[3]) / (x[4] - x[3]) * (t-x[3]) + y[3]);
	} else if (t > x[2]) {
	    directSteps = ((y[3] - y[2]) / (x[3] - x[2]) * (t-x[2]) + y[2]);
	} else if (t > x[1]) {
	    directSteps = ((y[2] - y[1]) / (x[2] - x[1]) * (t-x[1]) + y[1]);
	} else { // (throttle == x[1])
	    directSteps = y[1];
	};

	float f[5];
	for (int i = 0; i < 5; i++) f[i] = y[i];

	for (int j = 1; j < 5; j++ )
		for (int i = 4; i >= j; i--)
			f[i] = ( (t - x[i-j]) * f[i] - (t - x[i]) * f[i-1]) / (x[i] - x[i-j]);

	float spline = f[4] - ((f[4] - directSteps) * bezier_reduce_factor);

	// safety when stupid values are entered for x and y
	if (spline > 1.0) return 1.0;
	if (spline < 0.0) return 0.0;

	return spline;
}

void app_nunchuk_configure(chuk_config *conf, throttle_config *throttle_conf) {
	config = *conf;
	throt_config = *throttle_conf;
	
	px[0] = 0.0;
	px[1] = throt_config.x1_throttle;
	px[2] = throt_config.x2_throttle;
	px[3] = throt_config.x3_throttle;
	px[4] = 1.0;
	py[0] = 0.0;
	py[1] = throt_config.y1_throttle;
	py[2] = throt_config.y2_throttle;
	py[3] = throt_config.y3_throttle;
	py[4] = 1.0;
	
	nx[0] = 0.0;
	nx[1] = throt_config.x1_neg_throttle;
	nx[2] = throt_config.x2_neg_throttle;
	nx[3] = throt_config.x3_neg_throttle;
	nx[4] = 1.0;
	ny[0] = 0.0;
	ny[1] = throt_config.y1_neg_throttle;
	ny[2] = throt_config.y2_neg_throttle;
	ny[3] = throt_config.y3_neg_throttle;
	ny[4] = 1.0;
}

void app_nunchuk_start(void) {
	chuck_d.js_y = 128;
	chThdCreateStatic(chuk_thread_wa, sizeof(chuk_thread_wa), NORMALPRIO, chuk_thread, NULL);
}

float app_nunchuk_get_decoded_chuk(void) {
	return ((float)chuck_d.js_y - 128.0) / 128.0;
}

void app_nunchuk_update_output(chuck_data *data) {
	if (!output_running) {
		output_running = true;
		chuck_d.js_y = 128;
		chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO, output_thread, NULL);
	}

	chuck_d = *data;
	timeout_reset();
}

static THD_FUNCTION(chuk_thread, arg) {
	(void)arg;

	chRegSetThreadName("Nunchuk i2c");
	is_running = true;

	uint8_t rxbuf[10];
	uint8_t txbuf[10];
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(5);
	i2caddr_t chuck_addr = 0x52;
	chuck_data chuck_d_tmp;

	hw_start_i2c();
	chThdSleepMilliseconds(10);

	for(;;) {
		bool is_ok = true;

		txbuf[0] = 0xF0;
		txbuf[1] = 0x55;
		i2cAcquireBus(&HW_I2C_DEV);
		status = i2cMasterTransmitTimeout(&HW_I2C_DEV, chuck_addr, txbuf, 2, rxbuf, 0, tmo);
		i2cReleaseBus(&HW_I2C_DEV);
		is_ok = status == MSG_OK;

		if (is_ok) {
			txbuf[0] = 0xFB;
			txbuf[1] = 0x00;
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, chuck_addr, txbuf, 2, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;
		}

		if (is_ok) {
			txbuf[0] = 0x00;
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, chuck_addr, txbuf, 1, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;
		}

		if (is_ok) {
			chThdSleepMilliseconds(3);

			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterReceiveTimeout(&HW_I2C_DEV, chuck_addr, rxbuf, 6, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;
		}

		if (is_ok) {
			static uint8_t last_buffer[6];
			int same = 1;

			for (int i = 0;i < 6;i++) {
				if (last_buffer[i] != rxbuf[i]) {
					same = 0;
				}
			}

			memcpy(last_buffer, rxbuf, 6);

			if (!same) {
				chuck_error = 0;
				chuck_d_tmp.js_x = rxbuf[0];
				chuck_d_tmp.js_y = rxbuf[1];
				chuck_d_tmp.acc_x = (rxbuf[2] << 2) | ((rxbuf[5] >> 2) & 3);
				chuck_d_tmp.acc_y = (rxbuf[3] << 2) | ((rxbuf[5] >> 4) & 3);
				chuck_d_tmp.acc_z = (rxbuf[4] << 2) | ((rxbuf[5] >> 6) & 3);
				chuck_d_tmp.bt_z = !((rxbuf[5] >> 0) & 1);
				chuck_d_tmp.bt_c = !((rxbuf[5] >> 1) & 1);

				app_nunchuk_update_output(&chuck_d_tmp);
			}

			if (timeout_has_timeout()) {
				chuck_error = 1;
			}
		} else {
			chuck_error = 2;
			hw_try_restore_i2c();
			chThdSleepMilliseconds(100);
		}

		chThdSleepMilliseconds(10);
	}
}

static THD_FUNCTION(output_thread, arg) {
	(void)arg;

	chRegSetThreadName("Nunchuk output");

	for(;;) {
		chThdSleepMilliseconds(OUTPUT_ITERATION_TIME_MS);

		if (timeout_has_timeout() || chuck_error != 0 || config.ctrl_type == CHUK_CTRL_TYPE_NONE) {
			continue;
		}

		static bool is_reverse = false;
		static bool was_z = false;
		const float current_now = mc_interface_get_tot_current_directional_filtered();
		static float prev_current = 0.0;

		if (chuck_d.bt_c && chuck_d.bt_z) {
			led_external_set_state(LED_EXT_BATT);
			continue;
		}

		if ((config.buttons_mirrored ? chuck_d.bt_c : chuck_d.bt_z) && !was_z && (config.ctrl_type == CHUK_CTRL_TYPE_CURRENT || config.ctrl_type == CHUK_CTRL_TYPE_WATT) &&
				fabsf(current_now) < MAX_CURR_DIFFERENCE) {
			if (is_reverse) {
				is_reverse = false;
			} else {
				is_reverse = true;
			}
		}

		was_z = config.buttons_mirrored ? chuck_d.bt_c : chuck_d.bt_z;

		led_external_set_reversed(is_reverse);

		float out_val = app_nunchuk_get_decoded_chuk();
		utils_deadband(&out_val, config.hyst, 1.0);
		
		if (throt_config.adjustable_throttle_enabled && out_val != 0.0){
			if (out_val > 0.0) {
				out_val = calculate_throttle_curve_chuk(px, py, throt_config.bezier_reduce_factor, out_val);
			} else {
				out_val = -calculate_throttle_curve_chuk(nx, ny, throt_config.bezier_neg_reduce_factor, -out_val);
			}
		}

		// LEDs
		float x_axis = ((float)chuck_d.js_x - 128.0) / 128.0;
		if (out_val < -0.001) {
			if (x_axis < -0.4) {
				led_external_set_state(LED_EXT_BRAKE_TURN_LEFT);
			} else if (x_axis > 0.4) {
				led_external_set_state(LED_EXT_BRAKE_TURN_RIGHT);
			} else {
				led_external_set_state(LED_EXT_BRAKE);
			}
		} else {
			if (x_axis < -0.4) {
				led_external_set_state(LED_EXT_TURN_LEFT);
			} else if (x_axis > 0.4) {
				led_external_set_state(LED_EXT_TURN_RIGHT);
			} else {
				led_external_set_state(LED_EXT_NORMAL);
			}
		}

		// If c is pressed and no throttle is used, maintain the current speed with PID control
		static bool was_pid = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();

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

		if (config.buttons_mirrored ? chuck_d.bt_z : chuck_d.bt_c) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				pid_rpm = rpm_filtered;

				if ((is_reverse && pid_rpm > 0.0) || (!is_reverse && pid_rpm < 0.0)) {
					if (fabsf(pid_rpm) > mcconf->s_pid_min_erpm) {
						// Abort if the speed is too high in the opposite direction
						continue;
					} else {
						pid_rpm = 0.0;
					}
				}

				was_pid = true;
			} else {
				if (is_reverse) {
					if (pid_rpm > 0.0) {
						pid_rpm = 0.0;
					}

					pid_rpm -= (out_val * config.stick_erpm_per_s_in_cc) / ((float)OUTPUT_ITERATION_TIME_MS * 1000.0);
				} else {
					if (pid_rpm < 0.0) {
						pid_rpm = 0.0;
					}

					pid_rpm += (out_val * config.stick_erpm_per_s_in_cc) / ((float)OUTPUT_ITERATION_TIME_MS * 1000.0);
				}
			}

			// NEW
			switch (config.ctrl_type) {
			case CHUK_CTRL_TYPE_CURRENT:
			case CHUK_CTRL_TYPE_CURRENT_NOREV:
				mc_interface_set_pid_speed(pid_rpm);
				break;
			case CHUK_CTRL_TYPE_WATT:
			case CHUK_CTRL_TYPE_WATT_NOREV:
				mc_interface_set_pid_speed(pid_rpm);
				break;
			default:
				break;
			}
			// END NEW
			
			// Send the same duty cycle to the other controllers
			if (config.multi_esc) {
				float duty = mc_interface_get_duty_cycle_now();

				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_duty(msg->id, duty);
					}
				}
			}

			// Set the previous ramping current to not get a spike when releasing
			// PID control and to get a smooth transition.
			prev_current = current_now;

			continue;
		}

		was_pid = false;

		float current = 0;
		
		// NEW
		switch (config.ctrl_type) {
		case CHUK_CTRL_TYPE_CURRENT:
		case CHUK_CTRL_TYPE_CURRENT_NOREV:
			if (out_val >= 0.0) {
				current = out_val * mcconf->l_current_max;
			} else {
				current = out_val * fabsf(mcconf->l_current_min);
			}
			break;
		case CHUK_CTRL_TYPE_WATT:
		case CHUK_CTRL_TYPE_WATT_NOREV:
			if (out_val >= 0.0) {
				if (mcconf->use_max_watt_limit) {
					current = utils_smallest_of_2(out_val * mcconf->l_current_max, 
							out_val * (mcconf->watts_max / GET_INPUT_VOLTAGE() / mc_interface_get_duty_cycle_for_watt_calculation()));
				} else {
					current = utils_smallest_of_2(out_val * mcconf->l_current_max, 
							out_val * mcconf->l_in_current_max / mc_interface_get_duty_cycle_for_watt_calculation());
				}
			} else {
				current = out_val * fabsf(mcconf->l_current_min);
			}
			break;
		default:
			break;
		}
		// END NEW 

		// Find lowest RPM and highest current
		float rpm_local = mc_interface_get_rpm();
		if (is_reverse) {
			rpm_local = -rpm_local;
		}

		float rpm_lowest = rpm_local;
		float current_highest_abs = current_now;

		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					float rpm_tmp = msg->rpm;
					if (is_reverse) {
						rpm_tmp = -rpm_tmp;
					}

					if (rpm_tmp < rpm_lowest) {
						rpm_lowest = rpm_tmp;
					}

					// Make the current directional
					float msg_current = msg->current;
					if (msg->duty < 0) {
						msg_current = -msg_current;
					}

					if (fabsf(msg_current) > fabsf(current_highest_abs)) {
						current_highest_abs = msg_current;
					}
				}
			}
		}
		
		// Apply ramping
		const float current_range = mcconf->l_current_max + fabsf(mcconf->l_current_min);
		const float ramp_time = fabsf(current) > fabsf(prev_current) ? config.ramp_time_pos : config.ramp_time_neg;
			
		if (ramp_time > 0.01 && current != 0.0) {
			
            if (prev_current * current < 0.0) {
                prev_current = 0.0;
            }
			
			const float ramp_step = ((float)OUTPUT_ITERATION_TIME_MS * current_range) / (ramp_time * 1000.0);

			float current_goal = prev_current;
			const float goal_tmp = current_goal;
			utils_step_towards(&current_goal, current, ramp_step);
			bool is_decreasing = current_goal < goal_tmp;

			// Make sure the desired current is close to the actual current to avoid surprises
			// when changing direction
			float goal_tmp2 = current_goal;
			if (is_reverse) {
				if (fabsf(current_goal + current_highest_abs) > MAX_CURR_DIFFERENCE) {
					utils_step_towards(&goal_tmp2, -current_highest_abs, 2.0 * ramp_step);
				}
			} else {
				if (fabsf(current_goal - current_highest_abs) > MAX_CURR_DIFFERENCE) {
					utils_step_towards(&goal_tmp2, current_highest_abs, 2.0 * ramp_step);
				}
			}

			// Always allow negative ramping
			bool is_decreasing2 = goal_tmp2 < current_goal;
			if ((!is_decreasing || is_decreasing2) && fabsf(out_val) > 0.001) {
				current_goal = goal_tmp2;
			}

			out_val = out_val / current * current_goal;
						
			current = current_goal;
		}

		prev_current = current;
		
		if (current < 0.0) {
			
			mc_interface_set_brake_current(current);

			if (config.multi_esc) {
				// Send brake command to all ESCs seen recently on the CAN bus
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						if (config.ctrl_type == CHUK_CTRL_TYPE_WATT || config.ctrl_type == CHUK_CTRL_TYPE_WATT_NOREV) {
							comm_can_set_brake_servo(msg->id, out_val);
						} else {
							comm_can_set_current_brake(msg->id, current);
						}
					}
				}
			}
		} else {
						
			bool use_min_current = false;
			// Apply soft RPM limit
			if (rpm_lowest > config.rpm_lim_end) {
				if (out_val > 0.0) {
					use_min_current = true;
					out_val = 0.000001; // make sure min current ius used
				}
				if (current > 0.0) {
					current = mcconf->cc_min_current;	
				}
			} else if (rpm_lowest > config.rpm_lim_start) {
				if (out_val > 0.0) {
					use_min_current = true;
					out_val = utils_map(rpm_lowest, config.rpm_lim_start, config.rpm_lim_end, out_val, 0.000001);
				}
				if (current > 0.0) {
					current = utils_highest_of_2(utils_map(rpm_lowest, config.rpm_lim_start, config.rpm_lim_end, current, 0.0), mcconf->cc_min_current);
				}
			}

			float current_out = current;
			float servo_val_out = out_val;

			// Traction control
			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						if (config.tc) {
							float rpm_tmp = msg->rpm;
							if (is_reverse) {
								rpm_tmp = -rpm_tmp;
							}

							float diff = rpm_tmp - rpm_lowest;
							
							if (diff > config.tc_offset) {
								current_out = utils_map(diff - config.tc_offset, 0.0, config.tc_max_diff - config.tc_offset, current, 0.0);
								servo_val_out = utils_map(diff - config.tc_offset, 0.0, config.tc_max_diff - config.tc_offset, out_val, 0.0);
							} else {
								current_out = current;
								servo_val_out = out_val;
							}
						}

						if (is_reverse) {
							if (config.ctrl_type == CHUK_CTRL_TYPE_WATT || config.ctrl_type == CHUK_CTRL_TYPE_WATT_NOREV) {
								comm_can_set_servo(msg->id, -servo_val_out, use_min_current);
							} else {
								comm_can_set_current(msg->id, -current_out);
							}
						} else {
							if (config.ctrl_type == CHUK_CTRL_TYPE_WATT || config.ctrl_type == CHUK_CTRL_TYPE_WATT_NOREV) {
								comm_can_set_servo(msg->id, servo_val_out, use_min_current);
							} else {
								comm_can_set_current(msg->id, current_out);
							}
						}
					}
				}

				if (config.tc) {
					float diff = rpm_local - rpm_lowest;
					
					if (diff > config.tc_offset) {
						current_out = utils_map(diff - config.tc_offset, 0.0, config.tc_max_diff - config.tc_offset, current, 0.0);
					} else {
						current_out = current;
					}
				}
			}

			if (is_reverse) {
				mc_interface_set_current(-current_out);
			} else {
				mc_interface_set_current(current_out);
			}
		}
	}
}
