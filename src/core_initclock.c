/* Copyright 2011-2016 Tyler Gilbert;
 * This file is part of Stratify OS.
 *
 * Stratify OS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Stratify OS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#include "imxrt_local.h"


void sos_led_root_enable(void * args);

//requires mcu_core_osc_freq, mcu_board_config.core_cpu_freq, and mcu_board_config.core_periph_freq to be defined ext
int mcu_core_initclock(int div){

	//BSP event needs to configure the clocks
	mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_INITIALIZE_CLOCK, 0);

	//configure the systick
	cortexm_set_systick_reload(0x00ffffff);
	cortexm_start_systick();

	return 0;
}

/*! @} */
