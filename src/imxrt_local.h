/* Copyright 2011-2017 Tyler Gilbert;
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

#ifndef IMXRT_LOCAL_H_
#define IMXRT_LOCAL_H_

#include <mcu/types.h>
#include <mcu/core.h>

#define USE_FULL_LL_DRIVER

#include "imxrt_arch.h"

#include <fcntl.h>
#include <cortexm/cortexm.h>
#include <mcu/core.h>
#include <mcu/pio.h>
#include <mcu/debug.h>

#include "MIMXRT1052/fsl_lpi2c.h"
#include "MIMXRT1052/fsl_lpspi.h"
#include "MIMXRT1052/fsl_lpuart.h"
#include "MIMXRT1052/fsl_gpt.h"
#include "MIMXRT1052/fsl_gpio.h"
#include "MIMXRT1052/fsl_semc.h"
#include "MIMXRT1052/fsl_flexram.h"

int hal_set_alternate_pin_function(mcu_pin_t pin, core_periph_t function, int periph_port, int mode, int speed, int pull);
GPIO_Type * const hal_get_pio_regs(u8 port);
int hal_get_alternate_function(int gpio_port, int pin, core_periph_t function, int periph_port);


#endif /* STM32_LOCAL_H_ */
