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
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef IMXRT_ARCH_H_
#define IMXRT_ARCH_H_

#include <mcu/types.h>
#include <mcu/eth.h>
#include <mcu/spi.h>
#include <mcu/sdio.h>
#include <mcu/mmc.h>
#include <mcu/i2s.h>
#include <mcu/adc.h>
#include <mcu/dac.h>

#if defined __imxrt1052
#define CORE_M7 1
#define ARM_MATH_CM7 1
#if !defined MIMXRT1052CVJ5B
#define MIMXRT1052CVJ5B 1
#endif
#include "mcu_MIMXRT1052.h"
#endif


typedef struct MCU_PACK {
	u32 o_flags;
} imxrt_config_t;



#endif /* IMXRT_ARCH_H_ */
