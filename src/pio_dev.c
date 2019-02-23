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

#include <mcu/pio.h>
#include <mcu/core.h>

#include "imxrt_local.h"


#if MCU_PIO_PORTS > 0

typedef struct {
	GPIO_Type * instance;
	u8 ref_count;
	u8 pull_mode;
} pio_local_t;


static pio_local_t m_pio_local[MCU_PIO_PORTS] MCU_SYS_MEM;
static GPIO_Type * const m_pio_regs_table[MCU_PIO_PORTS] = MCU_PIO_REGS;
//static u8 const m_pio_irqs[MCU_PIO_PORTS] = MCU_PIO_IRQS;


//this function is used by other modules to access pio regs
GPIO_Type * const hal_get_pio_regs(u8 port){
	if( port < MCU_PIO_PORTS ){
		return m_pio_regs_table[port];
	}
	return 0;
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(pio, PIO_VERSION, PIO_IOC_IDENT_CHAR, I_MCU_TOTAL + I_PIO_TOTAL, mcu_pio_setmask, mcu_pio_clrmask, mcu_pio_get, mcu_pio_set)


int mcu_pio_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(pio, MCU_PIO_PORTS);
	if ( local->ref_count == 0 ){
		local->instance = m_pio_regs_table[handle->port];
	}
	local->ref_count++;
	return 0;
}

int mcu_pio_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(pio, MCU_PIO_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			//if there are any pending callbacks on interrupts execute them as cancelled

			local->instance = 0;
		}
		local->ref_count--;
	}
	return 0;
}

int mcu_pio_dev_is_powered(const devfs_handle_t * handle){
	int port = handle->port;
	if ( m_pio_local[port].ref_count > 0 ){
		return 1;
	}
	return 0;
}

int mcu_pio_write(const devfs_handle_t * handle, devfs_async_t * wop){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_pio_read(const devfs_handle_t * cfg, devfs_async_t * async){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_pio_setaction(const devfs_handle_t * handle, void * ctl){
	const mcu_action_t * action = ctl;

	//this is for setting up callbacks on edge interrupts


	return 0;
}


int mcu_pio_getinfo(const devfs_handle_t * handle, void * ctl){
	//This is to return some information about the driver
	//rather thatn the actual GPIO
	MCU_UNUSED_ARGUMENT(handle);
	pio_info_t * info = ctl;
	if( info == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

	//all flags that are actually supported by this driver should be set
	info->o_flags = PIO_FLAG_SET_INPUT |
			PIO_FLAG_SET_OUTPUT;

	//all flags that are actually supported by this driver should be set
	info->o_events = MCU_EVENT_FLAG_RISING;


	return 0;
}

int mcu_pio_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(pio, MCU_PIO_PORTS);
	pio_attr_t * attr;
	attr = ctl;
	gpio_pin_config_t pin_config;

	pin_config.interruptMode = kGPIO_NoIntmode;
	pin_config.outputLogic = 0;

	for(u32 i=0; i < 32; i++){
		if( attr->o_pinmask & (1<<i) ){
			if( attr->o_flags & PIO_FLAG_SET_INPUT ){
				pin_config.direction	= kGPIO_DigitalInput;
			} else {
				pin_config.direction	= kGPIO_DigitalOutput;
			}

			//pullup/down?

			if( local->instance == 0 ){
				local->instance = m_pio_regs_table[port];
			}

			pin_config.outputLogic = 0;
			//GPIO_PinInit(local->instance, i, &pin_config);
		}
	}

	return 0;
}

int mcu_pio_setmask(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(pio, MCU_PIO_PORTS);
	u32 o_pinmask = (u32)ctl;
	GPIO_PortSet(local->instance, o_pinmask);
	return 0;
}

int mcu_pio_clrmask(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(pio, MCU_PIO_PORTS);
	u32 o_pinmask = (u32)ctl;
	GPIO_PortClear(local->instance, o_pinmask);
	return 0;
}

int mcu_pio_get(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(pio, MCU_PIO_PORTS);
	u32 * value = ctl;
	if( value ){
		*value = local->instance->DR;
		return SYSFS_RETURN_SUCCESS;
	}
	return SYSFS_SET_RETURN(EINVAL);
}

int mcu_pio_set(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(pio, MCU_PIO_PORTS);
	local->instance->DR = (u32)ctl;
	return 0;
}



void mcu_core_gpio1_int0_isr(){

}

void mcu_core_gpio1_int1_isr(){

}

void mcu_core_gpio1_int2_isr(){

}

void mcu_core_gpio1_int3_isr(){

}

void mcu_core_gpio1_int4_isr(){

}

void mcu_core_gpio1_int5_isr(){

}

void mcu_core_gpio1_int6_isr(){

}

void mcu_core_gpio1_int7_isr(){

}

void mcu_core_gpio1_int8_isr(){

}

void mcu_core_gpio1_combined_0_15_isr(){

}

void mcu_core_gpio1_combined_16_31_isr(){

}

void mcu_core_gpio2_combined_0_15_isr(){

}

void mcu_core_gpio2_combined_16_31_isr(){

}

void mcu_core_gpio3_combined_0_15_isr(){

}

void mcu_core_gpio3_combined_16_31_isr(){

}


void mcu_core_gpio4_combined_0_15_isr(){

}

void mcu_core_gpio4_combined_16_31_isr(){

}

void mcu_core_gpio5_combined_0_15_isr(){

}

void mcu_core_gpio5_combined_16_31_isr(){

}






#endif
