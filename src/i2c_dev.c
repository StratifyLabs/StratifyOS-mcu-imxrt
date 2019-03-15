/* Copyright 2011-2018 Tyler Gilbert;
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

#include <errno.h>
#include <fcntl.h>
#include "imxrt_local.h"
#include "cortexm/cortexm.h"
#include "mcu/i2c.h"
#include "mcu/debug.h"
#include "mcu/core.h"
#include "mcu/pio.h"

#if MCU_I2C_PORTS > 0

typedef struct MCU_PACK {
	lpi2c_master_handle_t hal_handle;
	lpi2c_master_transfer_t hal_transfer;
	devfs_transfer_handler_t transfer_handler;
	LPI2C_Type * instance;
	u32 o_flags;
	u16 err;
	u16 slave_register_pointer;
	u16 ref_count;
} i2c_local_t;

static i2c_local_t m_i2c_local[MCU_I2C_PORTS] MCU_SYS_MEM;
static LPI2C_Type * const i2c_regs_table[MCU_I2C_PORTS] = MCU_I2C_REGS;
static u8 const i2c_irqs[MCU_I2C_PORTS] = MCU_I2C_IRQS;

static void i2c_handle_transfer_callback(LPI2C_Type *base,
													  lpi2c_master_handle_t *handle,
													  status_t completionStatus,
													  void *userData);

#if 0
typedef struct {
	u8 port;
	u8 is_pullup;
} post_configure_pin_t;
static void post_configure_pin(const mcu_pin_t * pin, void* arg);
#endif

DEVFS_MCU_DRIVER_IOCTL_FUNCTION_MIN(i2c, I2C_VERSION, I2C_IOC_IDENT_CHAR)

int mcu_i2c_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(i2c, MCU_I2C_PORTS);
	if ( local->ref_count == 0 ){

		local->transfer_handler.read = 0;
		local->transfer_handler.write = 0;
		local->instance = i2c_regs_table[port];
		cortexm_enable_irq(i2c_irqs[port]);
	}
	local->ref_count++;
	return 0;
}

int mcu_i2c_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(i2c, MCU_I2C_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EINTR), 0);
			local->instance = 0;
			cortexm_disable_irq(i2c_irqs[port]);


		}
		local->ref_count--;
	}
	return 0;
}

int mcu_i2c_getinfo(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	i2c_info_t * info = ctl;

	info->err = m_i2c_local[port].err;
	info->o_flags = I2C_FLAG_SET_MASTER |
			I2C_FLAG_SET_SLAVE |
			I2C_FLAG_PREPARE_PTR_DATA |
			I2C_FLAG_PREPARE_DATA |
			I2C_FLAG_IS_PULLUP |
			I2C_FLAG_IS_SLAVE_ADDR0 |
			I2C_FLAG_IS_SLAVE_ADDR1 |
			I2C_FLAG_IS_SLAVE_ADDR2 |
			I2C_FLAG_IS_SLAVE_ADDR3 |
			I2C_FLAG_RESET;

	info->freq = 400000;

	info->o_events = MCU_EVENT_FLAG_WRITE_COMPLETE |
			MCU_EVENT_FLAG_DATA_READY |
			MCU_EVENT_FLAG_CANCELED |
			MCU_EVENT_FLAG_ERROR;
	return 0;
}

int mcu_i2c_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(i2c, MCU_I2C_PORTS);
	u32 freq;

	lpi2c_master_config_t master_config;

	const i2c_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	u32 o_flags = attr->o_flags;

	if( freq == 0 ){
		freq = 100000;
	}

	if( freq > 400000 ){
		freq = 400000;
	}

	if( o_flags & (I2C_FLAG_SET_MASTER | I2C_FLAG_SET_SLAVE) ){
		//Init init structure with defaults

	}

	if( o_flags & I2C_FLAG_SET_MASTER ){
		//set the clock speed

		if( attr->freq ==	0 ){
			master_config.baudRate_Hz = 400000UL;
		} else {
			master_config.baudRate_Hz = attr->freq;
		}

	} else if( o_flags & I2C_FLAG_SET_SLAVE ){

		if( o_flags & I2C_FLAG_IS_SLAVE_ADDR1 ){

		}

		if( o_flags & I2C_FLAG_IS_SLAVE_ACK_GENERAL_CALL ){

		}

	}

	if( o_flags & (I2C_FLAG_SET_MASTER | I2C_FLAG_SET_SLAVE) ){


		if( o_flags & I2C_FLAG_STRETCH_CLOCK ){

		}

		local->o_flags = o_flags;

		//force a start and stop condition to clear the busy bit
		mcu_select_pin_assignment(&attr->pin_assignment,
										  MCU_CONFIG_PIN_ASSIGNMENT(i2c_config_t, handle),
										  MCU_PIN_ASSIGNMENT_COUNT(i2c_pin_assignment_t));



		LPI2C_MasterTransferCreateHandle(local->instance, &local->hal_handle, i2c_handle_transfer_callback, local);
		LPI2C_MasterInit(local->instance, &master_config, 0);

	}

	if( o_flags & (I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_PREPARE_PTR|I2C_FLAG_PREPARE_DATA) ){
		local->o_flags = o_flags;
		local->hal_transfer.slaveAddress = attr->slave_addr[0].addr8[0];
	}

	if( o_flags & I2C_FLAG_RESET ){
		//force a reset of the I2C

	}

	return 0;
}


int mcu_i2c_geterr(const devfs_handle_t * handle, void * ctl){
	int port = handle->port;
	return m_i2c_local[port].err;
}

int mcu_i2c_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	int port = handle->port;

	cortexm_set_irq_priority(i2c_irqs[port], action->prio, action->o_events);

	if( action->handler.callback == 0 ){
		//i2c_local[port].slave.handler.callback = 0;
		//i2c_local[port].slave.handler.context = 0;
		return 0;
	}

	if( cortexm_validate_callback(action->handler.callback) < 0 ){
		return -1;
	}

	//i2c_local[port].slave.handler.callback = action->handler.callback;
	//i2c_local[port].slave.handler.context = action->handler.context;

	return 0;
}

int mcu_i2c_write(const devfs_handle_t * handle, devfs_async_t * async){
	i2c_local_t * local = m_i2c_local + handle->port;


	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);


	/*
	 * Depending on the mode this needs to hande a register write
	 * then a data write or just a data write.
	 *
	 * It needs to handle both 16-bit and 8-bit register addressing
	 *
	 *
	 */

	u8 pointer_size = 1;
	if( local->o_flags & I2C_FLAG_PREPARE_PTR_DATA ){
		//This means write the pointer register then the data
		if( local->o_flags & I2C_FLAG_IS_PTR_16 ){
			//this means the register pointer is 16-bit
			pointer_size = 2;
		} else {
			pointer_size = 1;
		}

	} else if( local->o_flags & I2C_FLAG_PREPARE_DATA ){
		//this means just write the data
		pointer_size = 0;
	}

	local->hal_transfer.direction = kLPI2C_Write;
	local->hal_transfer.flags = kLPI2C_TransferDefaultFlag;
	local->hal_transfer.subaddress = async->loc;
	local->hal_transfer.subaddressSize = pointer_size;
	local->hal_transfer.data = async->buf;
	local->hal_transfer.dataSize = async->nbyte;

	if( LPI2C_MasterTransferNonBlocking(local->instance, &local->hal_handle, &local->hal_transfer) == kStatus_Success ){
		//return 0 tells the device filesystem to block until async->handler is called
		return 0;
	}

	//if an error occurred, this will tell the device filesystem not to wait for the callback
	local->transfer_handler.write = 0;
	return SYSFS_SET_RETURN(EIO);
}

int mcu_i2c_read(const devfs_handle_t * handle, devfs_async_t * async){
	i2c_local_t * local = m_i2c_local + handle->port;


	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

	u8 pointer_size = 0;
	if( local->o_flags & I2C_FLAG_PREPARE_PTR_DATA ){
		//This means write the pointer register then the data
		if( local->o_flags & I2C_FLAG_IS_PTR_16 ){
			//this means the register pointer is 16-bit
			pointer_size = 2;
		} else {
			pointer_size = 1;
		}

	} else if( local->o_flags & I2C_FLAG_PREPARE_DATA ){
		//this means just read the data
		pointer_size = 0;
	}

	local->hal_transfer.direction = kLPI2C_Read;
	local->hal_transfer.flags = kLPI2C_TransferDefaultFlag;
	local->hal_transfer.subaddress = async->loc;
	local->hal_transfer.subaddressSize = pointer_size;
	local->hal_transfer.data = async->buf;
	local->hal_transfer.dataSize = async->nbyte;

	if( LPI2C_MasterTransferNonBlocking(local->instance, &local->hal_handle, &local->hal_transfer) == kStatus_Success ){
		//return 0 tells the device filesystem to block until async->handler is called
		return 0;
	}


	local->transfer_handler.read = 0;
	return SYSFS_SET_RETURN(EIO);
}

void i2c_handle_transfer_callback(LPI2C_Type *base,
											 lpi2c_master_handle_t *handle,
											 status_t completionStatus,
											 void *userData){
	i2c_local_t * local = userData;


	if( completionStatus == kStatus_Success ){

		//this is called when a read completes (after final STOP)
		if( local->hal_transfer.direction == kLPI2C_Read ){
			devfs_execute_read_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_DATA_READY);
		}

		//this is called when a write completes (after final STOP)
		if( local->hal_transfer.direction == kLPI2C_Write ){
			devfs_execute_write_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);
		}
	}

}


static void mcu_i2c_ev_isr(const u32 port) {
	i2c_local_t * local = m_i2c_local + port;
	LPI2C_MasterTransferHandleIRQ(local->instance, &local->hal_handle);
}


void mcu_core_lpi2c1_isr(){ mcu_i2c_ev_isr(0); }
void mcu_core_lpi2c2_isr(){ mcu_i2c_ev_isr(1); }
void mcu_core_lpi2c3_isr(){ mcu_i2c_ev_isr(2); }
void mcu_core_lpi2c4_isr(){ mcu_i2c_ev_isr(3); }


#endif

