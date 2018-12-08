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

#include <mcu/spi.h>
#include "spi_local.h"

#if MCU_SPI_PORTS > 0

LPSPI_Type * const spi_regs[MCU_SPI_PORTS] = MCU_SPI_REGS;
u8 const spi_irqs[MCU_SPI_PORTS] = MCU_SPI_IRQS;
spi_local_t spi_local[MCU_SPI_PORTS] MCU_SYS_MEM;

int spi_local_open(const devfs_handle_t * handle){
	if( handle->port < MCU_SPI_PORTS ){
		spi_local_t * local = spi_local + handle->port;
		if ( local->ref_count == 0 ){
			//turn on clock
			
		}
		local->transfer_handler.read = NULL;
		local->transfer_handler.write = NULL;
		cortexm_enable_irq(spi_irqs[handle->port]);
		local->ref_count++;
		return 0;
	}
	return SYSFS_SET_RETURN(EINVAL);
}

int spi_local_close(const devfs_handle_t * handle){
	spi_local_t * local = spi_local + handle->port;
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){

			//De Init and power down the SPI

			cortexm_disable_irq(spi_irqs[handle->port]);
			devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN(EDEADLK), MCU_EVENT_FLAG_CANCELED);

			//turn off RCC clock

		}
		local->ref_count--;
	}
	return 0;
}

int spi_local_setattr(const devfs_handle_t * handle, void * ctl){
	const u32 port = handle->port;
	spi_local_t * local = spi_local + port;

	u32 pclk;
	u32 prescalar;
	lpspi_master_config_t master_config;

	const spi_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		//neither application or BSP has supplied spi attributes
		return SYSFS_SET_RETURN(EINVAL);
	}

	u32 o_flags = attr->o_flags;

	local->o_flags = 0;

	if( o_flags & SPI_FLAG_SET_MASTER ){

		// this needs to convert spi_attr_t to lpspi_master_config_t

		if( attr->freq == 0 ){
			//max speed

		} else {
			//get as close to attr->freq withouth going over
		}

		master_config.cpol = 0;
		master_config.cpha = 0;
		if( o_flags & SPI_FLAG_IS_MODE1 ){

		} else if( o_flags & SPI_FLAG_IS_MODE2 ){

		} else if( o_flags & SPI_FLAG_IS_MODE3 ){

		}

		//todo -- make sure attr->width is valid
		master_config.bitsPerFrame = attr->width;

		if( o_flags & SPI_FLAG_IS_FORMAT_TI ){

		} else {

		}

		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(spi_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(spi_pin_assignment_t),
				 CORE_PERIPH_SPI, handle->port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		//LPSPI_MasterInit(spi_regs[handle->port], &master_config, 0);
	}

	if( o_flags & SPI_FLAG_IS_FULL_DUPLEX ){
		local->o_flags |= SPI_LOCAL_IS_FULL_DUPLEX;
	} else {
		local->o_flags &= ~SPI_LOCAL_IS_FULL_DUPLEX;
	}

	return 0;
}

int spi_local_swap(const devfs_handle_t * handle, void * ctl){
	const u32 port = handle->port;
	spi_local_t * local = spi_local + port;

	//swap a byte and return synchronously
	//LPSPI_MasterTransferBlocking() with just one byte
	return 0;
}

int spi_local_setaction(const devfs_handle_t * handle, void * ctl){
	mcu_action_t * action = (mcu_action_t*)ctl;
	const u32 port = handle->port;
	spi_local_t * local = spi_local + port;

	//when callback is 0, this needs to cancel any ongoing operations
	if(action->handler.callback == 0){
		if (action->o_events & MCU_EVENT_FLAG_DATA_READY){
			devfs_execute_read_handler(
						&local->transfer_handler, 0,
						SYSFS_SET_RETURN(EINTR),
						MCU_EVENT_FLAG_CANCELED);
		}

		if (action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE){
			devfs_execute_write_handler(
						&local->transfer_handler, 0,
						SYSFS_SET_RETURN(EINTR),
						MCU_EVENT_FLAG_CANCELED);
		}
	} else {
		//not supported to directly manage SPI actions
		return SYSFS_SET_RETURN(ENOTSUP);
	}

	//handles adjusting the interrupt priority
	cortexm_set_irq_priority(spi_irqs[handle->port], action->prio, action->o_events);
	return 0;
}

void spi_local_master_transfer_complete_callback(LPSPI_Type *base,
																 lpspi_master_handle_t *handle,
																 status_t status,
																 void *userData){
	//execute the handler
	spi_local_t * local = (spi_local_t*)handle;

	//decode the status use devfs_execute_cancel_handler() for an error
	devfs_execute_cancel_handler(&local->transfer_handler, 0, SYSFS_SET_RETURN_WITH_VALUE(EIO, 0), MCU_EVENT_FLAG_ERROR);

	devfs_execute_write_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);
	devfs_execute_read_handler(&local->transfer_handler, 0, 0, MCU_EVENT_FLAG_DATA_READY);

}

//these handlers need to move to the local file
void mcu_core_lpspi1_isr(){
	//LPSPI_MasterTransferHandleIRQ();
}

void mcu_core_lpspi2_isr(){
	//LPSPI_MasterTransferHandleIRQ();
}

void mcu_core_lpspi3_isr(){
	//LPSPI_MasterTransferHandleIRQ();
}

void mcu_core_lpspi4_isr(){
	//LPSPI_MasterTransferHandleIRQ();
}


#endif

