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

#include <fcntl.h>
#include "imxrt_local.h"
#include "cortexm/cortexm.h"
#include "mcu/uart.h"
#include "mcu/pio.h"
#include "mcu/debug.h"
#include "mcu/core.h"
#include "fsl_lpuart.h"


#define UART_DMA_START_CHAN 0
#define UART_PORTS MCU_UART_PORTS

#if MCU_UART_PORTS > 0

enum {
	UART_LOCAL_FLAG_IS_INCOMING_ENABLED = (1<<0),
	UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE = (1<<1)
};

typedef struct {
	lpuart_handle_t hal_handle;
	LPUART_Type * instance;
	devfs_transfer_handler_t transfer_handler;
	u8 ref_count;
	const uart_attr_t * attr;
	u8 o_flags;
	lpuart_transfer_t incoming;
	u8 _incoming_data;
} uart_local_t;

static uart_local_t m_uart_local[UART_PORTS] MCU_SYS_MEM;
LPUART_Type * const uart_regs_table[UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[UART_PORTS] = MCU_UART_IRQS;

static void exec_readcallback(uart_local_t * uart, u32 o_events);
static void exec_writecallback(uart_local_t * uart, u32 o_events);

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(uart, UART_VERSION, UART_IOC_IDENT_CHAR, I_MCU_TOTAL + I_UART_TOTAL, mcu_uart_get, mcu_uart_put, mcu_uart_flush)

static void lpuart_xfer_callback(LPUART_Type *instance, lpuart_handle_t *handle, status_t status, void *userData){
	uart_local_t *uart = userData;

	if (kStatus_LPUART_TxIdle == status)
	{
		devfs_execute_write_handler(&uart->transfer_handler, NULL, 0, MCU_EVENT_FLAG_WRITE_COMPLETE);
	}

	if (kStatus_LPUART_RxIdle == status)
	{
		if( uart->o_flags & UART_LOCAL_FLAG_IS_INCOMING_ENABLED ){
			uart->o_flags |= UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE;
			LPUART_TransferReceiveNonBlocking(uart->instance, &uart->hal_handle, &uart->incoming, NULL);
		}

		devfs_execute_read_handler(&uart->transfer_handler, NULL, uart->incoming.dataSize, MCU_EVENT_FLAG_DATA_READY);
	}
}

int mcu_uart_open(const devfs_handle_t * handle){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);
	if ( local->ref_count == 0 ){
		local->instance = uart_regs_table[port];
		local->incoming.data = &local->_incoming_data;
		local->incoming.dataSize = 1;

		LPUART_TransferCreateHandle(local->instance, &(local->hal_handle), lpuart_xfer_callback, &local);

		cortexm_enable_irq(uart_irqs[port]);

	}
	local->ref_count++;

	return 0;
}

int mcu_uart_close(const devfs_handle_t * handle){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);

	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			cortexm_disable_irq(uart_irqs[port]);

			LPUART_Deinit(local->instance);
			local->instance = NULL;
		}
		local->ref_count--;
	}
	return 0;
}

int mcu_uart_dev_is_powered(const devfs_handle_t * handle){
	return ( m_uart_local[handle->port].ref_count != 0 );
}

int mcu_uart_getinfo(const devfs_handle_t * handle, void * ctl){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);

	uart_info_t * info = ctl;

	info->o_flags = UART_FLAG_IS_PARITY_NONE |
			UART_FLAG_IS_PARITY_ODD |
			UART_FLAG_IS_PARITY_EVEN |
			UART_FLAG_IS_STOP1 |
			UART_FLAG_IS_STOP2;

	return 0;
}

int mcu_uart_setattr(const devfs_handle_t * handle, void * ctl){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);
	u32 o_flags;
	u32 freq;

	lpuart_config_t uart_config;

	local->attr = mcu_select_attr(handle, ctl);
	if( local->attr == 0 ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	o_flags = local->attr->o_flags;

	if( o_flags & UART_FLAG_SET_LINE_CODING ){
		freq = local->attr->freq;
		if( freq == 0 ){
			freq = 115200;
		}

		LPUART_GetDefaultConfig(&uart_config);

		uart_config.baudRate_Bps = freq;

		if( local->attr->width == 7 ){
			uart_config.dataBitsCount = kLPUART_SevenDataBits;
		}

		if( o_flags & UART_FLAG_IS_STOP2 ){
			uart_config.stopBitCount = kLPUART_TwoStopBit;
		}

		uart_config.parityMode = kLPUART_ParityDisabled;
		if( o_flags & UART_FLAG_IS_PARITY_EVEN ){
			uart_config.parityMode = kLPUART_ParityEven;
		} else if( o_flags & UART_FLAG_IS_PARITY_ODD ){
			uart_config.parityMode = kLPUART_ParityOdd;
		}

		//FIXME: necessary to enable here? Or is it done upon *NonBlocking()?
		uart_config.enableTx = true;
		uart_config.enableRx = true;

		//pin assignments
		if( mcu_set_pin_assignment(
				 &(local->attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
				 CORE_PERIPH_UART, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		if( LPUART_Init(local->instance, &uart_config, BOARD_DebugConsoleSrcFreq()) != kStatus_Success ) {
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return 0;
}

static void exec_readcallback(uart_local_t * uart, u32 o_events){
	devfs_execute_read_handler(&(uart->transfer_handler), NULL, uart->incoming.dataSize, o_events);

	//if the callback is NULL now, disable the interrupt
	if( uart->transfer_handler.read == NULL ){

	}
}

static void exec_writecallback(uart_local_t * uart, u32 o_events){
	devfs_execute_write_handler(&(uart->transfer_handler), NULL, 0, o_events);

	//if the callback is NULL now, disable the interrupt
	if( uart->transfer_handler.write == NULL ){

	}
}

int mcu_uart_setaction(const devfs_handle_t * handle, void * ctl){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);
	mcu_action_t * action = (mcu_action_t*)ctl;

	if( action->handler.callback == 0 ){
		//if there is an ongoing operation -- cancel it

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			//execute the read callback if not null
			if( local->o_flags & UART_LOCAL_FLAG_IS_INCOMING_ENABLED ){
				local->o_flags &= ~UART_LOCAL_FLAG_IS_INCOMING_ENABLED;
				LPUART_TransferAbortReceive(local->instance, &local->hal_handle);
			}

			exec_readcallback(local, MCU_EVENT_FLAG_CANCELED);
			local->transfer_handler.read = NULL;
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			LPUART_TransferAbortSend(local->instance, &local->hal_handle);
			exec_writecallback(local, MCU_EVENT_FLAG_CANCELED);
			local->transfer_handler.write = NULL;
		}

	} else {

		if( cortexm_validate_callback(action->handler.callback) < 0 ){
			return SYSFS_SET_RETURN(EPERM);
		}

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			local->transfer_handler.read->handler = action->handler;

			//enable the receiver so that the action is called when a byte arrives
			local->o_flags |= UART_LOCAL_FLAG_IS_INCOMING_ENABLED;
			local->o_flags &= ~UART_LOCAL_FLAG_IS_INCOMING_AVAILABLE;
			if( LPUART_TransferReceiveNonBlocking(local->instance, &local->hal_handle, &local->incoming, NULL) != kStatus_Success ){
				return SYSFS_SET_RETURN(EIO);
			}
		}

		if ( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			local->transfer_handler.write->handler = action->handler;
		}
	}

	cortexm_set_irq_priority(uart_irqs[port], action->prio, action->o_events);
	return 0;
}

int mcu_uart_put(const devfs_handle_t * handle, void * ctl){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);
	u8 c = (u32)ctl;

	/* wait until there's room to write the data, since this call path has no async callback to use */
	LPUART_WriteBlocking(local->instance, &c, 1);

	return 0;
}

int mcu_uart_flush(const devfs_handle_t * handle, void * ctl){
	char c;
	mcu_uart_get(handle, &c);
	return 0;
}


int mcu_uart_get(const devfs_handle_t * handle, void * ctl){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);
	u8 * dest = ctl;
	if( dest == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

	/* wait until there's data, since this call path has no async callback to use */
	if( LPUART_ReadBlocking(local->instance, dest, 1) == kStatus_Success ){
		return 0;
	}

	return SYSFS_SET_RETURN(ENODATA);
}

int mcu_uart_getall(const devfs_handle_t * handle, void * ctl){
	return mcu_uart_get(handle, ctl); //only has a one byte buffer
}

int mcu_uart_read(const devfs_handle_t * handle, devfs_async_t * async){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);

	size_t read_cnt = async->nbyte;

	if ( async->flags & O_NONBLOCK ) {
		// make sure there's data before making a potentially blocking call
		uint32_t count;
		status_t ret = LPUART_TransferGetReceiveCount(local->instance, &local->hal_handle, &count);
		if (ret == kStatus_Success && count > 0) {
			read_cnt = MIN(read_cnt, count);
		} else if (ret == kStatus_NoTransferInProgress) {
			// validate and register callback if we can't block
			if( cortexm_validate_callback(async->handler.callback) < 0 ){
				return SYSFS_SET_RETURN(EPERM);
			}

			local->transfer_handler.read->handler = async->handler;
			lpuart_transfer_t myxfer;
			myxfer.data = async->buf;
			myxfer.dataSize = async->nbyte;
			if( LPUART_TransferReceiveNonBlocking(local->instance, &local->hal_handle, &myxfer, NULL) != kStatus_Success ){
				local->transfer_handler.read = 0;
				return SYSFS_SET_RETURN(EIO);
			}

			return 0;
		}
	}

	if( LPUART_ReadBlocking(local->instance, async->buf, read_cnt) != kStatus_Success ){
		return SYSFS_SET_RETURN(ENODATA);
	}

	//success
	return 0;
}


int mcu_uart_write(const devfs_handle_t * handle, devfs_async_t * async){
	DRIVER_ASSIGN_LOCAL(uart, MCU_UART_PORTS);

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	local->transfer_handler.write->handler = async->handler;
	lpuart_transfer_t myxfer;
	myxfer.data = async->buf;
	myxfer.dataSize = async->nbyte;
	status_t ret = LPUART_TransferSendNonBlocking(local->instance, &local->hal_handle, &myxfer);
	if( ret == kStatus_Success ){
		return 0;
	}

	return SYSFS_SET_RETURN(EIO);
}



void mcu_uart_isr(int port){
	uart_local_t * local = m_uart_local + port;
	LPUART_TransferHandleIRQ(local->instance, &local->hal_handle);
}

void mcu_core_lpuart1_isr(){ mcu_uart_isr(0); }
void mcu_core_lpuart2_isr(){ mcu_uart_isr(1); }
void mcu_core_lpuart3_isr(){ mcu_uart_isr(2); }
void mcu_core_lpuart4_isr(){ mcu_uart_isr(3); }
void mcu_core_lpuart5_isr(){ mcu_uart_isr(4); }
void mcu_core_lpuart6_isr(){ mcu_uart_isr(5); }
void mcu_core_lpuart7_isr(){ mcu_uart_isr(6); }
void mcu_core_lpuart8_isr(){ mcu_uart_isr(7); }





#endif
