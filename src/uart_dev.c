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


#define UART_DMA_START_CHAN 0
#define UART_PORTS MCU_UART_PORTS

#if MCU_UART_PORTS > 0

#define UART_CIRCULAR_BUF_SIZE 65

typedef struct {
	lpuart_handle_t hal_handle;
	LPUART_Type * instance;
	mcu_event_handler_t read_handler;
	mcu_event_handler_t write_handler;
	u8 ref_count;
	const uart_attr_t * attr;
	u8 o_flags;
	u8 circularbuf[UART_CIRCULAR_BUF_SIZE];
} uart_local_t;

static uart_local_t m_uart_local[UART_PORTS] MCU_SYS_MEM;
LPUART_Type * const uart_regs_table[UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[UART_PORTS] = MCU_UART_IRQS;

static void exec_readcallback(uart_local_t * uart, u32 o_events);
static void exec_writecallback(uart_local_t * uart, u32 o_events);

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(uart, UART_VERSION, UART_IOC_IDENT_CHAR, I_MCU_TOTAL + I_UART_TOTAL, mcu_uart_get, mcu_uart_put, mcu_uart_flush)

static void lpuart_xfer_callback(LPUART_Type *instance, lpuart_handle_t *handle, status_t status, void *userData){
	uart_local_t *uart = userData;
	int ret = 1;

	if (kStatus_LPUART_TxIdle == status)
	{
		ret = devfs_execute_event_handler(&uart->write_handler, MCU_EVENT_FLAG_WRITE_COMPLETE, NULL);

		if (ret == 0) {
			uart->write_handler.callback = NULL;
		}
	}

	if (kStatus_LPUART_RxIdle == status || kStatus_LPUART_IdleLineDetected == status)
	{
		ret = devfs_execute_event_handler(&uart->read_handler, MCU_EVENT_FLAG_DATA_READY, NULL);

		if (ret == 0) {
			uart->read_handler.callback = NULL;
		}
	}

	if (kStatus_LPUART_RxHardwareOverrun == status || kStatus_LPUART_RxRingBufferOverrun == status)
	{
		// callback not expecting this: devfs_execute_event_handler(&uart->read_handler, MCU_EVENT_FLAG_OVERFLOW, NULL);
		mcu_debug_log_error(MCU_DEBUG_SYS, "%d - uart overflow", (LPUART_GetInstance(uart->instance) - 1));
	}
}

int mcu_uart_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	if ( local->ref_count == 0 ){
		local->instance = uart_regs_table[port];

		LPUART_TransferCreateHandle(local->instance, &(local->hal_handle), lpuart_xfer_callback, local);
		LPUART_TransferStartRingBuffer(local->instance, &(local->hal_handle), local->circularbuf, UART_CIRCULAR_BUF_SIZE);

		cortexm_enable_irq(uart_irqs[port]);

	}
	local->ref_count++;

	return 0;
}

int mcu_uart_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			cortexm_disable_irq(uart_irqs[port]);

			LPUART_TransferStopRingBuffer(local->instance, &(local->hal_handle));
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
	//DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	uart_info_t * info = ctl;

	info->o_flags = UART_FLAG_IS_PARITY_NONE |
			UART_FLAG_IS_PARITY_ODD |
			UART_FLAG_IS_PARITY_EVEN |
			UART_FLAG_IS_STOP1 |
			UART_FLAG_IS_STOP2;

	return 0;
}

/* Get debug console frequency. Copied from NXP SDK interrupt_transfer example. */
static uint32_t BOARD_DebugConsoleSrcFreq(void)
{
    uint32_t freq;

    /* To make it simple, we assume default PLL and divider settings, and the only variable
       from application is use PLL3 source or OSC source */
    if (CLOCK_GetMux(kCLOCK_UartMux) == 0) /* PLL3 div6 80M */
    {
        freq = (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }
    else
    {
        freq = CLOCK_GetOscFreq() / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }

    return freq;
}

int mcu_uart_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
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

		// idle period dynamic based on baud to prevent OS reading one byte at a time before
		// complete asynchronous message is received
		uart_config.rxIdleType = kLPUART_IdleTypeStopBit;
		if (freq < 115200) {
			uart_config.rxIdleConfig = kLPUART_IdleCharacter1;
		} else if (freq < 230400) {
			uart_config.rxIdleConfig = kLPUART_IdleCharacter4;
		} else if (freq < 460800) {
			uart_config.rxIdleConfig = kLPUART_IdleCharacter8;
		} else {
			uart_config.rxIdleConfig = kLPUART_IdleCharacter16;
		}

		uart_config.baudRate_Bps = freq;

		if( local->attr->width == 7 ){
			uart_config.dataBitsCount = kLPUART_SevenDataBits;
		}

		if( o_flags & UART_FLAG_IS_STOP2 ){
			uart_config.stopBitCount = kLPUART_TwoStopBit;
		}

		if( o_flags & UART_FLAG_IS_PARITY_EVEN ){
			uart_config.parityMode = kLPUART_ParityEven;
		} else if( o_flags & UART_FLAG_IS_PARITY_ODD ){
			uart_config.parityMode = kLPUART_ParityOdd;
		}

		uart_config.txFifoWatermark = 1;
		uart_config.rxFifoWatermark = 1;

		uart_config.enableTx = true;
		uart_config.enableRx = true;

		//pin assignments
#if 0 //FIXME: needs implemented on RT. Use BOARD_InitPins() from SDK for now.
		if( mcu_set_pin_assignment(
				 &(local->attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
				 CORE_PERIPH_UART, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}
#endif

		if( LPUART_Init(local->instance, &uart_config, BOARD_DebugConsoleSrcFreq()) != kStatus_Success ) {
			return SYSFS_SET_RETURN(EIO);
		}
	}

	return 0;
}

static void exec_readcallback(uart_local_t * uart, u32 o_events){
	devfs_execute_event_handler(&uart->read_handler, o_events, NULL);
}

static void exec_writecallback(uart_local_t * uart, u32 o_events){
	devfs_execute_event_handler(&uart->write_handler, o_events, NULL);
}

int mcu_uart_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	mcu_action_t * action = (mcu_action_t*)ctl;
	const enum _lpuart_interrupt_enable rx_inten = (kLPUART_RxDataRegFullInterruptEnable |
			kLPUART_IdleLineInterruptEnable | kLPUART_RxOverrunInterruptEnable |
			kLPUART_NoiseErrorInterruptEnable | kLPUART_FramingErrorInterruptEnable |
			kLPUART_ParityErrorInterruptEnable | kLPUART_RxFifoUnderflowInterruptEnable);
	const enum _lpuart_interrupt_enable tx_inten = (kLPUART_TxDataRegEmptyInterruptEnable |
			kLPUART_TxFifoOverflowInterruptEnable);
	enum _lpuart_interrupt_enable inten = 0; // OR'd _lpuart_interrupt_enable flags


	if( action->handler.callback == 0 ){
		//if there is an ongoing operation -- cancel it

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			LPUART_TransferAbortReceive(local->instance, &local->hal_handle);

			exec_readcallback(local, MCU_EVENT_FLAG_CANCELED);
			local->read_handler.callback = NULL;

			inten |= rx_inten;
		}

		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			LPUART_TransferAbortSend(local->instance, &local->hal_handle);
			exec_writecallback(local, MCU_EVENT_FLAG_CANCELED);
			local->write_handler.callback = NULL;

			inten |= tx_inten;
		}

		LPUART_DisableInterrupts(local->instance, inten);
	} else {
		if( cortexm_validate_callback(action->handler.callback) < 0 ){
			return SYSFS_SET_RETURN(EPERM);
		}

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY ){
			memcpy(&local->read_handler, &action->handler, sizeof(mcu_event_handler_t));

			inten |= rx_inten;
		}

		if ( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE ){
			memcpy(&local->write_handler, &action->handler, sizeof(mcu_event_handler_t));

			inten |= tx_inten;
		}

		LPUART_EnableInterrupts(local->instance, inten);
	}

	cortexm_set_irq_priority(uart_irqs[port], action->prio, action->o_events);
	return 0;
}

int mcu_uart_put(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
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
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	u8 * dest = ctl;
	if( dest == 0 ){ return SYSFS_SET_RETURN(EINVAL); }

	/* See if there's something to read out of circular buffer */
	size_t count = LPUART_TransferGetRxRingBufferLength(local->instance, &local->hal_handle);
	if (count == 0) {
		return SYSFS_SET_RETURN(ENODATA);
	}

	/* Get one byte to return */
	lpuart_transfer_t myxfer;
	myxfer.data = dest;
	myxfer.dataSize = 1;
	local->hal_handle.callback = NULL; // We don't want recursive callbacks, so disable for this call
	status_t ret = LPUART_TransferReceiveNonBlocking(local->instance, &local->hal_handle, &myxfer, NULL);
	local->hal_handle.callback = lpuart_xfer_callback;
	if (ret != kStatus_Success ){
		return SYSFS_SET_RETURN(EIO);
	}

	return 0;
}

int mcu_uart_getall(const devfs_handle_t * handle, void * ctl){
	return mcu_uart_get(handle, ctl); //only has a one byte buffer
}

int mcu_uart_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	//FIXME: behavior if (!(async->flags & O_NONBLOCK))??

	/* see if there is a transfer in progress, i.e. read handler is already in place */
	uint32_t count;
	status_t ret = LPUART_TransferGetReceiveCount(local->instance, &local->hal_handle, &count);
	if (ret == kStatus_NoTransferInProgress) {
		// validate and register callback if we can't block
		if( cortexm_validate_callback(async->handler.callback) < 0 ){
			return SYSFS_SET_RETURN(EPERM);
		}

		memcpy(&local->read_handler, &async->handler, sizeof(mcu_event_handler_t));

		lpuart_transfer_t myxfer;
		myxfer.data = async->buf;
		myxfer.dataSize = async->nbyte;
		size_t received;
		if( LPUART_TransferReceiveNonBlocking(local->instance, &local->hal_handle, &myxfer, &received) != kStatus_Success ){
			local->read_handler.callback = NULL;
			return SYSFS_SET_RETURN(EIO);
		}

		if (received == async->nbyte) {
			return received;
		}

		return SYSFS_SET_RETURN(EAGAIN); // caller needs to wait for async callback for complete data,
		                                 // even though received > 0 indicates partial data has been copied
	}

	return SYSFS_SET_RETURN(EIO);

}


int mcu_uart_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	if (local->write_handler.callback) {
		return SYSFS_SET_RETURN(EBUSY);
	}

	if (async->nbyte == 0) {
		return 0;
	}

	memcpy(&local->write_handler, &async->handler, sizeof(mcu_event_handler_t));

	lpuart_transfer_t myxfer;
	myxfer.data = async->buf;
	myxfer.dataSize = async->nbyte;
	status_t ret = LPUART_TransferSendNonBlocking(local->instance, &local->hal_handle, &myxfer);
	if( ret == kStatus_Success ){
		return 0;
	} else if ( ret == kStatus_LPUART_TxBusy ) {
		return SYSFS_SET_RETURN(EAGAIN);
	}

	return SYSFS_SET_RETURN(EIO);
}



void mcu_uart_isr(int port){
	uart_local_t * local = m_uart_local + port;
	LPUART_TransferHandleIRQ(local->instance, &local->hal_handle);

	enum _lpuart_flags unhandled_flags = LPUART_GetStatusFlags(local->instance);
	enum _lpuart_flags err_flags = (kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
	if (unhandled_flags & err_flags) {
		mcu_debug_log_error(MCU_DEBUG_SYS, "%d - uart error", port);
  }

	LPUART_ClearStatusFlags(local->instance, unhandled_flags);
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
