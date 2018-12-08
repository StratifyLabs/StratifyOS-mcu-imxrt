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
} uart_local_t;

static uart_local_t m_uart_local[UART_PORTS] MCU_SYS_MEM;
LPUART_Type * const uart_regs_table[UART_PORTS] = MCU_UART_REGS;
u8 const uart_irqs[UART_PORTS] = MCU_UART_IRQS;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(uart, UART_VERSION, UART_IOC_IDENT_CHAR, I_MCU_TOTAL + I_UART_TOTAL, mcu_uart_get, mcu_uart_put, mcu_uart_flush)

int mcu_uart_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	if ( local->ref_count == 0 ){
		local->instance = uart_regs_table[port];
		//reset HAL UART
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
			local->instance = 0;
		}
		local->ref_count--;
	}
	return 0;
}

int mcu_uart_dev_is_powered(const devfs_handle_t * handle){
	return ( m_uart_local[handle->port].ref_count != 0 );
}

int mcu_uart_getinfo(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	uart_info_t * info = ctl;

	info->o_flags = UART_FLAG_IS_PARITY_NONE |
			UART_FLAG_IS_PARITY_ODD |
			UART_FLAG_IS_PARITY_EVEN |
			UART_FLAG_IS_STOP1 |
			UART_FLAG_IS_STOP2;

	return 0;
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


		if( local->attr->width == 9 ){
		}

		if( o_flags & UART_FLAG_IS_STOP2 ){
		}

		if( o_flags & UART_FLAG_IS_PARITY_EVEN ){

		} else if( o_flags & UART_FLAG_IS_PARITY_ODD ){

		}

		//pin assignments
		if( mcu_set_pin_assignment(
				 &(local->attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(uart_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(uart_pin_assignment_t),
				 CORE_PERIPH_UART, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}

		LPUART_Init(local->instance, &uart_config, 0);

	}

	return 0;
}

int mcu_uart_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	mcu_action_t * action = (mcu_action_t*)ctl;

	//if action->handler.callback == 0 then cancel ongoing operations

	cortexm_set_irq_priority(uart_irqs[port], action->prio, action->o_events);
	return 0;
}

int mcu_uart_put(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);
	u8 c = (u32)ctl;
	LPUART_WriteByte(local->instance, c);
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

	*dest = LPUART_ReadByte(local->instance);

	return SYSFS_SET_RETURN(ENODATA);
}

int mcu_uart_getall(const devfs_handle_t * handle, void * ctl){
	return mcu_uart_get(handle, ctl); //only has a one byte buffer
}

int mcu_uart_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.read, async);


	//if failure set local->transfer_handler.write = 0 and return SYSFS_SET_RETURN(<error number>)

	//success
	return 0;
}


int mcu_uart_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(uart, MCU_UART_PORTS);

	DEVFS_DRIVER_IS_BUSY(local->transfer_handler.write, async);

	//LPUART_TransferSendNonBlocking()

	//if failure set local->transfer_handler.write = 0 and return SYSFS_SET_RETURN(<error number>)

	//success
	return 0;
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
