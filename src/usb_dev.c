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

#include <fcntl.h>
#include <mcu/usb.h>
#include <mcu/pio.h>
#include <cortexm/cortexm.h>
#include <usbd/types.h>
#include <mcu/core.h>
#include <mcu/debug.h>
#include <mcu/boot_debug.h>
#include <usbd/types.h>

#include "imxrt_local.h"

#if MCU_USB_PORTS > 0

static void usb_connect(u32 port, u32 con);
static void usb_configure(const devfs_handle_t * handle, u32 cfg);
static void usb_set_address(const devfs_handle_t * handle, u32 addr);
static void usb_reset_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_flush_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_enable_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_disable_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_stall_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_unstall_endpoint(const devfs_handle_t * handle, u32 endpoint_num);
static void usb_configure_endpoint(const devfs_handle_t * handle, u32 endpoint_num, u32 max_packet_size, u8 type);
static void usb_reset(const devfs_handle_t * handle);

typedef struct MCU_PACK {

} usb_local_t;

static usb_local_t usb_local[MCU_USB_PORTS] MCU_SYS_MEM;

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(usb, USB_VERSION, USB_IOC_IDENT_CHAR, I_MCU_TOTAL + I_USB_TOTAL, mcu_usb_isconnected)


int mcu_usb_open(const devfs_handle_t * handle){
	return 0;
}

int mcu_usb_close(const devfs_handle_t * handle){
	return 0;
}

int mcu_usb_getinfo(const devfs_handle_t * handle, void * ctl){
	return 0;

}

int mcu_usb_setattr(const devfs_handle_t * handle, void * ctl){
	return 0;
}

void usb_connect(u32 port, u32 con){

}

int mcu_usb_setaction(const devfs_handle_t * handle, void * ctl){
	return 0;
}

int mcu_usb_read(const devfs_handle_t * handle, devfs_async_t * rop){
	return 0;
}

int mcu_usb_write(const devfs_handle_t * handle, devfs_async_t * wop){
	return 0;
}


void usb_reset(const devfs_handle_t * handle){

}

void usb_wakeup(int port){

}

void usb_set_address(const devfs_handle_t * handle, u32 addr){

}

void usb_configure(const devfs_handle_t * handle, u32 cfg){

}

void usb_configure_endpoint(const devfs_handle_t * handle, u32 endpoint_num, u32 max_packet_size, u8 type){

}

void usb_enable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_disable_endpoint(const devfs_handle_t * handle, u32 endpoint_num){

}

void usb_reset_endpoint(const devfs_handle_t * handle, u32 endpoint_num){
}

void usb_flush_endpoint(const devfs_handle_t * handle, u32 endpoint_num){


}

void usb_stall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){

}

void usb_unstall_endpoint(const devfs_handle_t * handle, u32 endpoint_num){

}

int mcu_usb_isconnected(const devfs_handle_t * handle, void * ctl){
	return 0;
}

void usb_clr_ep_buf(const devfs_handle_t * handle, u32 endpoint_num){

}

int mcu_usb_root_read_endpoint(const devfs_handle_t * handle, u32 endpoint_num, void * dest){
	return 0;

}

int mcu_usb_root_write_endpoint(const devfs_handle_t * handle, u32 endpoint_num, const void * src, u32 size){
	return 0;

}


void mcu_core_otg_fs_isr(){

}


#endif








