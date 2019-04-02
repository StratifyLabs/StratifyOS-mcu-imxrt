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

#include "MIMXRT1052/usb_device_config.h"
#include "MIMXRT1052/usb.h"
#include "MIMXRT1052/usb_device.h"
#include "MIMXRT1052/usb_phy.h"


#if MCU_USB_PORTS > 0

typedef struct MCU_PACK {
	usb_device_handle hal_handle;
	u32 read_ready;
	mcu_event_handler_t control_handler;
	mcu_event_handler_t special_event_handler;
	devfs_transfer_handler_t endpoint_handlers[DEV_USB_LOGICAL_ENDPOINT_COUNT];
	u8 control_endpoint_buffer[USB_CONTROL_MAX_PACKET_SIZE];
	u16 control_endpoint_size;
	u16 is_connected;
	u8 ref_count;
} usb_local_t;

static usb_local_t m_usb_local[MCU_USB_PORTS] MCU_SYS_MEM;
static usb_status_t USB_DeviceControlPipeInit(usb_device_handle handle, void *param);

//driver callbacks
static usb_status_t mcu_usb_device_callback(usb_device_handle handle, uint32_t event, void *eventParam);

static usb_status_t mcu_usb_control_in_endpoint_callback(usb_device_handle handle,
																			usb_device_endpoint_callback_message_struct_t *message,
																			void *callbackParam);

static usb_status_t mcu_usb_control_out_endpoint_callback(usb_device_handle handle,
																			 usb_device_endpoint_callback_message_struct_t *message,
																			 void *callbackParam);

static usb_status_t mcu_usb_device_endpoint_in_callback(usb_device_handle handle,
																		  usb_device_endpoint_callback_message_struct_t *message,
																		  void *callbackParam);

static usb_status_t mcu_usb_device_endpoint_out_callback(usb_device_handle handle,
																			usb_device_endpoint_callback_message_struct_t *message,
																			void *callbackParam);

static void usb_connect(u32 port, u32 con);
static void usb_configure(usb_device_handle hal_handle, u32 cfg);
static void usb_set_address(usb_device_handle hal_handle, u32 addr);
static void usb_reset_endpoint(usb_device_handle hal_handle, u32 endpoint_num);
static void usb_flush_endpoint(usb_device_handle hal_handle, u32 endpoint_num);
static void usb_enable_endpoint(usb_device_handle hal_handle, u32 endpoint_num);
static void usb_disable_endpoint(usb_device_handle hal_handle, u32 endpoint_num);
static void usb_stall_endpoint(usb_device_handle hal_handle, u32 endpoint_num);
static void usb_unstall_endpoint(usb_device_handle hal_handle, u32 endpoint_num);
static void usb_configure_endpoint(const devfs_handle_t * handle, usb_device_handle hal_handle, u32 endpoint_num, u32 max_packet_size, u8 type);
static void usb_reset(usb_device_handle hal_handle);
static usb_local_t * local_from_handle(usb_device_handle hal_handle);

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(usb, USB_VERSION, USB_IOC_IDENT_CHAR, I_MCU_TOTAL + I_USB_TOTAL, mcu_usb_isconnected)

int mcu_usb_open(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	if( local->ref_count == 0 ){

	}
	local->ref_count++;

	return 0;
}

int mcu_usb_close(const devfs_handle_t * handle){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	if( local->ref_count ){
		local->ref_count--;
		if( !local->ref_count ){

			//shut down the interface if it is running


		}
	}
	return 0;
}

int mcu_usb_getinfo(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	usb_info_t * info = ctl;

	info->o_flags = USB_FLAG_ATTACH |
			USB_FLAG_SET_DEVICE |
			USB_FLAG_CONFIGURE |
			USB_FLAG_CONFIGURE_ENDPOINT |
			USB_FLAG_DETACH |
			USB_FLAG_DISABLE_ENDPOINT |
			USB_FLAG_ENABLE_ENDPOINT;

	info->o_events = MCU_EVENT_FLAG_SETUP |
			MCU_EVENT_FLAG_WRITE_COMPLETE |
			MCU_EVENT_FLAG_DATA_READY |
			MCU_EVENT_FLAG_CANCELED |
			MCU_EVENT_FLAG_ERROR;

	return 0;

}

#define BOARD_USB_PHY_D_CAL (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)

int mcu_usb_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	//USB_DeviceGetStatus
	//USB_DeviceSetStatus

	const usb_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){ return SYSFS_SET_RETURN(ENOSYS); }
	u32 o_flags = attr->o_flags;
	int result;

	if( o_flags & USB_FLAG_SET_DEVICE ){

		//Start the USB clock
		CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
		CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);

		usb_phy_config_struct_t phyConfig = {
			BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
		};

		USB_EhciPhyInit(kUSB_ControllerEhci0, attr->freq, &phyConfig);


		local->read_ready = 0;

		result = mcu_set_pin_assignment(
					&(attr->pin_assignment),
					MCU_CONFIG_PIN_ASSIGNMENT(usb_config_t, handle),
					MCU_PIN_ASSIGNMENT_COUNT(usb_pin_assignment_t),
					CORE_PERIPH_USB, port, 0, 0, 0);

		if( result < 0 ){ return result; }


		if( o_flags & USB_FLAG_IS_HIGH_SPEED ){

		} else {
			if ( attr->max_packet_size <= 8 ){
			} else if ( attr->max_packet_size <= 16 ){
			} else if ( attr->max_packet_size <= 32 ){
			}
		}

		if( o_flags & USB_FLAG_IS_VBUS_SENSING_ENABLED ){

		}

		if( o_flags & USB_FLAG_IS_SOF_ENABLED ){

		}

		if( o_flags & USB_FLAG_IS_LOW_POWER_MODE_ENABLED ){

		}

		if( o_flags & USB_FLAG_IS_BATTERY_CHARGING_ENABLED ){

		}

		result = USB_DeviceInit(kUSB_ControllerEhci0, mcu_usb_device_callback, &local->hal_handle);
		if( result != kStatus_USB_Success ){
			return SYSFS_SET_RETURN(EIO);
		}

		//enable the interrrupts
		//cortexm_enable_irq(USB_PHY1_IRQn);
		cortexm_enable_irq(USB_OTG1_IRQn);
		USB_DeviceRun(local->hal_handle);
	}


	if( o_flags & USB_FLAG_RESET ){ usb_reset(local->hal_handle); }
	if( o_flags & USB_FLAG_ATTACH ){ usb_connect(port, 1); }
	if( o_flags & USB_FLAG_DETACH ){ usb_connect(port, 0); }
	if( o_flags & USB_FLAG_CONFIGURE ){ usb_configure(local->hal_handle, 1); }
	if( o_flags & USB_FLAG_UNCONFIGURE ){ usb_configure(local->hal_handle, 0); }
	if( o_flags & USB_FLAG_SET_ADDRESS ){ usb_set_address(local->hal_handle, attr->address); }
	if( o_flags & USB_FLAG_RESET_ENDPOINT ){ usb_reset_endpoint(local->hal_handle, attr->address); }
	if( o_flags & USB_FLAG_ENABLE_ENDPOINT ){ usb_enable_endpoint(local->hal_handle, attr->address); }
	if( o_flags & USB_FLAG_FLUSH_ENDPOINT ){ usb_flush_endpoint(local->hal_handle, attr->address); }
	if( o_flags & USB_FLAG_DISABLE_ENDPOINT ){ usb_disable_endpoint(local->hal_handle, attr->address); }
	if( o_flags & USB_FLAG_STALL_ENDPOINT ){ usb_stall_endpoint(local->hal_handle, attr->address); }
	if( o_flags & USB_FLAG_UNSTALL_ENDPOINT ){ usb_unstall_endpoint(local->hal_handle, attr->address); }
	if( o_flags & USB_FLAG_CONFIGURE_ENDPOINT ){
		usb_configure_endpoint(handle, local->hal_handle, attr->address, attr->max_packet_size, attr->type);
	}

	return SYSFS_RETURN_SUCCESS;
}

void usb_connect(u32 port, u32 con){

	if( con ){
		//connect
	} else {
		//disconnect
	}


}

int mcu_usb_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
	mcu_action_t * action = (mcu_action_t*)ctl;
	int log_ep;
	int result = -1;

	cortexm_set_irq_priority(USB_OTG1_IRQn, action->prio, action->o_events);
	log_ep = action->channel & ~0x80;

	if( action->o_events &
		 (MCU_EVENT_FLAG_POWER|MCU_EVENT_FLAG_SUSPEND|MCU_EVENT_FLAG_STALL|MCU_EVENT_FLAG_SOF|MCU_EVENT_FLAG_WAKEUP)
		 ){
		local->special_event_handler = action->handler;
		return 0;
	}

	if( action->handler.callback == 0 ){

		//cancel any pending actions and execute the callback
		if( action->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE){
			devfs_execute_write_handler(&local->endpoint_handlers[log_ep], 0, 0, MCU_EVENT_FLAG_CANCELED);
			if( log_ep == 0 ){
				mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_WRITE_COMPLETE, 0);
			} else {
				devfs_execute_write_handler(&local->endpoint_handlers[log_ep], 0, 0, MCU_EVENT_FLAG_CANCELED);
			}
		}

		if( action->o_events & MCU_EVENT_FLAG_DATA_READY){
			local->read_ready &= ~(1<<log_ep);
			if( log_ep == 0 ){
				mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_DATA_READY, 0);
			} else {
				devfs_execute_read_handler(&local->endpoint_handlers[log_ep], 0, 0, MCU_EVENT_FLAG_CANCELED);
			}
		}

	} else if( action->channel == 0 ){

		if( cortexm_validate_callback(action->handler.callback) < 0 ){
			return SYSFS_SET_RETURN(EPERM);
		}

		//setup the control callback for channel zero
		if( action->o_events &
			 (MCU_EVENT_FLAG_DATA_READY|MCU_EVENT_FLAG_WRITE_COMPLETE|MCU_EVENT_FLAG_SETUP) ){
			local->control_handler = action->handler;
			result = 0;
		}

	} else {
		return SYSFS_SET_RETURN(EINVAL);
	}

	if( result < 0 ){
		result = SYSFS_SET_RETURN(EINVAL);
	}
	return result;

	return 0;
}

int mcu_usb_read(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	int result;
	int loc = async->loc;

	if ( loc > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	DEVFS_DRIVER_IS_BUSY(local->endpoint_handlers[loc].read, async);

	//mcu_debug_printf("r:%d\n", async->nbyte);

	//this version must always be allowed to block
	if ( !(async->flags & O_NONBLOCK) ){
		//check to see if endpoint is configured
		usb_device_endpoint_status_struct_t endpoint_status;

		endpoint_status.endpointAddress = loc;
		if( USB_DeviceGetStatus(local->hal_handle, kUSB_DeviceStatusEndpoint, &endpoint_status) != kStatus_USB_Success ){
			result = SYSFS_SET_RETURN(EIO);
		} else {
			if( endpoint_status.endpointStatus == kUSB_DeviceEndpointStateIdle && (local->read_ready & (1<<loc)) ){
				//return success to indicate this is a blocking call
				result = USB_DeviceRecvRequest(local->hal_handle, loc, async->buf, async->nbyte); //next packet will be a setup packet
				if( result != kStatus_USB_Success ){
					result = SYSFS_SET_RETURN(EIO);
				} else {
					result = SYSFS_RETURN_SUCCESS;
				}
			} else {
				result = SYSFS_RETURN_SUCCESS;
			}
		}
	} else {
		result = SYSFS_SET_RETURN(EAGAIN);
	}

	if( result != SYSFS_RETURN_SUCCESS ){
		local->endpoint_handlers[loc].read = 0;
	}

	return result;
}

int mcu_usb_write(const devfs_handle_t * handle, devfs_async_t * async){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);

	//Asynchronous write
	int ep;
	int loc = async->loc;
	int bytes_written;

	ep = (loc & 0x7F);

	if ( ep > (DEV_USB_LOGICAL_ENDPOINT_COUNT-1) ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	DEVFS_DRIVER_IS_BUSY(local->endpoint_handlers[ep].write, async);

	//mcu_debug_printf("**************** w:%d *********************\n", async->nbyte);

	bytes_written = mcu_usb_root_write_endpoint(handle, loc, async->buf, async->nbyte);



	if ( bytes_written < 0 ){
		usb_disable_endpoint(local->hal_handle, loc );
		usb_reset_endpoint(local->hal_handle, loc );
		usb_enable_endpoint(local->hal_handle, loc );
		local->endpoint_handlers[loc].write = 0;
		return SYSFS_SET_RETURN(EIO);
	}

	return SYSFS_RETURN_SUCCESS;
}


void usb_reset(usb_device_handle hal_handle){

}

void usb_wakeup(int port){

}

void usb_set_address(usb_device_handle hal_handle, u32 addr){
	//this will just set the address in internal RAM, needs to be made effective after control in completes
	USB_DeviceSetStatus(hal_handle, kUSB_DeviceStatusAddress, &addr);
}

void usb_configure(usb_device_handle hal_handle, u32 cfg){

}

void usb_configure_endpoint(const devfs_handle_t * handle, usb_device_handle hal_handle, u32 endpoint_num, u32 max_packet_size, u8 type){
	usb_device_endpoint_init_struct_t epInitStruct;
	usb_device_endpoint_callback_struct_t epCallback;
	u8 endpoint_offset = endpoint_num & 0x7f;

	epInitStruct.endpointAddress = endpoint_num;
	epInitStruct.maxPacketSize = max_packet_size;
	epInitStruct.transferType = type;

	//USB_DeviceInitEndpoint
	epInitStruct.zlt = 0;
	if( endpoint_num & 0x80 ){
		epCallback.callbackFn = mcu_usb_device_endpoint_in_callback;
	} else {
		epCallback.callbackFn = mcu_usb_device_endpoint_out_callback;
	}
	usb_local_t * local = m_usb_local + handle->port;
	epCallback.callbackParam = local->endpoint_handlers + endpoint_offset;


	USB_DeviceInitEndpoint(hal_handle, &epInitStruct, &epCallback);

	if(endpoint_offset == endpoint_num){
		local->read_ready |= (1<<endpoint_offset);
		if( local->endpoint_handlers[endpoint_offset].read ){
			devfs_async_t * async = local->endpoint_handlers[endpoint_offset].read;
			if( USB_DeviceRecvRequest(local->hal_handle, async->loc, async->buf, async->nbyte) != kStatus_USB_Success ){
				devfs_execute_read_handler(local->endpoint_handlers + endpoint_offset, 0, SYSFS_SET_RETURN(EIO), MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR);
			}
		}
	}


}

void usb_enable_endpoint(usb_device_handle hal_handle, u32 endpoint_num){

}

void usb_disable_endpoint(usb_device_handle hal_handle, u32 endpoint_num){
	//USB_DeviceDeinitEndpoint
	USB_DeviceDeinitEndpoint(hal_handle, endpoint_num);

}

void usb_reset_endpoint(usb_device_handle hal_handle, u32 endpoint_num){

}

void usb_flush_endpoint(usb_device_handle hal_handle, u32 endpoint_num){

}

void usb_stall_endpoint(usb_device_handle hal_handle, u32 endpoint_num){
	//USB_DeviceStallEndpoint
	USB_DeviceStallEndpoint(hal_handle, endpoint_num);

}

void usb_unstall_endpoint(usb_device_handle hal_handle, u32 endpoint_num){
	//USB_DeviceUnstallEndpoint
	USB_DeviceUnstallEndpoint(hal_handle, endpoint_num);

}

int mcu_usb_isconnected(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
	return 0;
}

void usb_clr_ep_buf(const devfs_handle_t * handle, u32 endpoint_num){

}

int mcu_usb_root_read_endpoint(const devfs_handle_t * handle, u32 endpoint_num, void * dest){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
	int result;
	//at this point the data must be ready to copy
	if( endpoint_num == 0 ){
		memcpy(dest, local->control_endpoint_buffer, local->control_endpoint_size);
		result = local->control_endpoint_size;
		local->control_endpoint_size = 0;
		return result;
	}

	//USB_DeviceRecvRequest(local->hal_handle, endpoint_num, dest, 64);

	/*
	 * Other endpoints call	USB_DeviceRecvRequest() which drops the data
	 * directly in the async->buf with no need to read from the low level buffer
	 *
	 */

	return SYSFS_SET_RETURN(ENOTSUP);

}

int mcu_usb_root_write_endpoint(const devfs_handle_t * handle, u32 endpoint_num, const void * src, u32 size){
	DEVFS_DRIVER_DECLARE_LOCAL(usb, MCU_USB_PORTS);
	int result;

	result = USB_DeviceSendRequest(local->hal_handle, endpoint_num, (void*)src, size);
	if( result == kStatus_USB_Success ){
		if( ((endpoint_num & 0x7f) == 0) && (size != USB_CONTROL_MAX_PACKET_SIZE) && (size != 0)){
			USB_DeviceRecvRequest(local->hal_handle, 0, 0, 0); //next packet will be a setup packet
		}
		return size;
	}

	mcu_debug_printf("failed to write %d to 0x%X - %d\n", size, endpoint_num, result);
	return SYSFS_SET_RETURN(EIO);
}

usb_local_t * local_from_handle(usb_device_handle hal_handle){
	for(u32 i = 0; i < MCU_USB_PORTS; i++){
		if( hal_handle == m_usb_local[i].hal_handle ){
			return m_usb_local + i;
		}
	}
	return m_usb_local;
}


usb_status_t mcu_usb_device_callback(usb_device_handle handle, uint32_t event, void *eventParam){
	u32 o_flags = 0;
	usb_local_t * local = local_from_handle(handle);

	switch(event){
		case kUSB_DeviceEventBusReset:
			/* Initialize the control pipes */
			USB_DeviceControlPipeInit(handle, local);

			//execute special event handler if it is enabled
			o_flags = MCU_EVENT_FLAG_RESET;
			break;
		case kUSB_DeviceEventSuspend:
			o_flags = MCU_EVENT_FLAG_SUSPEND;
			break;
		case kUSB_DeviceEventResume:
			o_flags = MCU_EVENT_FLAG_RESUME;
			break;
		case kUSB_DeviceEventDetach:
			//not connected
			break;
		case kUSB_DeviceEventAttach:
			//connected
			break;
	}

	if( o_flags ){
		mcu_execute_event_handler(&(m_usb_local[0].special_event_handler), o_flags, 0);
	}

	return kStatus_USB_Success;
}

//OUT or setup (data has arrived)
usb_status_t mcu_usb_control_out_endpoint_callback(usb_device_handle handle,
																	usb_device_endpoint_callback_message_struct_t *message,
																	void *callbackParam){
	usb_local_t * local = callbackParam;
	usb_event_t usb_event;
	usb_event.epnum = 0;

	//execute the control handler
	if( message->isSetup ){
		//mcu_debug_printf("setup\n");
		if( local->control_endpoint_buffer != message->buffer ){
			local->control_endpoint_size = message->length > USB_CONTROL_MAX_PACKET_SIZE ? USB_CONTROL_MAX_PACKET_SIZE : message->length;
			memcpy(local->control_endpoint_buffer, message->buffer, local->control_endpoint_size);
		}

		usbd_setup_packet_t * setup = (usbd_setup_packet_t *)local->control_endpoint_buffer;
		local->control_endpoint_size = message->length;

		mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_SETUP, &usb_event);

		if( (setup->bmRequestType.bitmap_t.dir == USBD_REQUEST_TYPE_DIRECTION_HOST_TO_DEVICE) && (setup->wLength > 0) ){
			//get ready to receive
			//mcu_debug_printf("need to rx data\n");
			USB_DeviceRecvRequest(local->hal_handle, 0, local->control_endpoint_buffer, setup->wLength);
		}
	} else {
		//mcu_debug_printf("control out event %d\n", message->length);
		if( message->length > 0 ){
			local->control_endpoint_size = message->length;
			if( local->control_endpoint_buffer != message->buffer ){
				mcu_debug_printf("buffer mismatch\n");
			}
			mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_DATA_READY, &usb_event);
		}
	}


	return kStatus_USB_Success;
}

//in to HOST from device
usb_status_t mcu_usb_control_in_endpoint_callback(usb_device_handle handle,
																  usb_device_endpoint_callback_message_struct_t *message,
																  void *callbackParam){
	usb_local_t * local = callbackParam;
	usb_event_t usb_event;
	usb_event.epnum = 0;

	//mcu_debug_printf("control in %d\n", message->length);

	mcu_execute_event_handler(&local->control_handler, MCU_EVENT_FLAG_WRITE_COMPLETE, &usb_event);

	u8 state;
	USB_DeviceGetStatus(handle, kUSB_DeviceStatusDeviceState, &state);
	if( state == kUSB_DeviceStateAddressing ){
		//this makes the address effective at the phy layer
		USB_DeviceSetStatus(handle, kUSB_DeviceStatusAddress, 0);

		//now set state to address (address accepted)
		state = kUSB_DeviceStateAddress;
		USB_DeviceSetStatus(handle, kUSB_DeviceStatusDeviceState, &state);

		u8 speed;
		USB_DeviceGetStatus(handle, kUSB_DeviceStatusSpeed, &speed);

	}
	//mcu_debug_printf("-----CTRL--------------------\n");

	return kStatus_USB_Success;
}

usb_status_t mcu_usb_device_endpoint_in_callback(usb_device_handle handle,
																 usb_device_endpoint_callback_message_struct_t *message,
																 void *callbackParam){
	devfs_transfer_handler_t * transfer = callbackParam;
	//endpoint number?


	if( transfer->write ){
		u32 o_flags;
		mcu_debug_printf("-----USB Write Delay---------------\n");

		usb_event_t usb_event;
		usb_event.epnum = transfer->write->loc | 0x80;

		if( message->length > 0 ){
			transfer->write->nbyte = message->length;
			o_flags = MCU_EVENT_FLAG_WRITE_COMPLETE;
		} else {
			transfer->write->nbyte = SYSFS_SET_RETURN(EIO);
			o_flags = MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR;
		}
		//mcu_debug_printf("wd0x%X:% d\n", usb_event.epnum, message->length);
		if( message->buffer != transfer->write->buf ){
			mcu_debug_printf("buffer mismatch\n");
		}

		devfs_execute_write_handler(transfer, &usb_event, 0, o_flags);

	} else {
		mcu_debug_printf("nothing to transfer\n");
	}

	return kStatus_USB_Success;
}

usb_status_t mcu_usb_device_endpoint_out_callback(usb_device_handle handle,
																  usb_device_endpoint_callback_message_struct_t *message,
																  void *callbackParam){
	devfs_transfer_handler_t * transfer = callbackParam;

	//endpoint number?
	if( transfer->read ){
		u32 o_flags;

		usb_event_t usb_event;
		usb_event.epnum = transfer->read->loc;

		if( message->length > 0 ){
			transfer->read->nbyte = message->length;
			o_flags = MCU_EVENT_FLAG_DATA_READY;
		} else {
			transfer->read->nbyte = SYSFS_SET_RETURN(EIO);
			o_flags = MCU_EVENT_FLAG_CANCELED | MCU_EVENT_FLAG_ERROR;
		}

		//mcu_debug_printf("rd:%d\n", message->length);
		devfs_execute_read_handler(transfer, &usb_event, 0, o_flags);
	}
	return kStatus_USB_Success;
}

usb_status_t USB_DeviceControlPipeInit(usb_device_handle handle, void *param)
{
	usb_device_endpoint_init_struct_t epInitStruct;
	usb_device_endpoint_callback_struct_t epCallback;
	usb_status_t error;

	epCallback.callbackFn = mcu_usb_control_in_endpoint_callback;
	epCallback.callbackParam = param;

	epInitStruct.zlt = 0U;
	epInitStruct.transferType = USB_ENDPOINT_CONTROL;
	epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
	epInitStruct.maxPacketSize = USB_CONTROL_MAX_PACKET_SIZE;
	/* Initialize the control IN pipe */
	error = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

	if (kStatus_USB_Success != error){
		mcu_debug_printf("control init error IN %d\n", error);
		return error;
	}

	epCallback.callbackFn = mcu_usb_control_out_endpoint_callback;
	epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
	/* Initialize the control OUT pipe */
	error = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

	if (kStatus_USB_Success != error){
		USB_DeviceDeinitEndpoint(handle,
										 USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
		mcu_debug_printf("control init error OUT %d\n", error);
		return error;
	}

	return kStatus_USB_Success;
}

void mcu_core_usb_phy1_isr(){
	//mcu_debug_printf("phy1\n");
}

void mcu_core_usb_phy2_isr(){}


void mcu_core_usb_otg1_isr(){
	USB_DeviceEhciIsrFunction(m_usb_local[0].hal_handle);
	//cortexm_delay_us(5000);
}

void mcu_core_usb_otg2_isr(){}


#endif



