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

#include <mcu/debug.h>
#include <stdbool.h>
#include "imxrt_local.h"
#include "cortexm/cortexm.h"
#include "mcu/tmr.h"
#include "mcu/core.h"


#define NUM_TMRS MCU_TMR_PORTS
#define NUM_OCS 3
#define NUM_ICS 2

#define DRIVER_ASSIGN_LOCAL(object, port_count) \
	const u32 port = handle->port; \
	if( port >= port_count ){ return SYSFS_SET_RETURN(EBUSY); } \
	object##_local_t * local = m_##object##_local + handle->port

enum {
	CHANNEL_TYPE_NONE,
	CHANNEL_TYPE_OUTPUT_COMPARE,
	CHANNEL_TYPE_INPUT_CAPTURE,
	CHANNEL_TYPE_PWM
};

#if MCU_TMR_PORTS > 0

static GPT_Type * const tmr_regs_table[NUM_TMRS] = MCU_TMR_REGS;
static u8 const tmr_irqs[MCU_TMR_PORTS] = MCU_TMR_IRQS;



typedef struct MCU_PACK {
	GPT_Type * instance;
	//TIM_HandleTypeDef hal_handle; //must be first
	mcu_event_handler_t handler[NUM_OCS+NUM_ICS];
	mcu_event_handler_t period_handler;
	u8 ref_count;
} tmr_local_t;

static tmr_local_t m_tmr_local[NUM_TMRS];

static void clear_actions(int port);

static int execute_handler(mcu_event_handler_t * handler, u32 o_events, u32 channel, u32 value);

void clear_actions(int port){
	memset(m_tmr_local[port].handler, 0, (NUM_OCS+NUM_ICS)*sizeof(mcu_event_handler_t));
}

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(tmr, TMR_VERSION, TMR_IOC_IDENT_CHAR, I_MCU_TOTAL + I_TMR_TOTAL, mcu_tmr_setchannel, mcu_tmr_getchannel, mcu_tmr_set, mcu_tmr_get, mcu_tmr_enable, mcu_tmr_disable)

int mcu_tmr_open(const devfs_handle_t * handle){
	DRIVER_ASSIGN_LOCAL(tmr, MCU_TMR_PORTS);
	if ( local->ref_count == 0 ){
		clear_actions(port);
		local->instance = tmr_regs_table[port];

	}
	if( tmr_irqs[port] != (u8)-1 ){
		cortexm_enable_irq(tmr_irqs[port]);
	}
	local->ref_count++;
	return 0;
}


int mcu_tmr_close(const devfs_handle_t * handle){
	DRIVER_ASSIGN_LOCAL(tmr, MCU_TMR_PORTS);
	if ( m_tmr_local[port].ref_count > 0 ){
		if ( m_tmr_local[port].ref_count == 1 ){
			clear_actions(port);
			local->instance = 0;
			if( tmr_irqs[port] != (u8)-1 ){
				cortexm_disable_irq(tmr_irqs[port]);
			}


			m_tmr_local[port].ref_count--;
		}
	}
	return 0;
}


int mcu_tmr_getinfo(const devfs_handle_t * handle, void * ctl){
	tmr_info_t * info = ctl;

	// set supported flags and events
	info->o_flags = TMR_FLAG_IS_AUTO_RELOAD |
			TMR_FLAG_SET_TIMER |
			TMR_FLAG_IS_SOURCE_COUNTDOWN |
			TMR_FLAG_SET_CHANNEL |
			TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_PWM_MODE;
	info->o_events = 0;


	return 0;
}

int mcu_tmr_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);

	const tmr_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return -1;
	}

	u32 o_flags = attr->o_flags;
	u32 freq = attr->freq;
	//regs = tmr_regs_table[port];

	if( o_flags & TMR_FLAG_SET_TIMER ){

		if( (o_flags & (TMR_FLAG_IS_SOURCE_CPU|TMR_FLAG_IS_SOURCE_IC0|TMR_FLAG_IS_SOURCE_IC1)) ){

			if( o_flags & TMR_FLAG_IS_SOURCE_CPU ){
				if( attr->freq == 0 ){
					freq = 1000000;
				}

			} else {
				if( o_flags & TMR_FLAG_IS_SOURCE_EDGEFALLING ){

				} else if( o_flags & TMR_FLAG_IS_SOURCE_EDGEBOTH ){

				}
			}


			if( o_flags & TMR_FLAG_IS_SOURCE_IC1 ){

			}

			//Set the prescalar so that the freq is correct
			//get the peripheral clock frequency




			if( o_flags & TMR_FLAG_IS_SOURCE_COUNTDOWN ){
				//m_tmr_local[port].hal_handle.Init.CounterMode = TIM_COUNTERMODE_DOWN;
			} else {
				//m_tmr_local[port].hal_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
			}

			if( o_flags & TMR_FLAG_IS_AUTO_RELOAD ){
				//m_tmr_local[port].hal_handle.Init.Period = attr->period;
			} else {
				//m_tmr_local[port].hal_handle.Init.Period = (u32)-1; //set to the max
			}
			//m_tmr_local[port].hal_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;


		}
	}


	if( o_flags & TMR_FLAG_SET_CHANNEL ){

		//configure the channel according to tmr_attr_t

		//this sets the value of the channel
		if( mcu_tmr_setchannel(handle, (void*)&attr->channel) < 0 ){
			return SYSFS_SET_RETURN(EIO);
		}
	}

	if( o_flags & (TMR_FLAG_SET_TIMER|TMR_FLAG_SET_CHANNEL) ){
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(tmr_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(tmr_pin_assignment_t),
				 CORE_PERIPH_TMR, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}
	}

	return 0;
}

int mcu_tmr_enable(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
	GPT_StartTimer(local->instance);

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_disable(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
	GPT_StopTimer(local->instance);
	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_setchannel(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
	mcu_channel_t * req = (mcu_channel_t*)ctl;
	u8 chan;

	if( req->loc & MCU_CHANNEL_FLAG_IS_INPUT ){
		chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
		if ( chan >= NUM_ICS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
		GPT_SetInputCaptureValue(local->instance, chan, req->value);
	} else {
		chan = req->loc;
		if ( chan >= NUM_OCS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
		GPT_SetOutputCompareValue(local->instance, chan, req->value);
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_getchannel(const devfs_handle_t * handle, void * ctl){
	tmr_local_t * local = m_tmr_local + handle->port;
	mcu_channel_t * req = (mcu_channel_t*)ctl;
	u8 chan;


	if( req->loc & MCU_CHANNEL_FLAG_IS_INPUT ){
		chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
		if ( chan >= NUM_ICS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
		req->value = GPT_GetInputCaptureValue(local->instance, chan);
	} else {
		chan = req->loc;
		if ( chan >= NUM_OCS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
		req->value = GPT_GetOutputCompareValue(local->instance, chan);
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_write(const devfs_handle_t * handle, devfs_async_t * wop){
	return SYSFS_SET_RETURN(ENOTSUP);
}


int mcu_tmr_setaction(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
	mcu_action_t * action = (mcu_action_t*)ctl;
	//regs = tmr_regs_table[port];
	u32 chan;
	u32 o_events;
	u32 tim_channel;

	o_events = action->o_events;
	chan = action->channel & ~MCU_CHANNEL_FLAG_IS_INPUT;

	if( chan >= NUM_OCS ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	//tim_channel = tmr_channels[chan];

	if ( o_events == MCU_EVENT_FLAG_NONE ){ //Check to see if all actions are disabled for the channel
		m_tmr_local[port].handler[chan].callback = 0;
		if( action->channel & MCU_CHANNEL_FLAG_IS_INPUT ){
			//HAL_TIM_IC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
		} else {
			//HAL_TIM_OC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
		}

		//execute a cancelled callback

	} else if( o_events & MCU_EVENT_FLAG_MATCH ){

		if( action->channel & MCU_CHANNEL_FLAG_IS_INPUT ){

			if( action->handler.callback != 0 ){
				//HAL_TIM_IC_Start_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}  else {
				//HAL_TIM_IC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}

		} else {

			if( action->handler.callback != 0 ){
				//HAL_TIM_OC_Start_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}  else {
				//HAL_TIM_OC_Stop_IT(&m_tmr_local[port].hal_handle, tim_channel);
			}

		}

		m_tmr_local[port].handler[chan] = action->handler;
	} else if( o_events & MCU_EVENT_FLAG_OVERFLOW ){

		if( action->handler.callback != 0 ){
			//HAL_TIM_Base_Start_IT(&m_tmr_local[port].hal_handle);
		}  else {
			//HAL_TIM_Base_Stop_IT(&m_tmr_local[port].hal_handle);
		}

		m_tmr_local[port].period_handler = action->handler;
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_read(const devfs_handle_t * handle, devfs_async_t * rop){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_set(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
	//local->instance->CNT = (u32)ctl;
	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_get(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
	u32 * value = ctl;
	if( value ){
		*value = GPT_GetCurrentTimerCount(local->instance);
		return SYSFS_RETURN_SUCCESS;
	}
	return SYSFS_SET_RETURN(EINVAL);
}

static void tmr_isr(int port); //This is speed optimized
//void tmr_isr(int port); //This is size optimized

int execute_handler(mcu_event_handler_t * handler, u32 o_events, u32 channel, u32 value){
	tmr_event_t event;
	event.channel.loc = channel;
	event.channel.value = value;
	return mcu_execute_event_handler(handler, o_events, &event);
}

//Four timers with 4 OC's and 2 IC's each
void tmr_isr(int port){

}

void mcu_core_tmr1_isr(){
	tmr_isr(0);
}

void mcu_core_tmr2_isr(){
	tmr_isr(1);
}

void mcu_core_tmr3_isr(){
	tmr_isr(2);
}

void mcu_core_tmr4_isr(){
	tmr_isr(3);
}
#endif

