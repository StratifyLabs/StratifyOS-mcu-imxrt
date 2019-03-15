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

enum {
	CHANNEL_TYPE_NONE,
	CHANNEL_TYPE_OUTPUT_COMPARE,
	CHANNEL_TYPE_INPUT_CAPTURE,
	CHANNEL_TYPE_PWM
};

#if MCU_TMR_PORTS > 0

static GPT_Type * const tmr_regs_table[NUM_TMRS] = MCU_TMR_REGS;
static u8 const tmr_irqs[MCU_TMR_PORTS] = MCU_TMR_IRQS;

typedef enum {
	TIMER_FLAG_NONE = 0,
	TIMER_FLAG_MAP_ROLLOVER_TO_MATCH, // only valid on first channel
} timer_flag_e;

typedef struct MCU_PACK {
	GPT_Type * instance;
	mcu_event_handler_t handler[NUM_OCS+NUM_ICS]; // Out1, Out2, Out3, In1, In2
	mcu_event_handler_t period_handler;
	timer_flag_e flags;
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
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
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
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
	if ( local->ref_count > 0 ){
		if ( local->ref_count == 1 ){
			clear_actions(port);

			GPT_Deinit(local->instance);

			local->instance = 0;
			if( tmr_irqs[port] != (u8)-1 ){
				cortexm_disable_irq(tmr_irqs[port]);
			}
			local->ref_count--;
		}
	}
	return 0;
}


int mcu_tmr_getinfo(const devfs_handle_t * handle, void * ctl){
	tmr_info_t * info = ctl;

	// set supported flags and events
	info->o_flags = TMR_FLAG_SET_TIMER |
			TMR_FLAG_IS_AUTO_RELOAD |
			TMR_FLAG_SET_CHANNEL |
			TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH |
			TMR_FLAG_IS_CHANNEL_PWM_MODE;
	info->o_events = 0;


	return 0;
}

static gpt_input_capture_channel_t map_input_channel(u8 chan) {
	gpt_input_capture_channel_t input = kGPT_InputCapture_Channel1;
	if( chan == 1 ){
		input = kGPT_InputCapture_Channel2;
	}

	return input;
}

static gpt_output_compare_channel_t map_output_channel(u8 chan) {
	gpt_output_compare_channel_t output = kGPT_OutputCompare_Channel1;
	if (chan == 1) {
		output = kGPT_OutputCompare_Channel2;
	} else if (chan == 2) {
		output = kGPT_OutputCompare_Channel3;
	}

	return output;
}

int mcu_tmr_setattr(const devfs_handle_t * handle, void * ctl){
	DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);

	const tmr_attr_t * attr = mcu_select_attr(handle, ctl);
	if( attr == 0 ){
		return -1;
	}

	if (attr->channel.loc >= NUM_OCS) {
		return -1;
	}

	u32 o_flags = attr->o_flags;
	u32 freq = attr->freq;

	if( o_flags & TMR_FLAG_SET_TIMER ){

		/*
		 *
		 * When this flag is set, we only configure the actual timer. We
		 * don't configure any channels. The exception is if TMR_FLAG_IS_AUTO_RELOAD
		 * is set, then we set the compare output to attr->period.
		 *
		 *
		 */

		// Choose clock source
		//NOTE: logic could be added to optimize power consumption, use peripheral clock for now
		if( o_flags & (TMR_FLAG_IS_SOURCE_CPU)){

			//target 1MHz as the default if 0 is provided
			if( attr->freq == 0 ){ freq = 1000000; }


			gpt_config_t config;
			GPT_GetDefaultConfig(&config);
			config.enableMode = 0;

			/*
			 * with other flags similar to TMR_FLAG_IS_SOURCE_CPU the clock source
			 * could be set to something other than the peripheral clock
			 * config.clockSource = kGPT_ClockSource_HighFreq for flag TMR_FLAG_IS_SOURCE_HIGH_FREQUENCY (but that needs to be added to the driver interface)
			 *
			 */

			if( o_flags & TMR_FLAG_IS_AUTO_RELOAD ){
				//set compare channel 1 (0) to the period value
				config.enableFreeRun = 0; //uses compare channel '1' to set the period
			}
			GPT_Init(local->instance, &config);

			if( o_flags & TMR_FLAG_IS_AUTO_RELOAD ){
				GPT_SetOutputCompareValue(local->instance, kGPT_OutputCompare_Channel1, attr->period);
			}

			//Set the prescalar so that the freq is as accurate as possible
			u32 sourceclk_freq = CLOCK_GetFreq(kCLOCK_IpgClk)/2;
			u32 div = sourceclk_freq / freq;
			div = MIN(div, 4096);
			div = MAX(div, 1);
			GPT_SetClockDivider(local->instance, div);


		} else {
			//timer needs to use CPU (peripheral) clock until other flags are added to support HighFreq Ext LowFreq and Osc
			return SYSFS_SET_RETURN(EINVAL);
		}
	}

	if( o_flags & TMR_FLAG_SET_CHANNEL ){

		/*
		 * This will configure the channel. The timer can be configured
		 * using another call to ioctl with TMR_FLAG_SET_TIMER set.
		 *
		 *
		 */

		//this will configure the channel then set the channel value
		if( attr->channel.loc & MCU_CHANNEL_FLAG_IS_INPUT ){

			gpt_input_operation_mode_t mode = kGPT_InputOperation_RiseEdge;

			if( o_flags & TMR_FLAG_IS_CHANNEL_EDGEFALLING ){
				mode = kGPT_InputOperation_FallEdge;
			} else if( o_flags & TMR_FLAG_IS_CHANNEL_EDGEBOTH ){
				mode = kGPT_InputOperation_BothEdge;
			}

			//select between the two channels
			gpt_input_capture_channel_t channel = map_input_channel(attr->channel.loc & ~MCU_CHANNEL_FLAG_IS_INPUT);

			//set the mode
			GPT_SetInputOperationMode(local->instance, channel, mode);

		} else {
			//configure the output compare settings
			gpt_output_operation_mode_t mode = kGPT_OutputOperation_Disconnected;

			if( o_flags & TMR_FLAG_IS_CHANNEL_SET_OUTPUT_ON_MATCH ){
				mode = kGPT_OutputOperation_Set;
			} else if ( o_flags & TMR_FLAG_IS_CHANNEL_TOGGLE_OUTPUT_ON_MATCH ){
				mode = kGPT_OutputOperation_Toggle;
			} else if( o_flags & TMR_FLAG_IS_CHANNEL_CLEAR_OUTPUT_ON_MATCH ){
				mode = kGPT_OutputOperation_Clear;
			}
			//need another flag to support the active low like TMR_FLAG_IS_CHANNEL_ACTIVE_LOW

			gpt_output_compare_channel_t channel = map_output_channel(attr->channel.loc);
			GPT_SetOutputOperationMode(local->instance, channel, mode);
		}

		mcu_tmr_setchannel(handle, (void*)&attr->channel);

	}


#if 0 // SDK pin_mux.c for now
	if( o_flags & (TMR_FLAG_SET_TIMER|TMR_FLAG_SET_CHANNEL) ){
		if( mcu_set_pin_assignment(
				 &(attr->pin_assignment),
				 MCU_CONFIG_PIN_ASSIGNMENT(tmr_config_t, handle),
				 MCU_PIN_ASSIGNMENT_COUNT(tmr_pin_assignment_t),
				 CORE_PERIPH_TMR, port, 0, 0, 0) < 0 ){
			return SYSFS_SET_RETURN(EINVAL);
		}
	}
#endif

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
		return SYSFS_SET_RETURN(ENOTSUP); //channel write to the input value
	} else {
		chan = req->loc;
		if ( chan >= NUM_OCS ){
			return SYSFS_SET_RETURN(EINVAL);
		}
		gpt_output_compare_channel_t output = map_output_channel(chan);
		GPT_SetOutputCompareValue(local->instance, output, req->value);
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_getchannel(const devfs_handle_t * handle, void * ctl){
	tmr_local_t * local = m_tmr_local + handle->port;
	mcu_channel_t * req = (mcu_channel_t*)ctl;
	u8 chan;

	if( req->loc & MCU_CHANNEL_FLAG_IS_INPUT ){
		chan = req->loc & ~MCU_CHANNEL_FLAG_IS_INPUT;
		if ( chan >= NUM_ICS ){ return SYSFS_SET_RETURN(EINVAL); }
		gpt_input_capture_channel_t input = map_input_channel(req->loc);
		req->value = GPT_GetInputCaptureValue(local->instance, input);
	} else {
		chan = req->loc;
		if ( chan >= NUM_OCS ){ return SYSFS_SET_RETURN(EINVAL); }
		gpt_output_compare_channel_t output = map_output_channel(chan);
		req->value = GPT_GetOutputCompareValue(local->instance, output);
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
	uint32_t handler_idx;
	bool op_isenable;
	gpt_interrupt_enable_t inten = kGPT_OutputCompare1InterruptEnable;

	o_events = action->o_events;
	chan = action->channel & ~MCU_CHANNEL_FLAG_IS_INPUT;

	handler_idx = chan;
	if( action->channel & MCU_CHANNEL_FLAG_IS_INPUT ){
		handler_idx += NUM_OCS;
	}

	if( handler_idx >= (NUM_OCS + NUM_ICS) ){
		return SYSFS_SET_RETURN(EINVAL);
	}

	// default to controlling compare/capture interrupt source
	if (handler_idx == 1) {
		inten = kGPT_OutputCompare2InterruptEnable;
	} else if (handler_idx == 2) {
		inten = kGPT_OutputCompare3InterruptEnable;
	} else if (handler_idx == 3) {
		inten = kGPT_InputCapture1InterruptEnable;
	} else if (handler_idx == 4) {
		inten = kGPT_InputCapture2InterruptEnable;
	}

	// determine whether we are enabling or disabling
	op_isenable = 0;
	if( action->handler.callback != 0 ){
		op_isenable = 1;
	}

	// update callback
	if ( o_events == MCU_EVENT_FLAG_NONE ){ //Check to see if all actions are disabled for the channel
		local->handler[handler_idx].callback = 0;

		op_isenable = 0;

		//execute a cancelled callback

	} else if( o_events & MCU_EVENT_FLAG_MATCH ){
		local->handler[handler_idx] = action->handler;
	} else if( o_events & MCU_EVENT_FLAG_OVERFLOW ){

		local->period_handler = action->handler;
		inten = kGPT_RollOverFlagInterruptEnable;
	}

	// un/mask the interrupt for this source
	if (op_isenable) {
		GPT_EnableInterrupts(local->instance, inten);
	} else {
		GPT_DisableInterrupts(local->instance, inten);
	}

	return SYSFS_RETURN_SUCCESS;
}

int mcu_tmr_read(const devfs_handle_t * handle, devfs_async_t * rop){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int mcu_tmr_set(const devfs_handle_t * handle, void * ctl){
	//DEVFS_DRIVER_DECLARE_LOCAL(tmr, MCU_TMR_PORTS);
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

void tmr_isr(int port){
	if (port >= MCU_TMR_PORTS) {
		return;
	}

	tmr_local_t *local = &m_tmr_local[port];

	gpt_status_flag_t flags =	GPT_GetStatusFlags(local->instance,
																 (kGPT_OutputCompare1Flag | kGPT_OutputCompare2Flag | kGPT_OutputCompare3Flag |
																  kGPT_InputCapture1Flag | kGPT_InputCapture2Flag | kGPT_RollOverFlag) );

	GPT_ClearStatusFlags(local->instance, flags);

	if (flags & kGPT_RollOverFlag) {
		execute_handler(&local->period_handler, MCU_EVENT_FLAG_OVERFLOW, (u32)-1, 0);
	}

	if (flags & kGPT_OutputCompare1Flag) {
		execute_handler(&local->handler[0], MCU_EVENT_FLAG_MATCH, 0, GPT_GetCurrentTimerCount(local->instance));
	}

	if (flags & kGPT_OutputCompare2Flag) {
		execute_handler(&local->handler[1], MCU_EVENT_FLAG_MATCH, 1, GPT_GetCurrentTimerCount(local->instance));
	}

	if (flags & kGPT_OutputCompare3Flag) {
		execute_handler(&local->handler[2], MCU_EVENT_FLAG_MATCH, 2, GPT_GetCurrentTimerCount(local->instance));
	}

	if (flags & kGPT_InputCapture1Flag) {
		execute_handler(&local->handler[3],
				MCU_EVENT_FLAG_MATCH,
				0 | MCU_CHANNEL_FLAG_IS_INPUT,
				GPT_GetCurrentTimerCount(local->instance));
	}

	if (flags & kGPT_InputCapture2Flag) {
		execute_handler(&local->handler[4],
				MCU_EVENT_FLAG_MATCH,
				1 | MCU_CHANNEL_FLAG_IS_INPUT,
				GPT_GetCurrentTimerCount(local->instance));
	}
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

