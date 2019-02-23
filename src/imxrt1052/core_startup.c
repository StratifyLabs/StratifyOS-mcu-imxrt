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

#include <core_startup.h>
#include <mcu/mcu.h>
#include <mcu/bootloader.h>
#include "../core_startup.h"
#include "imxrt_local.h"


const bootloader_api_t mcu_core_bootloader_api MCU_WEAK;
const bootloader_api_t mcu_core_bootloader_api = {
	.code_size = 0,
};

void mcu_core_default_isr();
void mcu_core_hardware_id() MCU_WEAK;

void mcu_core_reset_handler() __attribute__ ((section(".reset_vector")));
void mcu_core_nmi_isr() MCU_WEAK;

void mcu_core_hardfault_handler();
void mcu_core_memfault_handler();
void mcu_core_busfault_handler();
void mcu_core_usagefault_handler();

void mcu_core_svcall_handler();
void mcu_core_debugmon_handler() MCU_WEAK;
void mcu_core_pendsv_handler();
void mcu_core_systick_handler();


//ISR's -- weakly bound to default handler
_DECLARE_ISR(dma0_dma16); //0
_DECLARE_ISR(dma1_dma17); //1
_DECLARE_ISR(dma2_dma18); //2
_DECLARE_ISR(dma3_dma19); //3
_DECLARE_ISR(dma4_dma20); //4
_DECLARE_ISR(dma5_dma21); //5
_DECLARE_ISR(dma6_dma22); //6
_DECLARE_ISR(dma7_dma23); //7
_DECLARE_ISR(dma8_dma24); //8
_DECLARE_ISR(dma9_dma25); //9
_DECLARE_ISR(dma10_dma26); //10
_DECLARE_ISR(dma11_dma27); //11
_DECLARE_ISR(dma12_dma28); //12
_DECLARE_ISR(dma13_dma29); //13
_DECLARE_ISR(dma14_dma30); //14
_DECLARE_ISR(dma15_dma31); //15
_DECLARE_ISR(dma_error); //16
_DECLARE_ISR(cti0_error); //17
_DECLARE_ISR(cti1_error); //18
_DECLARE_ISR(core); //19
_DECLARE_ISR(lpuart1); //20
_DECLARE_ISR(lpuart2); //21
_DECLARE_ISR(lpuart3); //22
_DECLARE_ISR(lpuart4); //23
_DECLARE_ISR(lpuart5); //24
_DECLARE_ISR(lpuart6); //25
_DECLARE_ISR(lpuart7); //26
_DECLARE_ISR(lpuart8); //27
_DECLARE_ISR(lpi2c1); //28
_DECLARE_ISR(lpi2c2); //29
_DECLARE_ISR(lpi2c3); //30
_DECLARE_ISR(lpi2c4); //31
_DECLARE_ISR(lpspi1); //32
_DECLARE_ISR(lpspi2); //33
_DECLARE_ISR(lpspi3); //34
_DECLARE_ISR(lpspi4); //35
_DECLARE_ISR(can1); //36
_DECLARE_ISR(can2); //37
_DECLARE_ISR(flexram); //38
_DECLARE_ISR(kpp); //39
_DECLARE_ISR(tsc_dig); //40
_DECLARE_ISR(gpr_irq); //41
_DECLARE_ISR(lcdif); //42
_DECLARE_ISR(csi); //43
_DECLARE_ISR(pxp); //44
_DECLARE_ISR(wdog2); //45
_DECLARE_ISR(snvs_hp_wrapper); //46
_DECLARE_ISR(snvs_hp_wrapper_tz); //47
_DECLARE_ISR(snvs_lp_wrapper); //48
_DECLARE_ISR(csu); //49
_DECLARE_ISR(dcp); //50
_DECLARE_ISR(dcp_vmi); //51
_DECLARE_ISR(reserved68); //52
_DECLARE_ISR(trng); //53
_DECLARE_ISR(sjc); //54
_DECLARE_ISR(bee); //55
_DECLARE_ISR(sai1); //56
_DECLARE_ISR(sai2); //57
_DECLARE_ISR(sai3_rx); //58
_DECLARE_ISR(sai3_tx); //59
_DECLARE_ISR(spdif); //60
_DECLARE_ISR(anatop_event0); //61
_DECLARE_ISR(anatop_event1); //62
_DECLARE_ISR(anatop_tamp_low_high); //63
_DECLARE_ISR(anatop_temp_panic); //64
_DECLARE_ISR(usb_phy1); //65
_DECLARE_ISR(usb_phy2); //66
_DECLARE_ISR(adc1); //67
_DECLARE_ISR(adc2); //68
_DECLARE_ISR(dcdc); //69
_DECLARE_ISR(reserved86); //70
_DECLARE_ISR(reserved87); //71
_DECLARE_ISR(gpio1_int0); //72
_DECLARE_ISR(gpio1_int1); //73
_DECLARE_ISR(gpio1_int2); //74
_DECLARE_ISR(gpio1_int3); //75
_DECLARE_ISR(gpio1_int4); //76
_DECLARE_ISR(gpio1_int5); //77
_DECLARE_ISR(gpio1_int6); //78
_DECLARE_ISR(gpio1_int7); //79
_DECLARE_ISR(gpio1_combined_0_15); //80
_DECLARE_ISR(gpio1_combined_16_31); //81
_DECLARE_ISR(gpio2_combined_0_15); //82
_DECLARE_ISR(gpio2_combined_16_31); //83
_DECLARE_ISR(gpio3_combined_0_15); //84
_DECLARE_ISR(gpio3_combined_16_31); //85
_DECLARE_ISR(gpio4_combined_0_15); //86
_DECLARE_ISR(gpio4_combined_16_31); //87
_DECLARE_ISR(gpio5_combined_0_15); //88
_DECLARE_ISR(gpio5_combined_16_31); //89
_DECLARE_ISR(flexio1); //90
_DECLARE_ISR(flexio2); //91
_DECLARE_ISR(wdog1); //92
_DECLARE_ISR(rtwdog); //93
_DECLARE_ISR(ewm); //94
_DECLARE_ISR(ccm_1); //95
_DECLARE_ISR(ccm_2); //96
_DECLARE_ISR(gpc); //97
_DECLARE_ISR(src); //98
_DECLARE_ISR(reserved115); //99
_DECLARE_ISR(tmr1); //100 - call these 'tmr' instead of 'gpt', so OS will use this for internal timers
_DECLARE_ISR(tmr2); //101
_DECLARE_ISR(pwm1_0); //102
_DECLARE_ISR(pwm1_1); //103
_DECLARE_ISR(pwm1_2); //104
_DECLARE_ISR(pwm1_3); //105
_DECLARE_ISR(pwm1_fault); //106
_DECLARE_ISR(reserved123); //107
_DECLARE_ISR(flexspi); //108
_DECLARE_ISR(semc); //109
_DECLARE_ISR(usdhc1); //110
_DECLARE_ISR(usdhc2); //111
_DECLARE_ISR(usb_otg2); //112
_DECLARE_ISR(usb_otg1); //113
_DECLARE_ISR(enet); //114
_DECLARE_ISR(enet_1588_timer); //115
_DECLARE_ISR(xbar1_irq_0_1); //116
_DECLARE_ISR(xbar1_irq_2_3); //117
_DECLARE_ISR(adc_etc_irq0); //118
_DECLARE_ISR(adc_etc_irq1); //119
_DECLARE_ISR(adc_etc_irq2); //120
_DECLARE_ISR(adc_etc_error_irq); //121
_DECLARE_ISR(pit); //122
_DECLARE_ISR(acmp1); //123
_DECLARE_ISR(acmp2); //124
_DECLARE_ISR(acmp3); //125
_DECLARE_ISR(acmp4); //126
_DECLARE_ISR(reserved143); //127
_DECLARE_ISR(reserved144); //128
_DECLARE_ISR(enc1); //129
_DECLARE_ISR(enc2); //130
_DECLARE_ISR(enc3); //131
_DECLARE_ISR(enc4); //132
_DECLARE_ISR(qtmr1); //133
_DECLARE_ISR(qtmr2); //134
_DECLARE_ISR(qtmr3); //135
_DECLARE_ISR(qtmr4); //136
_DECLARE_ISR(pwm2_0); //137
_DECLARE_ISR(pwm2_1); //138
_DECLARE_ISR(pwm2_2); //139
_DECLARE_ISR(pwm2_3); //140
_DECLARE_ISR(pwm2_fault); //141
_DECLARE_ISR(pwm3_0); //142
_DECLARE_ISR(pwm3_1); //143
_DECLARE_ISR(pwm3_2); //144
_DECLARE_ISR(pwm3_3); //145
_DECLARE_ISR(pwm3_fault); //146
_DECLARE_ISR(pwm4_0); //147
_DECLARE_ISR(pwm4_1); //148
_DECLARE_ISR(pwm4_2); //149
_DECLARE_ISR(pwm4_3); //150
_DECLARE_ISR(pwm4_fault); //151
_DECLARE_ISR(reserved168); //152
_DECLARE_ISR(reserved169); //153
_DECLARE_ISR(reserved170); //154
_DECLARE_ISR(reserved171); //155
_DECLARE_ISR(reserved172); //156
_DECLARE_ISR(reserved173); //157
_DECLARE_ISR(sjc_arm_debug); //158
_DECLARE_ISR(nmi_wakeup); //159

/* hook for NXP's FlexSPI configuration */
extern void * __Vectors __attribute__ ((alias ("mcu_core_vector_table")));

void (* const mcu_core_vector_table[])() __attribute__ ((section(".startup"))) = {
		// Core Level - CM3
		(void*)&_top_of_stack,					// The initial stack pointer
		mcu_core_reset_handler,						// The reset handler
		mcu_core_nmi_isr,							// The NMI handler
		mcu_core_hardfault_handler,					// The hard fault handler
		mcu_core_memfault_handler,					// The MPU fault handler
		mcu_core_busfault_handler,					// The bus fault handler
		mcu_core_usagefault_handler,				// The usage fault handler
		mcu_core_hardware_id,					// Reserved
		0,										// Reserved
		(void*)&mcu_core_bootloader_api,										// Reserved -- this is the kernel signature checksum value 0x24
		0,										// Reserved
		mcu_core_svcall_handler,					// SVCall handler
		0,					// Debug monitor handler
		0,										// Reserved
		mcu_core_pendsv_handler,					// The PendSV handler
		mcu_core_systick_handler,					// The SysTick handler
		//Non Cortex M interrupts (device specific interrupts)

		_ISR(dma0_dma16), //0
		_ISR(dma1_dma17), //1
		_ISR(dma2_dma18), //2
		_ISR(dma3_dma19), //3
		_ISR(dma4_dma20), //4
		_ISR(dma5_dma21), //5
		_ISR(dma6_dma22), //6
		_ISR(dma7_dma23), //7
		_ISR(dma8_dma24), //8
		_ISR(dma9_dma25), //9
		_ISR(dma10_dma26), //10
		_ISR(dma11_dma27), //11
		_ISR(dma12_dma28), //12
		_ISR(dma13_dma29), //13
		_ISR(dma14_dma30), //14
		_ISR(dma15_dma31), //15
		_ISR(dma_error), //16
		_ISR(cti0_error), //17
		_ISR(cti1_error), //18
		_ISR(core), //19
		_ISR(lpuart1), //20
		_ISR(lpuart2), //21
		_ISR(lpuart3), //22
		_ISR(lpuart4), //23
		_ISR(lpuart5), //24
		_ISR(lpuart6), //25
		_ISR(lpuart7), //26
		_ISR(lpuart8), //27
		_ISR(lpi2c1), //28
		_ISR(lpi2c2), //29
		_ISR(lpi2c3), //30
		_ISR(lpi2c4), //31
		_ISR(lpspi1), //32
		_ISR(lpspi2), //33
		_ISR(lpspi3), //34
		_ISR(lpspi4), //35
		_ISR(can1), //36
		_ISR(can2), //37
		_ISR(flexram), //38
		_ISR(kpp), //39
		_ISR(tsc_dig), //40
		_ISR(gpr_irq), //41
		_ISR(lcdif), //42
		_ISR(csi), //43
		_ISR(pxp), //44
		_ISR(wdog2), //45
		_ISR(snvs_hp_wrapper), //46
		_ISR(snvs_hp_wrapper_tz), //47
		_ISR(snvs_lp_wrapper), //48
		_ISR(csu), //49
		_ISR(dcp), //50
		_ISR(dcp_vmi), //51
		_ISR(reserved68), //52
		_ISR(trng), //53
		_ISR(sjc), //54
		_ISR(bee), //55
		_ISR(sai1), //56
		_ISR(sai2), //57
		_ISR(sai3_rx), //58
		_ISR(sai3_tx), //59
		_ISR(spdif), //60
		_ISR(anatop_event0), //61
		_ISR(anatop_event1), //62
		_ISR(anatop_tamp_low_high), //63
		_ISR(anatop_temp_panic), //64
		_ISR(usb_phy1), //65
		_ISR(usb_phy2), //66
		_ISR(adc1), //67
		_ISR(adc2), //68
		_ISR(dcdc), //69
		_ISR(reserved86), //70
		_ISR(reserved87), //71
		_ISR(gpio1_int0), //72
		_ISR(gpio1_int1), //73
		_ISR(gpio1_int2), //74
		_ISR(gpio1_int3), //75
		_ISR(gpio1_int4), //76
		_ISR(gpio1_int5), //77
		_ISR(gpio1_int6), //78
		_ISR(gpio1_int7), //79
		_ISR(gpio1_combined_0_15), //80
		_ISR(gpio1_combined_16_31), //81
		_ISR(gpio2_combined_0_15), //82
		_ISR(gpio2_combined_16_31), //83
		_ISR(gpio3_combined_0_15), //84
		_ISR(gpio3_combined_16_31), //85
		_ISR(gpio4_combined_0_15), //86
		_ISR(gpio4_combined_16_31), //87
		_ISR(gpio5_combined_0_15), //88
		_ISR(gpio5_combined_16_31), //89
		_ISR(flexio1), //90
		_ISR(flexio2), //91
		_ISR(wdog1), //92
		_ISR(rtwdog), //93
		_ISR(ewm), //94
		_ISR(ccm_1), //95
		_ISR(ccm_2), //96
		_ISR(gpc), //97
		_ISR(src), //98
		_ISR(reserved115), //99
		_ISR(tmr1), //100 - call these 'tmr' instead of 'gpt', so OS will use this for internal timers
		_ISR(tmr2), //101
		_ISR(pwm1_0), //102
		_ISR(pwm1_1), //103
		_ISR(pwm1_2), //104
		_ISR(pwm1_3), //105
		_ISR(pwm1_fault), //106
		_ISR(reserved123), //107
		_ISR(flexspi), //108
		_ISR(semc), //109
		_ISR(usdhc1), //110
		_ISR(usdhc2), //111
		_ISR(usb_otg2), //112
		_ISR(usb_otg1), //113
		_ISR(enet), //114
		_ISR(enet_1588_timer), //115
		_ISR(xbar1_irq_0_1), //116
		_ISR(xbar1_irq_2_3), //117
		_ISR(adc_etc_irq0), //118
		_ISR(adc_etc_irq1), //119
		_ISR(adc_etc_irq2), //120
		_ISR(adc_etc_error_irq), //121
		_ISR(pit), //122
		_ISR(acmp1), //123
		_ISR(acmp2), //124
		_ISR(acmp3), //125
		_ISR(acmp4), //126
		_ISR(reserved143), //127
		_ISR(reserved144), //128
		_ISR(enc1), //129
		_ISR(enc2), //130
		_ISR(enc3), //131
		_ISR(enc4), //132
		_ISR(qtmr1), //133
		_ISR(qtmr2), //134
		_ISR(qtmr3), //135
		_ISR(qtmr4), //136
		_ISR(pwm2_0), //137
		_ISR(pwm2_1), //138
		_ISR(pwm2_2), //139
		_ISR(pwm2_3), //140
		_ISR(pwm2_fault), //141
		_ISR(pwm3_0), //142
		_ISR(pwm3_1), //143
		_ISR(pwm3_2), //144
		_ISR(pwm3_3), //145
		_ISR(pwm3_fault), //146
		_ISR(pwm4_0), //147
		_ISR(pwm4_1), //148
		_ISR(pwm4_2), //149
		_ISR(pwm4_3), //150
		_ISR(pwm4_fault), //151
		_ISR(reserved168), //152
		_ISR(reserved169), //153
		_ISR(reserved170), //154
		_ISR(reserved171), //155
		_ISR(reserved172), //156
		_ISR(reserved173), //157
		_ISR(sjc_arm_debug), //158
		_ISR(nmi_wakeup) //159
		};

void mcu_core_reset_handler(){
	core_init();
	cortexm_set_vector_table_addr((void*)mcu_core_vector_table);
	_main(); //This function should never return
	mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, "main");
	while(1){
		;
	}
}

void mcu_core_hardware_id(){
	mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, "hwid");
}

void mcu_core_debugmon_handler(){
	mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, "dbgmon");
}

void mcu_core_default_isr(){
	mcu_board_execute_event_handler(MCU_BOARD_CONFIG_EVENT_ROOT_FATAL, "dflt");
}
