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
 * along with Stratify OS.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef MCU_MIMXRT1050_H_
#define MCU_MIMXRT1050_H_

#include <mcu/types.h>
#include "cmsis/MIMXRT1052.h"
#include "cmsis/MIMXRT1052_features.h"

#define MCU_NO_HARD_FAULT 1

#define MCU_ADC_API 0
#define MCU_ADC_PORTS 0
#define MCU_ADC_REGS 0
#define MCU_ADC_IRQS 0
#define MCU_ADC_CHANNELS 0

#define MCU_ENET_PORTS 0
#define MCU_FLASH_PORTS 0
#define MCU_MEM_PORTS 0

#define MCU_I2C_API 0
#define MCU_I2C_PORTS 5
#define MCU_I2C_REGS LPI2C_BASE_PTRS
#define MCU_I2C_IRQS LPI2C_IRQS

#define MCU_CORE_PORTS 1

#define MCU_SPI_API 0
#define MCU_SPI_PORTS 5
#define MCU_SPI_REGS LPSPI_BASE_PTRS
#define MCU_SPI_IRQS LPSPI_IRQS

#define MCU_SAI_API 0
#define MCU_SAI_PORTS 0

#define MCU_SDIO_API 0
#define MCU_SDIO_PORTS 0
#define MCU_SDIO_REGS 0
#define MCU_SDIO_IRQS 0

#define MCU_TMR_PORTS 2
#define MCU_TMR_REGS { GPT1, GPT2 }
#define MCU_TMR_IRQS { GPT1_IRQn, GPT2_IRQn }

#define MCU_PIO_PORTS 5
#define MCU_PIO_REGS { GPIO1, GPIO2, GPIO3, GPIO4, GPIO5 }
//#define MCU_PIO_IRQS GPIO_IRQS doesn't work as expected IRQs aren't associated with port

#define MCU_UART_PORTS 8
#define MCU_UART_REGS { LPUART1, LPUART2, LPUART3, LPUART4, LPUART5, LPUART6, LPUART7, LPUART8 }
#define MCU_UART_IRQS { LPUART1_IRQn, LPUART2_IRQn, LPUART3_IRQn, LPUART4_IRQn, LPUART5_IRQn, LPUART6_IRQn, LPUART7_IRQn, LPUART8_IRQn }

#define MCU_USB_API 1
#define MCU_USB_PORTS 3
#define MCU_USB_REGS USB_BASE_PTRS
#define MCU_USB_IRQS USB_IRQS

#define MCU_DMA_PORTS 2


#define MCU_RTC_PORTS 1
#define MCU_RTC_REGS { RTC }
#define MCU_RTC_IRQS { RTC_Alarm_IRQn }

#define DEV_USB_LOGICAL_ENDPOINT_COUNT 4

#define MCU_LAST_IRQ NMI_WAKEUP_IRQn
#define MCU_MIDDLE_IRQ_PRIORITY 8


#define MCU_RAM_PAGES 80
#define MCU_DELAY_FACTOR 12
#define MCU_TOTAL_PINS (7*16+2)


#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#endif /* MCU_MIMXRT1050_H_ */
