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

#include <mcu/boot_debug.h>
#include "imxrt_flash.h"

u16 imxrt_flash_local_get_sector_size(u16 sector){
	return 128;
}

int imxrt_flash_write(u32 addr, const void * buf, int nbyte){
	int result = 0;

	cortexm_disable_interrupts();

	//RT1050 doesn't have internal flash

	cortexm_enable_interrupts();
	return result;
}

int imxrt_flash_erase_sector(u32 sector){
	cortexm_disable_interrupts();

	cortexm_enable_interrupts();
	return 0;
}

int imxrt_flash_blank_check(int loc, int nbyte){
	int i;
	const s8 * locp;
	//Do a blank check
	locp = (const s8*)loc;
	for(i = 0; i < nbyte; i++){
		if ( locp[i] != -1 ){
			return -1;
		}
	}
	return 0;
}

int imxrt_flash_get_sector_size(u32 sector){
	if( sector < MCU_FLASH_PAGE_COUNT ){
		return imxrt_flash_local_get_sector_size(sector)*1024;
	}

	return 0;
}

int imxrt_flash_get_sector_addr(u32 sector){
	u32 offset = 0;
	u16 sum = 0;
	int i;

	if( sector < MCU_FLASH_PAGE_COUNT ){

		for(i=0; i < sector; i++){
			sum += imxrt_flash_local_get_sector_size(i);
		}

		offset = sum * 1024 + MCU_FLASH_START;

		return offset;

	}

	return -1;
}

int imxrt_flash_is_flash(u32 addr, u32 size){
	if ( ((addr + size) <= (MCU_FLASH_SIZE + MCU_FLASH_START)) && (addr >= MCU_FLASH_START) ){
		return 1;
	}
	return 0;
}

int imxrt_flash_is_code(u32 addr, u32 size){
	if( addr + size <= MCU_FLASH_CODE_START ){
		return 0;
	}

	if( addr >= MCU_FLASH_CODE_END ){
		return 0;
	}

	return 1;
}

//Get the flash page that contains the address
int imxrt_flash_get_sector(u32 addr){
	u32 offset;
	u16 search = 0;
	int i;

	offset = (addr - MCU_FLASH_START)/1024;
	for(i=0; i < MCU_FLASH_PAGE_COUNT; i++){
		if( offset < search + imxrt_flash_local_get_sector_size(i) ){
			return i;
		}
		search += imxrt_flash_local_get_sector_size(i);
	}

	return -1;
}






