
INCLUDE "RT1050_DEV_Debug.ld"

SECTIONS {

	 .SRAM_ITC : {
		_sys = . ;
		*(.SRAM_ITC)
		. = ALIGN(1024);
	} >SRAM_ITC
	_esys = .;

}

PROVIDE ( _end = . );
PROVIDE ( _top_of_stack = (ORIGIN(SRAM_ITC) + LENGTH(SRAM_ITC)) );
PROVIDE ( _estack = (ORIGIN(SRAM_ITC) + LENGTH(SRAM_ITC)) );
PROVIDE ( _flash_start = ORIGIN(BOARD_FLASH) );
PROVIDE ( _text = _image_start );
PROVIDE ( _unique_id = 0x1FF0F420 );
