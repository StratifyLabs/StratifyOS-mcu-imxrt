flash_page_count = 6;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 1024K
	 ram : ORIGIN = 0x20000000, LENGTH = 512K
	 sysmem : ORIGIN = ORIGIN(ram) + LENGTH(ram), LENGTH = 16K
}

INCLUDE ldscripts/imxrt105x.ld

PROVIDE ( _unique_id = 0x1FF07A10 );

