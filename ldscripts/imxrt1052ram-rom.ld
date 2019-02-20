flash_page_count = 0;

MEMORY {
	flash (rwx) : ORIGIN = 0x0, LENGTH = 256K /* ITCM */
	sysmem (rwx) : ORIGIN = 0x20000000, LENGTH = 64K /* 128K bytes (DTCM) - 64K for SYSMEM */
	ram (rwx) : ORIGIN = ORIGIN(sysmem) + LENGTH(sysmem), LENGTH = 64K /* 64K DTCM*/
	ram2 (rwx) : ORIGIN = 0x20200000, LENGTH = 128K /* 128K bytes (OCRAM) -- for applications on mem_dev */
}

INCLUDE ldscripts/imxrt105x.ld

PROVIDE ( _unique_id = 0x401f4410 );

