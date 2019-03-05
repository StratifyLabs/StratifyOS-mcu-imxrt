flash_page_count = 0;

MEMORY {
	flash (rx) : ORIGIN = 0x60000000, LENGTH = 0x4000000 /* 64M bytes (HyperFlash) */
	sysmem (rwx) : ORIGIN = 0x20000000, LENGTH = 64K /* 64KB bytes (DTCM) for system memory */
	ram (rwx) : ORIGIN = ORIGIN(sysmem) + LENGTH(sysmem), LENGTH = 64K /* 256KB DTCM 192KB bytes for general RAM */
	ram1 (rwx) : ORIGIN = 0x20200000, LENGTH = 256K /* 256K bytes (OCRAM) */
}

SECTIONS
{
	.boot_hdr : ALIGN(4)
	{
		FILL(0xff)
		__boot_hdr_start__ = ABSOLUTE(.) ;
		KEEP(*(.boot_hdr.conf))
		. = 0x1000 ;
		KEEP(*(.boot_hdr.ivt))
		. = 0x1020 ;
		KEEP(*(.boot_hdr.boot_data))
		. = 0x1030 ;
		KEEP(*(.boot_hdr.dcd_data))
		__boot_hdr_end__ = ABSOLUTE(.) ;
		. = 0x2000 ;
	} >flash
}

INCLUDE ldscripts/imxrt105x.ld

PROVIDE ( _unique_id = 0x401f4410 );

