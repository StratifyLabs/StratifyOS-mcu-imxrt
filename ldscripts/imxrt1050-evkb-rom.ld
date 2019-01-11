/* Memory regions for NXP IMXRT1050-EVKB evaluation board. */

flash_page_count = 1;

MEMORY
{
  flash (rx) : ORIGIN = 0x60000000, LENGTH = 0x4000000 /* 64M bytes (HyperFlash) */
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000 /* 128K bytes (DTCM) */
  ram1 (rwx) : ORIGIN = 0x0, LENGTH = 0x20000 /* 128K bytes (ITCM) */
  sysmem (rwx) : ORIGIN = 0x20200000, LENGTH = 0x40000 /* 256K bytes (OCRAM) */
  ram2 (rwx) : ORIGIN = 0x80000000, LENGTH = 0x2000000 /* 32M bytes (SDRAM) */
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
        . = 0x40000 ; /* MPU boundary alignment */
    } >flash
}

INCLUDE ldscripts/imxrt105x.ld

PROVIDE ( _unique_id = 0x1FF07A10 );

/*
 * Needed for apps ...
PROVIDE ( _app_ram_size = _sram2_size );
*/
