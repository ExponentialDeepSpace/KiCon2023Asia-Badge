/**
  ****************************************************************************
  Copyright (c) 2019, Nations Technologies Inc.

  All rights reserved.
  ****************************************************************************

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  - Redistributions of source code must retain the above copyright notice,
  this list of conditions and the disclaimer below.

  Nations' name may not be used to endorse or promote products derived from
  this software without specific prior written permission.

  DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
  DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************
 **/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = 0x20008000;    /* end of RAM */

/*
    Sector Size: 512 Byte
    Cluster Size: 4 sectors = 2KB = one Flash Page

    screen data size is 72 * 144 / 2

    KiCon Background image:
    size is 72(H) * 288(W) / 2 = 10368 bytes
    sectors: (10368 + 511) / 512 = 21 sectors
    clusters: (21 + 3) / 4 = 6 clusters
    Flash: 6 clusters * 4 (sectors per cluster) * 512 (bytes per sector) = 12288 bytes = 12K

    Name image:
    size is 72(H) * 144(W) / 2 = 5184 bytes
    sectors: (5184 + 511) / 512 = 11 sectors
    clusters: (11 + 3) / 4 = 3 clusters
    Flash: 3 clusters * 4 (sectors per cluster) * 512 (bytes per sector) = 6144 Bytes = 6K
*/

/* N32L40x has 2KB as one Page */
_Sector_Size = 2048;
_Sectors_Per_Cluster = 1;
_Flash_Page_Size = _Sector_Size * _Sectors_Per_Cluster;

_Bg_Image_Size = (72 /* width */ / 4 /* every two pixels is one byte, use 16-bit word as one data */ + 1 /* one prefix row number */ ) * 288 * 2;
_Bg_Image_Sectors = (_Bg_Image_Size + _Sector_Size - 1) / _Sector_Size;
_Bg_Image_Clusters = (_Bg_Image_Sectors + _Sectors_Per_Cluster - 1) / _Sectors_Per_Cluster;
_Bg_Image_Flash_Size = _Bg_Image_Clusters * _Flash_Page_Size;

_Name_Image_Size = (72 / 4 + 1) * 144 * 2;
_Name_Image_Sectors = (_Name_Image_Size + _Sector_Size - 1) / _Sector_Size;
_Name_Image_Clusters = (_Name_Image_Sectors + _Sectors_Per_Cluster - 1) / _Sectors_Per_Cluster;
_Name_Image_Flash_Size = _Name_Image_Clusters * _Flash_Page_Size;

_Usb_Disk_Size = _Bg_Image_Flash_Size + _Name_Image_Flash_Size;

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x800; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 128K
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))
	
	/* section information for finsh shell */
	. = ALIGN(4);
	__fsymtab_start = .;
	KEEP(*(FSymTab))
	__fsymtab_end = .;

	. = ALIGN(4);
	__vsymtab_start = .;
	KEEP(*(VSymTab))
	__vsymtab_end = .;

	/* section information for initial. */
	. = ALIGN(4);
	__rt_init_start = .;
	KEEP(*(SORT(.rti_fn*)))
	__rt_init_end = .;

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH
  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(4);
  } >RAM

  ._user_usb_disk_space :
  {
    . = ORIGIN(FLASH) + LENGTH(FLASH) - _Usb_Disk_Size;
    . = ALIGN(4);
    __disk_start = .;
    __bg_image_start = .;
    . = __bg_image_start + _Bg_Image_Flash_Size;
    __bg_image_end = .;
    __name_image_start = .;
    . = __name_image_start + _Name_Image_Flash_Size;
    __name_image_end = .;
    __disk_end = .;
  } >FLASH
    

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
