/*
*****************************************************************************
**
**  File        : stm32_rom.ld
**
**  Abstract    : Linker script for STM32F072RB Device with
**                128KByte rom, 16KByte ram
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Environment : Atollic TrueSTUDIO(R)
**
**  Distribution: The file is distributed �as is,� without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. This file may only be built (assembled or compiled and linked)
**  using the Atollic TrueSTUDIO(R) product. The use of this file together
**  with other tools than Atollic TrueSTUDIO(R) is not permitted.
**
*****************************************************************************
*/

/* Enforce emmition of the vector table. */
EXTERN (vector_table)

/* Entry Point */
ENTRY(reset_handler)

/* Specify the memory areas */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 128K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}


/* Define output sections */
SECTIONS
{
  /* The program code and other data goes into rom */
  .text : {
		. = ALIGN(4);
		*(.vectors)	/* Vector table */
	    *(.text)           /* .text sections (code) */
	    *(.text*)          /* .text* sections (code) */
	    *(.glue_7)         /* glue arm to thumb code */
	    *(.glue_7t)        /* glue thumb to arm code */
	    *(.eh_frame)
	
	    KEEP (*(.init))
	    KEEP (*(.fini))
		    
	} >rom

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >rom

/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {
		. = ALIGN(4);
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array))
		  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >rom
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >rom
	.fini_array : {
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >rom

	/*
	 * Another section used by C++ stuff, appears when using newlib with
	 * 64bit (long long) printf support
	 */
	.ARM.extab : {
		*(.ARM.extab*)
	} >rom
	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >rom

	. = ALIGN(4);
	_etext = .;

  .data : {
		_data = .;
		*(.data*)	/* Read-write initialized data */
		. = ALIGN(4);
		_edata = .;
	} >ram AT >rom
	_data_loadaddr = LOADADDR(.data);
  
  . = ALIGN(4);
	.bss : {
		sbss = .;         /* define a global symbol at bss start */
		__bss_start__ = sbss;
		*(.bss*)	/* Read-write zero initialized data */
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = _ebss;
	} >ram
  

  /* Remove information from the standard libraries */
 /DISCARD/ : {
	
	libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
	
	 /* *(.eh_frame) */ 
	 } 

  .ARM.attributes 0 : { *(.ARM.attributes) }
 
  
  /*. = ALIGN(4);
	end = .;*/
}

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));

