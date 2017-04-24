/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-13
 * (c) NXP Semiconductors 2013-2015
 * Generated linker script file for LPC824
 * Created from generic_c.ld (7.7.2 ())
 * By LPCXpresso v7.7.2 [Build 379] [2015-03-31]  on Mon Sep 14 08:22:36 EEST 2015
 */

INCLUDE "T0_Release_lib.ld"
INCLUDE "T0_Release_mem.ld"

ENTRY(ResetISR)

SECTIONS
{
 
     /* MAIN TEXT SECTION */    
    .bootloader : ALIGN(4)
    {
        FILL(0xff)
           
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))     
        *src/aeabi_romdiv_patch.o
        
         /* Code Read Protect data */
        . = 0x000002FC ;
        PROVIDE(__CRP_WORD_START__ = .) ;
        KEEP(*(.crp))
        PROVIDE(__CRP_WORD_END__ = .) ;
        ASSERT(!(__CRP_WORD_START__ == __CRP_WORD_END__), "Linker CRP Enabled, but no CRP_WORD provided within application");
        /* End of Code Read Protect */
        
    	*(.bootloader_func*)  
    		
    	
    	
      /*  *(.boot_string*)*/
        
    } > Flash_00
    
    
    .text_Flash2 : ALIGN(4)
    {
       FILL(0xff)
    	*(.text_Flash2*) /* for compatibility with previous releases */
    	*(.text_MFlash32*) /* for compatibility with previous releases */
    	*(.text.$Flash2*)
    	*(.text.$MFlash32*)
    	*(.rodata.$Flash2*)
    	*(.rodata.$MFlash32*)
    } > MFlash32

   
    
    .text : ALIGN(4)    
    {
        FILL(0xff)
         . = ALIGN(4) ;
        __section_table_start = .;
        
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
       __data_section_table_end = .;
       
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
         __bss_section_table_end = .;
         
        __section_table_end = . ;
        /* End of Global Section Table */
        *(.nmi_handler_jmp_address*)
        *(.hard_fault_handler_jmp_address*)
        *(.svc_handler_jmp_address*)
        *(.pendsv_handler_jmp_address*)
        *(.systick_handler_jmp_address*)
        *(.spi0_isr_jmp_address*)
        *(.spi1_isr_jmp_address*)
        *(.uart0_isr_jmp_address*)
        *(.uart1_isr_jmp_address*)
        *(.uart2_isr_jmp_address*)
        *(.i2c0_isr_jmp_address*)
        *(.i2c1_isr_jmp_address*)
        *(.i2c2_isr_jmp_address*)
        *(.i2c3_isr_jmp_address*)
        *(.sct_isr_jmp_address*)
        *(.mrt_isr_jmp_address*)
        *(.cmp_isr_jmp_address*)
        *(.wdt_isr_jmp_address*)
        *(.bod_isr_jmp_address*)
        *(.flash_isr_jmp_address*)
        *(.wkt_isr_jmp_address*)
        *(.adc_seqa_isr_jmp_address*)
        *(.adc_seqb_isr_jmp_address*)
        *(.adc_thcmp_isr_jmp_address*)
        *(.adc_ovr_isr_jmp_address*)
        *(.dma_isr_jmp_address*)
        *(.pin_int0_isr_jmp_address*)
        *(.pin_int1_isr_jmp_address*)
        *(.pin_int2_isr_jmp_address*)
        *(.pin_int3_isr_jmp_address*)
        *(.pin_int4_isr_jmp_address*)
        *(.pin_int5_isr_jmp_address*)
        *(.pin_int6_isr_jmp_address*)
        *(.pin_int7_isr_jmp_address*)
        /* Global Section Table */
       
       *(.application_entry*)
        *(.after_vectors*)
        *(EXCLUDE_FILE(*src/cr_startup_lpc82x.o) .text .text.* .rodata .rodata.* .constdata .constdata.*)
    /*    *(EXCLUDE_FILE(*src/bootloader.o).text .text*)*/
        *(EXCLUDE_FILE(*src/aeabi_romdiv_patch.o).text .text*)
        
         *(.text*)
        *(.rodata .rodata.* .constdata .constdata.*)
        . = ALIGN(4);
        
    } > MFlash32
    
 
  /* other placements follow here... */

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4)
    {
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    } > MFlash32
    __exidx_start = .;
    
    .ARM.exidx : ALIGN(4)
    {
    	*(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > MFlash32
    __exidx_end = .;
    
    _etext = .;
        
    
    /* MAIN DATA SECTION */
    
    /* Default MTB section */
    .mtb_buffer_default (NOLOAD) :
    {
       KEEP(*(.mtb*))
    } > RamLoc8 

    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RamLoc8

	
	/* Main DATA section (RamLoc8) */
	.data : ALIGN(4)
	{
	   FILL(0xff)
	   _data = . ;
	   *(vtable)
	   *(.ramfunc*)
	   *(.data*)
	   . = ALIGN(4) ;
	   _edata = . ;
	}  > RamLoc8 AT>MFlash32
	
    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > RamLoc8
        
    
    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*) 
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RamLoc8
    
      
     .firmware_download_variables SIZEOF(.noinit) + ADDR(.noinit) (NOLOAD) : ALIGN(4)
    {
       PROVIDE(_firmware_download_variables_start = .);
       *(.common2*)
       PROVIDE(_firmware_download_variables_end = .);
      } > CommonRAM2
    
    .gps_variables SIZEOF(.noinit) + ADDR(.noinit) (NOLOAD)  : ALIGN(4)
    {
     PROVIDE(_gps_variables_start = .);
       *(.common1*)
     PROVIDE(_gps_variables_end = .);
    }  > CommonRAM1
    
    .bootloader_data (NOLOAD) : ALIGN(4)
 	 {  
  	   PROVIDE(_bootloader_data_start = .);
  	    KEEP(*(.bootloader_ram*)) 
  	   PROVIDE(_bootloader_data_end = .);
  	 } > Boot_RAM 
  	 
 	 .rom_vars   (NOLOAD):
  	 {
  	 	*(.romDiv_Vars*) 
  /*	 	 *(COMMON)*/
  	 } > Rom_Vars  
  	 
    PROVIDE(_pvHeapStart = DEFINED(__user_heap_base) ? __user_heap_base : .);
    PROVIDE(_vStackTop = DEFINED(__user_stack_top) ? __user_stack_top : __top_RamLoc8);
  	PROVIDE(_vStackBottom =__top_RamLoc8 - 1024);
}
