/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2015
 * (c) NXP Semiconductors 2013-2015
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v7.7.2 [Build 379] [2015-03-31]  on Tue Oct 20 13:26:12 EEST 2015)
*/

MEMORY
{
  Flash_00 (rx) : ORIGIN = 0x0, LENGTH = 0x800             /* 2K byte */
  MFlash32 (rx) : ORIGIN = 0x800, LENGTH = 0x7800          /* 30K bytes */
  Rom_Vars (rw) : ORIGIN = 0x10000000, LENGTH = 8        /*  8 bytes */
  CommonRAM1 (rw) : ORIGIN = 0x10000000, LENGTH = 0x2000-8    /* 4k bytes */
  CommonRAM2 (rw) : ORIGIN = 0x10000000, LENGTH = 0x2000-8    /*4k bytes */
  RamLoc8 (rw) : ORIGIN = 0x10000008, LENGTH = 0x2000 -8     /* 8K bytes */
  Boot_RAM (rw) : ORIGIN = 0x10000008, LENGTH = 0x2000 -8  /*  8K bytes */

}
  /* Define a symbol for the top of each memory region */
  __top_Flash_00 = 0x0 + 0x800;
  __top_MFlash32 = 0x800 + 0x7800;
  __top_RamLoc8 = 0x10000000 + 0x2000;







