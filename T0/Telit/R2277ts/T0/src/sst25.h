/* FLASH ID and Manufacturer*****************************************************************/
#define  S25FL208    0x00014014      /* Spansion 8Mb SPI Flash */
/* SST25 Instructions ***************************************************************/
/*      Command                    Value      Description               Addr   Data */
/*                                                                         Dummy    */
#define SST25_READ                  0x03    /* Read data bytes           3   0  >=1 */
#define SST25_FAST_READ             0x0b    /* Higher speed read         3   1  >=1 */
#define SST25_SE                    0x20    /* 4Kb Sector erase          3   0   0  */
#define SST25_BE32                  0x52    /* 32Kbit block Erase        3   0   0  */
#define SST25_BE64                  0xd8    /* 64Kbit block Erase        3   0   0  */
#define SST25_CE                    0xc7    /* Chip erase                0   0   0  */
#define SST25_CE_ALT                0x60    /* Chip erase (alternate)    0   0   0  */
#define SST25_BP                    0x02    /* Byte program              3   0   1  */
#define SST25_AAI                   0xad    /* Auto address increment    3   0  >=2 */
#define SST25_RDSR                  0x05    /* Read status register      0   0  >=1 */
#define SST25_EWSR                  0x50    /* Write enable status       0   0   0  */
#define SST25_WRSR                  0x01    /* Write Status Register     0   0   1  */
#define SST25_WREN                  0x06    /* Write Enable              0   0   0  */
#define SST25_WRDI                  0x04    /* Write Disable             0   0   0  */
#define SST25_RDID                  0xab    /* Read Identification       0   0  >=1 */
#define SST25_RDID_ALT              0x90    /* Read Identification (alt) 0   0  >=1 */
#define SST25_JEDEC_ID              0x9f    /* JEDEC ID read             0   0  >=3 */
#define SST25_EBSY                  0x70    /* Enable SO RY/BY# status   0   0   0  */
#define SST25_DBSY                  0x80    /* Disable SO RY/BY# status  0   0   0  */

#define SST25_SECTOR_SIZE           4096
#define SST25_FLASH_SIZE            (1024 * 1024)
#define SST25_SECTOR_COUNT          (SST25_FLASH_SIZE / SST25_SECTOR_SIZE)
#define SST25_FLASH_SECTOR_MASK	    0x00000FFF


#define SST25_BP0                   4
#define SST25_BP1                   8
#define SST25_BP2                  16


/* protected flash block starts from sector 225*/
#define  PROTECTED_BLOCK_START_ADDRESS          (225 * SST25_SECTOR_SIZE)
#define  OFFLINE_DATA_WRITE_ADDRESS             (235 * SST25_SECTOR_SIZE)
#define  OFFLINE_DATA_READ_ADDRESS              (236 * SST25_SECTOR_SIZE)
#define  KM_COUNTER_FLASH_ADDRESS               (237 * SST25_SECTOR_SIZE)
#define  FIRST_FLASH_SETTINGS_ADDRESS           (238 * SST25_SECTOR_SIZE)
#define  SECOND_FLASH_SETTINGS_ADDRESS          (239 * SST25_SECTOR_SIZE)
#define  FIRMWARE_MAGICWORD_ADDRESS             (240 * SST25_SECTOR_SIZE)
#define  FIRMWARE_BACKUP_ADDRESS                (241 * SST25_SECTOR_SIZE)
#define  FIRMWARE_ADDRESS                       (248 * SST25_SECTOR_SIZE)

/*upper and lower boundries for log records*/
#define  LOG_UPPER_BOUNDARY_ADDRESS             (PROTECTED_BLOCK_START_ADDRESS)
#define  LOG_LOWER_BOUNDARY_ADDRESS             (0)

#define  LOG_UPPER_BOUNDARY_SECTOR              (LOG_UPPER_BOUNDARY_ADDRESS / SST25_SECTOR_SIZE)
#define  LOG_LOWER_BOUNDARY_SECTOR              (LOG_LOWER_BOUNDARY_ADDRESS / SST25_SECTOR_SIZE)
/**/

#define FLASH_CHIP_ERASE_TIME                   (15 * SECOND)
#define FLASH_SECTOR_ERASE_TIME                 30           /* 300 ms */
#define FLASH_WRITE_BYTE_TIMEOUT                2            /* 20 ms  */
#define FLASH_WRITE_SR_TIME                     2

#define SST25_DUMMY_BYTE            0xFF

