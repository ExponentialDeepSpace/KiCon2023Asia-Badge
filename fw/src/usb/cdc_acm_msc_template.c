/*
 * Copyright 2023 Yang Hongbo, Beijing Exponential Deep Space Ltd.,
 *
 * KiCon 2023 Asia Badge Firmware is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * KiCon 2023 Asia Badge Firmware is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with KiCon 2023 Asia Badge Firmware. If not, see
 * <https://www.gnu.org/licenses/>. 
 */

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_msc.h"

#if defined(CONFIG_USBDEV_MSC_THREAD)
#include "usb_osal.h"
#endif

#include "n32l40x.h"
#include <FreeRTOS.h>
#include "setup.h"
#include <string.h>

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define MSC_IN_EP  0x84
#define MSC_OUT_EP 0x05

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN + MSC_DESCRIPTOR_LEN)

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

#ifdef CONFIG_USB_HS
#define MSC_MAX_MPS 512
#else
#define MSC_MAX_MPS 64
#endif

/*!< global descriptor */
static const uint8_t cdc_msc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x03, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x02),
    MSC_DESCRIPTOR_INIT(0x02, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x00),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x26,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ' ', 0x00,                  /* wcChar9 */
    'C', 0x00,                  /* wcChar10 */
    '-', 0x00,                  /* wcChar11 */
    'M', 0x00,                  /* wcChar12 */
    ' ', 0x00,                  /* wcChar13 */
    'D', 0x00,                  /* wcChar14 */
    'E', 0x00,                  /* wcChar15 */
    'M', 0x00,                  /* wcChar16 */
    'O', 0x00,                  /* wcChar17 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '2', 0x00,                  /* wcChar3 */
    '1', 0x00,                  /* wcChar4 */
    '2', 0x00,                  /* wcChar5 */
    '3', 0x00,                  /* wcChar6 */
    '4', 0x00,                  /* wcChar7 */
    '5', 0x00,                  /* wcChar8 */
    '6', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x02,
    0x02,
    0x01,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[2048] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30 };

volatile bool ep_tx_busy_flag = false;

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

void usbd_event_handler(uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            /* setup first out ep read transfer */
            usbd_ep_start_read(CDC_OUT_EP, read_buffer, 2048);
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

void usbd_cdc_acm_bulk_out(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual out len:%d\r\n", nbytes);
    /* setup next out ep read transfer */
    usbd_ep_start_read(CDC_OUT_EP, read_buffer, 2048);
}

void usbd_cdc_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual in len:%d\r\n", nbytes);

    if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(CDC_IN_EP, NULL, 0);
    } else {
        ep_tx_busy_flag = false;
    }
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

struct usbd_interface intf0;
struct usbd_interface intf1;
struct usbd_interface intf2;

#if defined(CONFIG_USBDEV_MSC_THREAD)
usb_osal_sem_t usb_dma_sem;
#endif

void cdc_acm_msc_init(void)
{
    usbd_desc_register(cdc_msc_descriptor);
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf0));
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf1));
    usbd_add_endpoint(&cdc_out_ep);
    usbd_add_endpoint(&cdc_in_ep);
    usbd_add_interface(usbd_msc_init_intf(&intf2, MSC_OUT_EP, MSC_IN_EP));

    usb_dma_sem = usb_osal_sem_create(0);

    usbd_initialize();
}

volatile uint8_t dtr_enable = 0;

void usbd_cdc_acm_set_dtr(uint8_t intf, bool dtr)
{
    if (dtr) {
        dtr_enable = 1;
    } else {
        dtr_enable = 0;
    }
}

void cdc_acm_data_send_with_dtr_test(void)
{
    if (dtr_enable) {
        memset(&write_buffer[10], 'a', 2038);
        ep_tx_busy_flag = true;
        usbd_ep_start_write(CDC_IN_EP, write_buffer, 2048);
        while (ep_tx_busy_flag) {
        }
    }
}

// Ghost Fat code is referenced from
// blupill-bootloader(https://github.com/lupyuen/bluepill-bootloader)
// Microsoft FAT specification
// and http://elm-chan.org/docs/fat_e.html
// check virtual filesystem: `fsck.fat -n -v /dev/sda`

typedef struct {
    uint8_t JumpInstruction[3];
    uint8_t OEMInfo[8];
    uint16_t SectorSize;
    uint8_t SectorsPerCluster;
    uint16_t ReservedSectors;
    uint8_t FATCopies;
    uint16_t RootDirectoryEntries;
    uint16_t TotalSectors16;
    uint8_t MediaDescriptor;
    uint16_t FATSz16;
    uint16_t SectorsPerTrack;
    uint16_t Heads;
    uint32_t HiddenSectors;
    uint32_t TotalSectors32;
    uint8_t PhysicalDriveNum;
    uint8_t Reserved;
    uint8_t ExtendedBootSig;
    uint32_t VolumeSerialNumber;
    uint8_t VolumeLabel[11];
    uint8_t FilesystemIdentifier[8];
    uint8_t BootCode[448];
    uint16_t BootSign;
} __attribute__((packed)) FAT_BootBlock;

typedef struct {
    char name[11];
    uint8_t attrs;
    uint8_t reserved;
    uint8_t createTimeFine;
    uint16_t createTime;
    uint16_t createDate;
    uint16_t lastAccessDate;
    uint16_t highStartCluster;
    uint16_t updateTime;
    uint16_t updateDate;
    uint16_t startCluster;
    uint32_t size;
} __attribute__((packed)) dir_entry_t;

// occupied in end of the actual flash memory
extern char __disk_start;
extern char __disk_end;
extern char __bg_image_start;
extern char __name_image_start;


#define FAT_VOLUME_SECTOR_SIZE  512
 // N32L40x Flash has 2K Page size
#define FAT_VOLUME_SECTORS_PER_CLUSTER  4
// virtual / fake; clusters >= 4086 and <= 65525 is FAT16
#define FAT16_MINIMUM_CLUSTERS (4086)

#define RESERVED_SECTORS 1 // boot sector

// FAT Directory
#define ROOT_DIR_ENTRIES (512)
#define DIR_ENTRY_SIZE (sizeof(dir_entry_t))
#define ROOT_DIR_SECTORS ((ROOT_DIR_ENTRIES * DIR_ENTRY_SIZE + FAT_VOLUME_SECTOR_SIZE - 1) / FAT_VOLUME_SECTOR_SIZE)

// FAT Entries
#define FAT_ENTRIES_PER_SECTOR (FAT_VOLUME_SECTOR_SIZE / sizeof(uint16_t)) // FAT16
// FAT_TABLE_SECTORS should be ((10000 / 4 + 512 / 2 - 1) / (512 / 2)) = 10
#define NUMBER_OF_FAT_TABLE 2
#define FAT_TABLE_SECTORS ((FAT16_MINIMUM_CLUSTERS * FAT_VOLUME_SECTORS_PER_CLUSTER + FAT_ENTRIES_PER_SECTOR - 1) / FAT_ENTRIES_PER_SECTOR)
#define FAT_TABLE_AREA_SECTORS (FAT_TABLE_SECTORS * NUMBER_OF_FAT_TABLE)

#define START_SECTOR_OF_FAT0 RESERVED_SECTORS
#define START_SECTOR_OF_FAT1 (START_SECTOR_OF_FAT0 + FAT_TABLE_SECTORS)
#define START_SECTOR_OF_ROOTDIR (START_SECTOR_OF_FAT1 + FAT_TABLE_SECTORS)
#define START_SECTOR_OF_DATAAREA (START_SECTOR_OF_ROOTDIR + ROOT_DIR_SECTORS)

#define FAT_VOLUME_SECTOR_COUNT (FAT16_MINIMUM_CLUSTERS * FAT_VOLUME_SECTORS_PER_CLUSTER + START_SECTOR_OF_DATAAREA)

#define FIRST_CLUSTER (0x0002)
#define EMPTY_CLUSTER (0x0000)
#define END_OF_CLUSTER (0xFFFF) // EOC Mark

#define VOLUME_LABEL "KiCon2023  "
#define USB_FLASH_START (uint32_t)(&__disk_start)
#define USB_FLASH_END (uint32_t)(&__disk_end)

#define STR0(x) #x
#define STR(x) STR0(x)

static const FAT_BootBlock BootBlock = {
    .JumpInstruction = {0xeb, 0x3c, 0x90},
    .OEMInfo = "KiCon2023 ",
    .SectorSize = FAT_VOLUME_SECTOR_SIZE,
    .SectorsPerCluster = FAT_VOLUME_SECTORS_PER_CLUSTER, // N32L40x Flash has 2K Page size
    .ReservedSectors = RESERVED_SECTORS,
    .FATCopies = 2,
    .RootDirectoryEntries = ROOT_DIR_ENTRIES,
    .TotalSectors16 = FAT_VOLUME_SECTOR_COUNT,
    .MediaDescriptor = 0xF8,
    .FATSz16 = FAT_TABLE_SECTORS,
    .SectorsPerTrack = 1,
    .Heads = 1,
    .ExtendedBootSig = 0x29,
    .VolumeLabel = VOLUME_LABEL,
    .VolumeSerialNumber = 0x00420042,
    .FilesystemIdentifier = "FAT16   ",
    .BootSign = 0xaa55,
};

#define TODAY (((2023 - 1980) << 9) | (11 << 5) | (07))
#define NOW ((20 << 11) | (56 << 5) | (0))

#define README_FILE_START_CLUSTER (FIRST_CLUSTER)
#define README_FILE_CLUSTERS (1) // max 512 bytes

#define BG_IMAGE_BIN_FILE_SIZE (72*288/2)
#define BG_IMAGE_BIN_FILE_SECTORS ((BG_IMAGE_BIN_FILE_SIZE + FAT_VOLUME_SECTOR_SIZE - 1) / FAT_VOLUME_SECTOR_SIZE)
#define BG_IMAGE_BIN_FILE_CLUSTERS ((BG_IMAGE_BIN_FILE_SECTORS + FAT_VOLUME_SECTORS_PER_CLUSTER - 1) / FAT_VOLUME_SECTORS_PER_CLUSTER)
#define BG_IMAGE_BIN_FILE_START_CLUSTER (README_FILE_START_CLUSTER + README_FILE_CLUSTERS) // First Cluster (2) is occupied by Read only Text File
#define BG_IMAGE_BIN_FILE_PHYSICAL_ADDR (uint32_t)(&__bg_image_start)

#define NAME_IMAGE_BIN_FILE_SIZE (72*144/2)
#define NAME_IMAGE_BIN_FILE_SECTORS ((NAME_IMAGE_BIN_FILE_SIZE + FAT_VOLUME_SECTOR_SIZE - 1) / FAT_VOLUME_SECTOR_SIZE)
#define NAME_IMAGE_BIN_FILE_CLUSTERS ((NAME_IMAGE_BIN_FILE_SECTORS + FAT_VOLUME_SECTORS_PER_CLUSTER - 1) / FAT_VOLUME_SECTORS_PER_CLUSTER)
#define NAME_IMAGE_BIN_FILE_START_CLUSTER (BG_IMAGE_BIN_FILE_START_CLUSTER + BG_IMAGE_BIN_FILE_CLUSTERS)
#define NAME_IMAGE_BIN_FILE_PHYSICAL_ADDR (uint32_t)(&__name_image_start)


static const dir_entry_t root_entry = {
    // Root Entry
    .name = VOLUME_LABEL,
    .attrs = 0x28, // VOLUME_ID | Directory
    .createTimeFine = 0,
    .createDate = TODAY,
    .createTime = NOW,
    .updateDate = TODAY,
    .updateTime = NOW,
    .highStartCluster = 0,
    .startCluster = 0,
};

static const char readme_file_contents[] = "Welcome to KiCon2023 Asia!";
typedef struct virtual_file_entry_t {
    char *name;
    uint8_t attrs;
    uint32_t size;
    uint32_t start_of_cluster;
    uint32_t clusters;
    uint32_t physical_addr;
} virtual_file_entry_t;

static const virtual_file_entry_t virtual_file_entries[] = {
  {
      .name = "README  TXT",
      .attrs = 0x01, // Read Only
      .size = sizeof(readme_file_contents) - 1,
      .start_of_cluster = README_FILE_START_CLUSTER,
      .clusters = README_FILE_CLUSTERS,
      .physical_addr = (uint32_t)readme_file_contents,
  },
  {
      .name = "BG      BIN",
      .attrs = 0x00,
      .size = BG_IMAGE_BIN_FILE_SIZE,
      .start_of_cluster = BG_IMAGE_BIN_FILE_START_CLUSTER,
      .clusters = BG_IMAGE_BIN_FILE_CLUSTERS,
      .physical_addr = BG_IMAGE_BIN_FILE_PHYSICAL_ADDR,
  },
  {
      .name = "NAME    BIN",
      .attrs = 0x00,
      .size = NAME_IMAGE_BIN_FILE_SIZE,
      .start_of_cluster = NAME_IMAGE_BIN_FILE_START_CLUSTER,
      .clusters = NAME_IMAGE_BIN_FILE_CLUSTERS,
      .physical_addr = NAME_IMAGE_BIN_FILE_PHYSICAL_ADDR,
  },
};

// manually crafted FAT
static const uint16_t first_fat[] = {0xFF00 | BootBlock.MediaDescriptor, 0xFFFF,
};

void usbd_msc_get_cap(uint8_t lun, uint32_t *block_num,
                        uint16_t *block_size) {
  *block_num = FAT_VOLUME_SECTOR_COUNT;
  *block_size = FAT_VOLUME_SECTOR_SIZE;
}

typedef enum usbd_transfer_dir_t {
  usbd_transfer_read,
  usbd_transfer_write,
} usbd_transfer_dir_t;

typedef enum usbd_transfer_inc_t {
  usbd_transfer_inc,
  usbd_transfer_no_inc,
} usbd_transfer_inc_t;

static void
usbd_transfer_op(const uint32_t memory_addr,
                 const uint32_t disk_addr,
                 const uint32_t length,
                 const usbd_transfer_dir_t dir,
                 const usbd_transfer_inc_t inc) {
  // at least 10 word long, then use DMA, otherwise, use CPU to copy
    uint32_t remaining_length = length;
    uint32_t disk_access_addr = disk_addr;
    uint32_t memory_access_addr = memory_addr;

    // DMA transferring is not always better than cpu copying
    // especially when the transerring size is small enough.
    // The size Here is merely a guess.
#define GUESSED_OPTIMUM_MIN_DMA_SIZE (sizeof(uint32_t) * 10)
    // need to unlock FLASH before write op
    if (remaining_length >= GUESSED_OPTIMUM_MIN_DMA_SIZE) {
        DMA_InitType dmaInit;
        DMA_StructInit(&dmaInit);

        const uint32_t word_length = remaining_length / 4;

        dmaInit.PeriphAddr = disk_access_addr;
        dmaInit.PeriphInc = (usbd_transfer_inc == inc) ? DMA_PERIPH_INC_ENABLE : DMA_PERIPH_INC_DISABLE;
        dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD;
        dmaInit.Direction = (dir == usbd_transfer_read) ? DMA_DIR_PERIPH_SRC : DMA_DIR_PERIPH_SRC;
        dmaInit.MemAddr = memory_access_addr;
        dmaInit.BufSize = word_length;
        dmaInit.Mem2Mem = DMA_M2M_ENABLE;
        dmaInit.MemDataSize = DMA_MemoryDataSize_Word;
        dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
        dmaInit.CircularMode = DMA_MODE_NORMAL;
        dmaInit.Priority = DMA_PRIORITY_HIGH;

        DMA_Init(DISK_DMA_CHANNEL, &dmaInit);
        DMA_ClearFlag(DISK_DMA_CHANNEL, DISK_DMA_FLG_TXC);
        DMA_EnableChannel(DISK_DMA_CHANNEL, ENABLE);

        int ret = usb_osal_sem_take(usb_dma_sem, 1); // 1 ms
        remaining_length -= word_length * 4;
        disk_access_addr += word_length * 4;
        memory_access_addr += word_length * 4;
    }

    assert_param(remaining_length < GUESSED_OPTIMUM_MIN_DMA_SIZE);
    if (remaining_length > 0) {
        if (usbd_transfer_read == dir) {
            memcpy((void *)memory_access_addr, (void *)disk_access_addr, remaining_length);
        } else { // write operation
            uint8_t data[4] = {0};
            for (uint32_t i = 0; i < remaining_length; i++) {
                data[i] = *((uint8_t *)memory_access_addr + i);
            }
            const uint32_t d = *((uint32_t *)data);
            FLASH_ProgramWord(disk_access_addr, d);
        }
    }
}

int usbd_msc_sector_read(uint32_t sector, uint8_t *buffer, uint32_t length)
{
    assert_param(length > 0);
    // virtual disk addr
    uint32_t disk_access_addr = 0;
    uint32_t buffer_access_addr = (uint32_t)buffer;
    // Boot Sector
    uint32_t sector_size_sum = 0;

    if (0 == sector) {
        assert_param(sizeof(BootBlock) <= FAT_VOLUME_SECTOR_SIZE);
        disk_access_addr = (uint32_t)&BootBlock;
        uint32_t transferring_len = MIN(sizeof(BootBlock), length);
        usbd_transfer_op(buffer_access_addr,
                         disk_access_addr,
                         transferring_len,
                         usbd_transfer_read,
                         usbd_transfer_inc);
        assert_param(length >= transferring_len);
        if (length > transferring_len) {
            length -= transferring_len;
        } else {
            return 0;
        }
        disk_access_addr += transferring_len;
        buffer_access_addr += transferring_len;

        // use zero to fill the remaining sector in the buffer
        // restrict the filling within a size of sector
        transferring_len = MIN(START_SECTOR_OF_FAT0
                               * FAT_VOLUME_SECTOR_SIZE
                               - disk_access_addr, length);
        const uint32_t zero = 0;
        usbd_transfer_op(buffer_access_addr,
                         (uint32_t)(&zero),
                         transferring_len,
                         usbd_transfer_read,
                         usbd_transfer_no_inc);

        assert_param(length >= transferring_len);
        if (length > transferring_len) {
            length -= transferring_len;
        } else {
            return 0;
        }
        disk_access_addr += transferring_len;
        buffer_access_addr += transferring_len;
        sector++;
    }

    const uint16_t fat_start_sectors[3] = {
        START_SECTOR_OF_FAT0,
        START_SECTOR_OF_FAT1,
        START_SECTOR_OF_ROOTDIR,        
    };
    
    for (int fat_idx = 0; fat_idx < 2; fat_idx ++) {
        // copy FAT entries
        if (fat_start_sectors[fat_idx] <= sector && sector < fat_start_sectors[fat_idx + 1]) {
            // crafted first sector of FAT entries
            if (fat_start_sectors[fat_idx] == sector) {
              uint32_t transferring_len = MIN(sizeof(first_fat), length);
              usbd_transfer_op(buffer_access_addr, (uint32_t)first_fat,
                               transferring_len, usbd_transfer_read,
                               usbd_transfer_inc);
              assert_param(length >= transferring_len);
              if (length > transferring_len) {
                  length -= transferring_len;
              } else {
                  return 0;
              }
              disk_access_addr += transferring_len;
              buffer_access_addr += transferring_len;

              sector_size_sum += transferring_len;
              if (sector_size_sum >= FAT_VOLUME_SECTOR_SIZE) {
                  sector += sector_size_sum / FAT_VOLUME_SECTOR_SIZE;
                  sector_size_sum = sector_size_sum % FAT_VOLUME_SECTOR_SIZE;
              }
            }

            uint16_t cluster_id = FIRST_CLUSTER;
            for (uint32_t entry_idx = 0;
                 entry_idx < sizeof(virtual_file_entries) / sizeof(virtual_file_entries[0]);
                 entry_idx++) {
                virtual_file_entry_t ventry = virtual_file_entries[entry_idx];
                for (uint32_t en_cls_idx = 0; en_cls_idx < ventry.clusters - 1; en_cls_idx++) {
                    cluster_id++;
                    const uint32_t transferring_len = MIN(sizeof(cluster_id), length);
                    usbd_transfer_op(buffer_access_addr, (uint32_t)&cluster_id,
                                     transferring_len, usbd_transfer_read,
                                     usbd_transfer_inc);
                    assert_param(length >= transferring_len);
                    if (length > transferring_len) {
                        length -= transferring_len;
                    } else {
                        return 0;
                    }
                    disk_access_addr += transferring_len;
                    buffer_access_addr += transferring_len;
                    sector_size_sum += transferring_len;
                    if (sector_size_sum >= FAT_VOLUME_SECTOR_SIZE) {
                      sector += sector_size_sum / FAT_VOLUME_SECTOR_SIZE;
                      sector_size_sum =
                          sector_size_sum % FAT_VOLUME_SECTOR_SIZE;
                    }
                }
                cluster_id++;
                const uint16_t eoc = END_OF_CLUSTER;
                uint32_t transferring_len = MIN(sizeof(eoc), length);
                usbd_transfer_op(buffer_access_addr, (uint32_t)&eoc,
                                 transferring_len, usbd_transfer_read,
                                 usbd_transfer_inc);
                assert_param(length >= transferring_len);
                if (length > transferring_len) {
                    length -= transferring_len;
                } else {
                    return 0;
                }
                disk_access_addr += transferring_len;
                buffer_access_addr += transferring_len;
                sector_size_sum += transferring_len;
                if (sector_size_sum >= FAT_VOLUME_SECTOR_SIZE) {
                    sector += sector_size_sum / FAT_VOLUME_SECTOR_SIZE;
                    sector_size_sum = sector_size_sum % FAT_VOLUME_SECTOR_SIZE;
                }

            } // for: construct FAT for virtual files
            // use zero to fill the remaining sector in the buffer
            // restrict the filling within a size of sector
            uint32_t transferring_len =
              MIN((FAT_TABLE_SECTORS - (sector - fat_start_sectors[fat_idx])) * FAT_VOLUME_SECTOR_SIZE - sector_size_sum,
                  length);
            const uint32_t zero = 0;
            usbd_transfer_op(buffer_access_addr,
                             (uint32_t)(&zero),
                             transferring_len, usbd_transfer_read,
                             usbd_transfer_no_inc);

            assert_param(length >= transferring_len);
            if (length > transferring_len) {
              length -= transferring_len;
            } else {
              return 0;
            }
            disk_access_addr += transferring_len;
            buffer_access_addr += transferring_len;
            sector ++;
            sector_size_sum = 0;
        } // if check start of sector
    } // for

    // Directory Entries
    if (START_SECTOR_OF_ROOTDIR <= sector
        && sector < START_SECTOR_OF_DATAAREA) {
        uint32_t dir_entry_idx = (sector - START_SECTOR_OF_ROOTDIR)
            * FAT_VOLUME_SECTOR_SIZE
            / DIR_ENTRY_SIZE;

        if (0 == dir_entry_idx) {
            const uint32_t transferring_len = MIN(sizeof(root_entry), length);
            usbd_transfer_op(buffer_access_addr,
                             (uint32_t)&root_entry,
                             transferring_len,
                             usbd_transfer_read,
                             usbd_transfer_inc);
            assert_param(length >= transferring_len);
            if (length > transferring_len) {
                length -= transferring_len;
            } else {
                return 0;
            }
            disk_access_addr += transferring_len;
            buffer_access_addr += transferring_len;
            sector_size_sum += transferring_len;
            if (sector_size_sum >= FAT_VOLUME_SECTOR_SIZE) {
                sector += sector_size_sum / FAT_VOLUME_SECTOR_SIZE;
                sector_size_sum = sector_size_sum % FAT_VOLUME_SECTOR_SIZE;
            }
            dir_entry_idx ++;
        }
        static const max_dir_entries = MIN(ROOT_DIR_ENTRIES, sizeof(virtual_file_entries) / sizeof(virtual_file_entries[0]));
        while (length > 0 && dir_entry_idx <= max_dir_entries) {
            virtual_file_entry_t vfile = virtual_file_entries[dir_entry_idx - 1];
            dir_entry_t dir_entry = {
                .attrs = vfile.attrs,
                .highStartCluster = 0,
                .startCluster = vfile.start_of_cluster,
                .createTime = NOW,
                .createDate = TODAY,
                .updateTime = NOW,
                .updateDate = TODAY,
                .lastAccessDate = TODAY,
                .size = vfile.size,
            };
            memcpy(dir_entry.name, vfile.name, sizeof(dir_entry.name));
            const uint32_t transferring_len = MIN(sizeof(dir_entry), length);
            usbd_transfer_op(buffer_access_addr,
                             (uint32_t)&dir_entry,
                             transferring_len,
                             usbd_transfer_read,
                             usbd_transfer_inc);
            assert_param(length >= transferring_len);
            if (length > transferring_len) {
                length -= transferring_len;
            } else {
                return 0;
            }
            disk_access_addr += transferring_len;
            buffer_access_addr += transferring_len;
            sector_size_sum += transferring_len;
            if (sector_size_sum >= FAT_VOLUME_SECTOR_SIZE) {
                sector += sector_size_sum / FAT_VOLUME_SECTOR_SIZE;
                sector_size_sum = sector_size_sum % FAT_VOLUME_SECTOR_SIZE;
            }
            dir_entry_idx ++;
        }
        if (sector < START_SECTOR_OF_DATAAREA) {
            // use zero to fill the remaining sector in the buffer
            // restrict the filling within a size of sector
            const uint32_t transferring_len =
              MIN((START_SECTOR_OF_DATAAREA - sector) * FAT_VOLUME_SECTOR_SIZE,
                  length);
            const uint32_t zero = 0;
            usbd_transfer_op(buffer_access_addr, (uint32_t)(&zero),
                             transferring_len, usbd_transfer_read,
                             usbd_transfer_no_inc);

            assert_param(length >= transferring_len);
            if (length > transferring_len) {
                length -= transferring_len;
            } else {
                return 0;
            }
            disk_access_addr += transferring_len;
            buffer_access_addr += transferring_len;
            sector_size_sum += transferring_len;
            if (sector_size_sum >= FAT_VOLUME_SECTOR_SIZE) {
                sector += sector_size_sum / FAT_VOLUME_SECTOR_SIZE;
                sector_size_sum = sector_size_sum % FAT_VOLUME_SECTOR_SIZE;
            }
        }
    }

    // Data Area
    if (START_SECTOR_OF_DATAAREA <= sector) {
        sector -= START_SECTOR_OF_DATAAREA;

        for (uint32_t entry_idx = 0;
             entry_idx <
                 sizeof(virtual_file_entries) / sizeof(virtual_file_entries[0]);
             entry_idx++) {
            virtual_file_entry_t ventry = virtual_file_entries[entry_idx];
            const uint32_t ventry_offset_sector = (ventry.start_of_cluster - FIRST_CLUSTER) * FAT_VOLUME_SECTORS_PER_CLUSTER;
            if (ventry_offset_sector <= sector
                && sector < ventry_offset_sector + ventry.clusters * FAT_VOLUME_SECTORS_PER_CLUSTER) {
              const uint32_t transferring_len =
                MIN(ventry.size - (sector - ventry_offset_sector) * FAT_VOLUME_SECTOR_SIZE,
                    length);
                usbd_transfer_op(buffer_access_addr,
                                 (uint32_t)ventry.physical_addr + (sector - ventry_offset_sector) * FAT_VOLUME_SECTOR_SIZE,
                                 transferring_len,
                                 usbd_transfer_read,
                                 usbd_transfer_inc);
                assert_param(length >= transferring_len);
                if (length > transferring_len) {
                    length -= transferring_len;
                } else {
                    return 0;
                }
                disk_access_addr += transferring_len;
                buffer_access_addr += transferring_len;
                sector_size_sum += transferring_len;
                if (sector_size_sum >= FAT_VOLUME_SECTOR_SIZE) {
                    sector += sector_size_sum / FAT_VOLUME_SECTOR_SIZE;
                    sector_size_sum = sector_size_sum % FAT_VOLUME_SECTOR_SIZE;
                }
                // finish transferring actual data

                // just leave grabage data here
                if (sector_size_sum > 0) {
                    const uint32_t transferring_len =
                        MIN(FAT_VOLUME_SECTOR_SIZE - sector_size_sum,
                            length);
                    assert_param(length >= transferring_len);
                    if (length > transferring_len) {
                        length -= transferring_len;
                    } else {
                        return 0;
                    }
                    disk_access_addr += transferring_len;
                    buffer_access_addr += transferring_len;
                    sector ++;
                    sector_size_sum = 0;
                }
            } // for loop virtual file entries
        } // if sector is inside a virtual entry
    } // if START_SECTOR_OF_DATAAREA
    return 0;
}

void DISK_DMA_IRQHandler(void) {
  if (DMA_GetIntStatus(DISK_DMA_INT_TXC, DMA)) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    usb_osal_sem_give(usb_dma_sem);
  }
  
  DMA_ClrIntPendingBit(DISK_DMA_INT_GLB
                       |DISK_DMA_INT_TXC
                       |DISK_DMA_INT_HTX
                       |DISK_DMA_INT_ERR,
                       DMA);
}

int usbd_msc_sector_write(uint32_t sector, uint8_t *buffer, uint32_t length)
{
    uint8_t * start_addr =(uint8_t *) __disk_start + sector * FAT_VOLUME_SECTOR_SIZE;
    
  if (sector * FAT_VOLUME_SECTOR_SIZE + length < &__disk_start) {
#ifdef CONFIG_USBDEV_MSC_THREAD
    uint32_t len = length;
    uint32_t transferred_len = 0;

    if (len > 10 * 4) {
      DMA_InitType dmaInit;
      DMA_StructInit(&dmaInit);

      const uint32_t dma_len = len / 4;
      dmaInit.PeriphAddr = (uint32_t)start_addr;
      dmaInit.PeriphInc = DMA_PERIPH_INC_ENABLE;
      dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_WORD;
      dmaInit.Direction = DMA_DIR_PERIPH_DST;
      dmaInit.MemAddr = (uint32_t)buffer;
      dmaInit.BufSize = dma_len;
      dmaInit.Mem2Mem = DMA_M2M_ENABLE;
      dmaInit.MemDataSize = DMA_MemoryDataSize_Word;
      dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
      dmaInit.CircularMode = DMA_MODE_NORMAL;
      dmaInit.Priority = DMA_PRIORITY_HIGH;

      DMA_Init(DISK_DMA_CHANNEL, &dmaInit);
      DMA_ClearFlag( DISK_DMA_FLG_TXC, DMA);
      DMA_EnableChannel(DISK_DMA_CHANNEL, ENABLE);

      transferred_len = dma_len * 4;
      len -= transferred_len;
      start_addr += transferred_len;
      int ret = usb_osal_sem_take(usb_dma_sem, 1); // 1 ms
      if (!ret) {                                  // timeout
        return -1;
      }
    }
    memcpy(start_addr, buffer + transferred_len, len);
#else
    memcpy(start_addr, buffer, length);
#endif
  }
  return 0;
}
