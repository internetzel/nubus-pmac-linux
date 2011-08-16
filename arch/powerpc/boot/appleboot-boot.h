/*
 * arch/ppc/appleboot/boot.h
 */

#ifndef __APPLE_BOOT_BOOT_H
#define __APPLE_BOOT_BOOT_H

#ifndef __ASSEMBLY__

/* Apple MkLinux Booter Infos */

struct region_desc {
	unsigned long addr;
	unsigned long offset;
	unsigned long size;
	unsigned long prot;
	unsigned long mapped;
};

#define MAX_REGIONS 8

struct prog {
	struct region_desc regions[MAX_REGIONS];
	int region_count;
	unsigned long entry;
	unsigned long base_addr;
	unsigned long args_start;
	unsigned long args_size;
};

struct boot_video {
	unsigned long v_base;
	unsigned long unused;
	unsigned long v_rowbytes;
	unsigned long v_width;
	unsigned long v_height;
	unsigned long v_depth;
};

#define MAX_DRAM_BANKS 26

struct dram_bank {
	unsigned long base;
	unsigned long size;
};

#define BOOT_LINE_LENGTH 256

struct boot_args {
	unsigned long version;
	unsigned long size;
	struct prog kern_info;
	struct prog task_info;
	char command_line[BOOT_LINE_LENGTH];
	struct dram_bank physical_dram[MAX_DRAM_BANKS];
	unsigned long first_avail;
	struct boot_video video;
	unsigned long machine_type;
	long device_tree_size;
	unsigned long device_tree_buffer;
};

/* BootX Boot Infos */
#define BOOT_ARCH_PCI                   0x00000001UL
#define BOOT_ARCH_NUBUS                 0x00000002UL
#define BOOT_ARCH_NUBUS_CLASS		0x00000070UL
#define BOOT_ARCH_NUBUS_PDM             0x00000010UL
#define BOOT_ARCH_NUBUS_PERFORMA        0x00000020UL
#define BOOT_ARCH_NUBUS_POWERBOOK       0x00000040UL

struct boot_infos {
	unsigned long version;
	unsigned long compatible_version;
	unsigned char *logicalDisplayBase;
	unsigned long machine_id;
	unsigned long architecture;
	unsigned long device_tree_offset;
	unsigned long device_tree_size;
	unsigned long dispDeviceRect[4];
	unsigned long dispDeviceDepth;
	unsigned char *dispDeviceBase;
	unsigned long dispDeviceRowBytes;
	unsigned long dispDeviceColorsOffset;
	unsigned long dispDeviceRegEntryOffset;
	unsigned long ram_disk;
	unsigned long ram_disk_size;
	unsigned long kernel_params_offset;
	struct dram_bank mem_map[MAX_DRAM_BANKS];
	unsigned long mem_map_size;
	unsigned long frame_buffer_size;
	unsigned long total_params_size;
};

#endif /* !__ASSEMBLY__ */

#define BOOTX_SIGS		0x426f6f58
#define APPLE_BOOTER_SIGS	0x4d6b4c42 /* 'MkLB' */
#define KERNEL_STACK_SIZE	65536
#define LOAD_OFFSET		0x00000000 /* phys addr of kernel load base */
#define TEXT_BASE		0x00200000
#define DATA_BASE		0x00400000
#define BOOT_STACK		0x001f0000
#define PDM_VIDEO_RAM		0x00100000
#define PDM_VIDEO_SIZE		0x00096000
#define MAX_ZIMAGE_SIZE		0x001d0000
#define KERNELBASE		0xc0000000

/* for debugging */

#define LOADER_PRINT

#endif /* __APPLE_BOOT_BOOT_H */
