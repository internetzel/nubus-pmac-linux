/*
 *
 *  In this implementation, NuBus PowerMacs are to act as if they were PCI
 *  PowerMacs.  Here, "fake" OF device tree is created based on classes/models
 *  of NuBus PowerMacs.
 *
 *  Copyright (C) 2000 Takashi Oe (toe@users.sourceforge.net)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */

/*
 *
 *  Supported command line option:
 *
 *	"nubus_simm="
 *
 *	To specify that bank 3 has 8 MB SIMM and bank 6 has 4 MB SIM,
 *	for example, try "nubus_simm=b3:8m,b6:4m"
 *
 *	bank #	Phys Address
 *	------+--------------
 *	   0  | 0x00000000
 *	   1  | 0x01000000
 *	   2  | 0x05000000
 *	   3  | 0x09000000
 *	   4  | 0x0d000000
 *	   5  | 0x11000000
 *	   6  | 0x15000000
 *	   7  | 0x19000000
 *	   8  | 0x1d000000
 *	   9  | 0x10000000	XXX
 *
 */

#include <stdarg.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/version.h>
//#include <linux/blk.h>

#include <asm/machdep.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/bootx.h>
#include <asm/ptrace.h>

#include "nbpmac.h"
#include "nbpmac_id.h"
#if 0
#include "nbpmac_node.h"

#include "../mm/mem_pieces.h"

#define ALIGN_NBPMAC(x) (((unsigned long)(x) + 0x7) & ~0x7)

/* TODO ?: kdev_t boot_dev; */
int is_PM6100;
unsigned char *amic_dma_buf_base;

extern boot_infos_t *boot_infos;
extern char *klimit;
extern unsigned long dev_tree_size;
extern struct device_node *allnodes;
extern struct reg_property *ioremap_ok_list;
extern int pmac_newworld;

static int is_pb5300;

extern struct device_node *memory_node;

#ifdef CONFIG_DISCONTIGMEM
extern char _stext[];
extern char *klimit;
extern unsigned long __max_memory;
extern unsigned long real_memory;
extern struct mem_pieces phys_mem;
extern struct mem_pieces phys_avail;
#endif

/*
 * External functions
 */
long nbpmac_time_init(void);
void nbpmac_init(unsigned long, unsigned long, unsigned long,
		     unsigned long, unsigned long);
void fake_nodes(void);
void amic_dma_init(void);
extern long pmac_time_init(void);

/*
 * Internal functions
 */
static void setup_display_prop(void);
static void mcpy(void *, const void *, int);
static void mset(void *, int, int);
#endif

//int ppc_nubus_init(void);
int nbpmac_slot2irq(int);
int nbpmac_irq2slot(int);

/*
 * Nubus Support
 */

int
nbpmac_slot2irq(int slot)
{
	int irq = -1;

	if (of_machine_is_compatible("PDM")) {
		if (slot >= 11 && slot <= 14)
			irq = slot + 15;
	} else if (of_machine_is_compatible("Performa")) {
		if (slot == 14)
			irq = 29;
	} else if (of_machine_is_compatible("M2")) {
		if (slot == 13)
			irq = 29;
	}

	if (irq < 0)
		printk(KERN_WARNING "slot2irq: unhandled slot (%d)\n", slot);

	return irq;
}

int
nbpmac_irq2slot(int irq)
{
	int slot = -1;

	if (of_machine_is_compatible("PDM")) {
		if (irq >= 26 && irq <= 29)
			slot = irq - 15;
	} else if (of_machine_is_compatible("Performa")) {
		if (irq == 29)
			slot = 14;
	} else if (of_machine_is_compatible("M2")) {
		if (irq == 29)
			slot = 13;
	}

	if (slot < 0)
		panic("irq2slot: unknown irq (%d)\n", irq);

	return slot;
}

#if 0
int __init
ppc_nubus_init(void)
{
	return 0;
}
#endif

#include <asm/setjmp.h>
#include <asm/synch.h>
#include <asm/delay.h>

static long bus_error_jmp[JMP_BUF_LEN];
//extern void (*nubus_fault_handler)(struct pt_regs *regs);

static int fault_type;
static int fault_except;

static int handle_fault(struct pt_regs *regs)
{
        fault_except = TRAP(regs);
        switch (TRAP(regs)) {
        case 0x200:  
                fault_type = 0;
                break;  
        case 0x300: 
        case 0x380:
                fault_type = 1;
                break;
        default:
                fault_type = 2;
        }

        longjmp(bus_error_jmp, 1);	

	return 0;
}

static inline void sync(void)
{
	mb();
//	asm volatile("isync");
	isync();
}

int
hwreg_present(void __iomem *regp)
{
	unsigned char x;
	unsigned long flags;
	volatile int n;

        if (!(of_machine_is_compatible("PDM") ||
	      of_machine_is_compatible("Performa") ||
	      of_machine_is_compatible("M2")))
		return 0;

	//if (ppc_md.progress) ppc_md.progress( "hwreg_present(): enter\n", 0 );

	n = 0;
	fault_type = -1;
	/* TODO ?: save_flags(flags); cli(); */
	local_irq_save( flags );
	if(setjmp(bus_error_jmp) == 0) {
		ppc_md.machine_check_exception = handle_fault;
//		nubus_fault_handler = handle_fault;
		sync();
		x = in_8(regp);
		sync();
		/* wait a little while to see if we get a machine check */
		__delay(2000);
		n = 1;
	}
	ppc_md.machine_check_exception = NULL;
//	nubus_fault_handler = 0;
	/* TODO ?: restore_flags(flags); */
	local_irq_restore( flags );
	return n;
}

int __init nbpmac_pic_probe(void)
{
	if(of_machine_is_compatible("M2")) {
		/* disable keystone interrupts */
		nbpbook_exp_init();
		/* setup interrupt controller */
		pmac_m2_init();
	}
#if 0
	else if(of_machine_is_compatible("Performa")) {
		/* disable valkyrie interrupts */
		pfm_vlk_init();
		/* setup interrupt controller */
		pmac_pfm_init();
	}
#endif
	return 0;
}

#if 0
/*
 * Nubus Power Macs Setup
 */

long __init
nbpmac_time_init(void)
{
#if 0 /* TODO: FB, cuda_init_via() called during pmac_setup_arch() already */
#ifdef CONFIG_ADB_CUDA
	extern int cuda_init_via(void);
#endif /* CONFIG_ADB_CUDA */

#ifdef CONFIG_ADB_CUDA
	if (boot_infos &&
	    !(boot_infos->architecture & BOOT_ARCH_NUBUS_POWERBOOK) &&
	    cuda_init_via() < 0) {
		printk(KERN_ERR "CUDA init has failed\n");
#ifdef CONFIG_BOOTX_TEXT
		prom_print("CUDA init has failed\n");
#endif /* CONFIG_BOOTX_TEXT */
	}
#endif /* CONFIG_ADB_CUDA */
#endif /* disabled cuda init here */
	return pmac_time_init();
}


#define GLUE_TWO_NODES(N1,N2)		\
do {					\
	struct device_node *dp, *dn;	\
	dp = find_devices(N1);		\
	dn = find_devices(N2);		\
	if (dp && dn)			\
		dp->next = dn;		\
} while (0)


#define FIX_PREVIOUS_NODE(START, NEXT, OLD, NEW)	\
do {							\
	struct device_node **dt;			\
	for (dt = &START; *dt; dt = &(*dt)->NEXT) {	\
		if (*dt == OLD)				\
			break;				\
	}						\
	if (*dt)					\
		*dt = NEW;				\
} while (0)

#if 0
void __init
pmac_pdm_init(void)
{
	int i, j;
	struct device_node *dd;
	struct reg_property *rp;
	struct reg_property mem_bank[10] = {
		{0,0},
		{0x01000000,0}, {0x05000000,0}, {0x09000000,0}, {0x0d000000,0},
		{0x11000000,0}, {0x15000000,0}, {0x19000000,0}, {0x1d000000,0},
		{0x10000000,0}
	};
	char *p, *q;

	switch (boot_infos->machineID) {
	case MACHID_POWERMAC6100_60:
	case MACHID_POWERMAC6100_66:
	case MACHID_POWERMAC6100_80:
		is_PM6100 = 1;
		break;
	default:
		break;
	}

	/* Add PDM VRAM area in main memory to ioremap ok list */
	ioremap_ok_list = (struct reg_property *) ALIGN_NBPMAC(klimit);
	ioremap_ok_list->address = 0x100000;
	ioremap_ok_list->size = 0x96000;
	klimit = (char *) (ioremap_ok_list + 1);

	/* get memory bank infos from BootX/miBoot */
	for (i = 0; i < boot_infos->physMemoryMapSize; i++) {
		if (!boot_infos->physMemoryMap[i].size)
			continue;
		for (j = 0; j < 10; j++) {
			if (boot_infos->physMemoryMap[i].physAddr
			    == mem_bank[j].address)
				mem_bank[j].size =
					boot_infos->physMemoryMap[i].size;
		}
	}

	/* fix up the infos according to command line */
	p = q = 0;
	if ((p = strstr(cmd_line, "nubus_simm=")))
		q = p + 12;
	else if ((p = strstr(cmd_line, "nubus_sim=")))
		q = p + 11;
	if (p && q) {
		unsigned long mem = 0, bank = 0;

		for (; q[-1] == 'b'; ) {
			bank = simple_strtoul(q, &q, 0);
			if (bank > 9)
				break;
			++q;
			if (q[-1] != ':')
				break;
			mem = simple_strtoul(q, &q, 0);
			if (*q == 'k' || *q == 'K') {
				mem <<= 10;
				++q;
			} else if (*q == 'm' || *q == 'M') {
				mem <<= 20;
				++q;
			}
			/* no sanity checking, it'd better be right. */
			mem_bank[bank].size = mem;
			if (*q != ',')
				break;
			q += 2;
		}
	}

	/* do the memory property fix up */
	dd = find_devices("memory");
	/* "reg" is the only property in properties */
	rp = (struct reg_property *)dd->properties[0].value;
	/* update device node */
	for (i = 0; i < 10; ++i) {
		rp[i].size = mem_bank[i].size;
		dd->addrs[i].size = mem_bank[i].size;
	}

	/* fix up serial property */
	GLUE_TWO_NODES("ch-a", "ch-b");

	/* fix up scsi property */
	GLUE_TWO_NODES("amic-53c94", "53cf94");
}
#endif

void __init
pmac_pbook_init(void)
{
	int i, irq;
	unsigned long size = 0;
	struct device_node *dd;
	struct reg_property *rp;
	char *conn;

	/* get memory bank infos from BootX/miBoot */
	dd = find_devices("memory");
	rp = (struct reg_property *)dd->properties[0].value;

	/* get memory bank infos from BootX/miBoot */
	for (i = 0; i < boot_infos->physMemoryMapSize; i++) {
		if (!boot_infos->physMemoryMap[i].size)
			continue;
		if (boot_infos->physMemoryMap[i].physAddr == 0)
			size = boot_infos->physMemoryMap[i].size;
	}

	rp[0].size = size;
	dd->addrs[0].size = size;

	/*
	 * Remove '5380' and 'trex' node if it's not 1400 or 5300.  This
	 * should be replaced by hardware probing code for DuoDock and such.
	 * Also, fix up ATA0 irq line for Duo 2300.
	 */
	is_pb5300 = 0;
	switch (boot_infos->machineID) {
	case MACHID_POWERBOOK5300:
		is_pb5300 = 1;
	case MACHID_POWERBOOK1400:
		break;
	case MACHID_POWERBOOKDUO2300:
		/*
		 * Duo 2300's ATA0 irq line is connected to where TREX is on
		 * other PowerBooks.
		 */
		dd = find_devices("trex");
		if (dd) {
			irq = dd->intrs[0].line;
			dd = find_devices("baboon");
			if (dd)
				dd->intrs[0].line = irq;
		}

		/*
		 * Duo 2300 has an internal modem in place of infrared module
		 * found in other NuBus PPC PowerBooks. (conjecture)
		 */
		dd= find_devices("ch-b");
		if (dd) {
			conn = get_property(dd, "AAPL,connector", &i);
			if (conn)
				strncpy(conn, "modem   ", 8);
		}
	default:
		dd = find_devices("5380");
		if (dd) {
			FIX_PREVIOUS_NODE(dd->parent->child, sibling, dd,
					  dd->sibling);
			FIX_PREVIOUS_NODE(allnodes, allnext, dd, dd->allnext);
		}
		dd = find_devices("trex");
		if (dd) {
			FIX_PREVIOUS_NODE(dd->parent->child, sibling, dd,
					  dd->sibling);
			FIX_PREVIOUS_NODE(allnodes, allnext, dd, dd->allnext);
		}
		break;
	}

	/* fix up serial property */
	GLUE_TWO_NODES("ch-a", "ch-b");

	/* fix up ATA property */
	GLUE_TWO_NODES("baboon", "baboon2");
}


void __init
pmac_performa_init(void)
{
	int i;
	unsigned long size = 0;
	struct device_node *dd;
	struct reg_property *rp;

	/* get memory bank infos from BootX/miBoot */
	dd = find_devices("memory");
	rp = (struct reg_property *)dd->properties[0].value;

	/* get memory bank infos from BootX/miBoot */
	for (i = 0; i < boot_infos->physMemoryMapSize; i++) {
		if (!boot_infos->physMemoryMap[i].size)
			continue;
		if (boot_infos->physMemoryMap[i].physAddr == 0)
			size = boot_infos->physMemoryMap[i].size;
	}

	rp[0].size = size;
	dd->addrs[0].size = size;

	/* fix up serial property */
	GLUE_TWO_NODES("ch-a", "ch-b");
}


void __init
nbpmac_init(unsigned long r3, unsigned long r4, unsigned long r5,
		unsigned long r6, unsigned long r7)
{
	if (!boot_infos)
		return;

	is_PM6100 = 0;
	if (boot_infos->architecture & BOOT_ARCH_NUBUS_POWERBOOK)
		pmac_pbook_init();

	else if (boot_infos->architecture & BOOT_ARCH_NUBUS_PERFORMA)
		pmac_performa_init();
/* TODO: Only PowerBook and Performa for now
	else
		pmac_pdm_init();
*/
	/* add a display property for non-ecsc video */
	setup_display_prop();
}


struct video_chip {
	char name[16];
	struct reg_property reg[4];
	struct interrupt_info irq;
	unsigned long fb_addr;
	unsigned long fb_mask;
};


static struct video_chip __attribute((aligned(16)))
nbpmac_video[] __initdata = {
	{"ariel2", {{0x50f24000,0x10},{0x50f28000,0x10},{0x100000,0x96000},},
		{30,0}, 0x00100000UL, 0xfff00000UL},
	{"av", {{0xe0036000, 0x1000},{0xe0f20000,0x11000},},{0,},
		0xe0100000UL, 0xfff00000UL},
	{"hpv",{{0,},},{0,}, 0xfe000000UL, 0xff000000UL},
	{"valkyrie",{{0,},},{0,}, 0xf9000000UL, 0xff000000UL},
	{"ecsc",{{0,},},{0,}, 0x60000000UL, 0xf0000000UL},
	{"radius",{{0,},},{0,}, 0xfb000000UL, 0xff000000UL}
};


/* XXX fix this for powerbook and performa classes */
static void __init
setup_display_prop(void)
{
	int i, num;
	unsigned long video, size;
	struct device_node *dp, *dn;
	struct video_chip *chips;

	dn = find_devices("ariel2");
	if (!dn)
		return;
	video = (unsigned long)boot_infos->dispDeviceBase;
	if (video <= 0x100000UL)
		return;
	size = boot_infos->dispDeviceRowBytes * boot_infos->dispDeviceRect[3];
	num = sizeof(nbpmac_video)/sizeof(*nbpmac_video);
	for (chips = nbpmac_video, i = 0; i < num; ++i, ++chips) {
		if (chips->fb_addr == (video & chips->fb_mask))
			break;
	}
	dp = (struct device_node *) ALIGN_NBPMAC(klimit);
	mset(dp, 0, sizeof(*dp));
	dp->name = (char *) (dp + 1);
	dp->full_name = (char *) (dp->name + 16);
	strcpy(dp->full_name, "/nubus/");
	dp->type = (char *) (dp->full_name + 32);
	strcpy(dp->type, "display");
	dp->addrs = (struct address_range *) (dp->type + 16);
	mset(dp->addrs, 0, sizeof(*(dp->addrs)));
	dp->addrs[0].address = video;
	dp->addrs[0].size = size;
	dp->n_addrs = 1;
	dp->intrs = (struct interrupt_info *) (dp->addrs + 1);
	if (i < num) {
		strcpy(dp->name, chips->name);
		strcat(dp->full_name, chips->name);
		for (i = 0; chips->reg[i].address; ++i) {
			mset(dp->addrs + 1 + i, 0, sizeof(*(dp->addrs)));
			dp->addrs[1 + i].address = chips->reg[i].address;
			dp->addrs[1 + i].size = chips->reg[i].size;
		}
		dp->n_addrs += i;
		dp->intrs = (struct interrupt_info *) (dp->addrs + 1 + i);
		if (chips->irq.line) {
			dp->intrs->line = chips->irq.line;
			dp->intrs->sense = 0;
			dp->n_intrs = 1;
		}
	} else {
		strcpy(dp->name, "unknown");
		strcat(dp->full_name, "unknown");
	}

	dp->parent = dn->parent;
	/* remove Ariel II node */
	FIX_PREVIOUS_NODE(dn->parent->child, sibling, dn, dp);
	FIX_PREVIOUS_NODE(allnodes, allnext, dn, dp);
	if (!(boot_infos->architecture & BOOT_ARCH_NUBUS_PDM)) {
		dp->sibling = dn->sibling;
		dp->allnext = dn->allnext;
	} else {
		/* add back Ariel II */
		dp->sibling = dn;
		dp->next = dn;
		dp->allnext = dn;
	}

	klimit = (char *) (dp->intrs + dp->n_intrs);
}

void __init
fake_nodes(void)
{
	int i;
	struct device_node *dp, *pre, *parent, **prevp;
	struct property *pr, *pp;
	unsigned long mem = (unsigned long) ALIGN_NBPMAC(klimit);

	/*
	 * pmac_newworld is used in get_cpuinfo
	 */
	pmac_newworld = -1;

	parent = pre = 0;
	allnodes = (struct device_node *) mem;
	prevp = &allnodes;
	if (boot_infos->architecture & BOOT_ARCH_NUBUS_POWERBOOK)
		pr = pbook_props;
	else if (boot_infos->architecture & BOOT_ARCH_NUBUS_PERFORMA)
		pr = pfm_props;
	else
		pr = pdm_props;
	for (; pr->name;) {
		dp = (struct device_node *) ALIGN_NBPMAC(mem);
		if (!strncmp(pr->name, "child", 5)) {
			parent = pre;
			pre = 0;
			if (parent) {
				parent->child = dp;
				parent->sibling = 0;
			}
			pr++;
			continue;
		} else if (!strncmp(pr->name, "parent", 6)) {
			pre->sibling = 0;
			pre = pre->parent;
			if (pre)
				parent = pre->parent;
			else
				parent = 0;
			pr++;
			continue;
		} else if (pre)
			pre->sibling = dp;
		pre = dp;
		mset(dp, 0, sizeof(*dp));
		mem = (unsigned long)(dp + 1);
		dp->parent = parent;
		dp->name = (char *) mem;
		strncpy(dp->name, pr->value, pr->length + 1);
		mem += pr->length + 1;
		*((char*)mem-1) = 0;
		pr++;
		pp = 0;
		while (pr->name && strncmp(pr->name, "name", 4)
			      && strncmp(pr->name, "child", 5)
			      && strncmp(pr->name, "parent", 6)) {
			if (!strncmp(pr->name, "type", 4)) {
				dp->type = (char *) mem;
				strncpy(dp->type, pr->value, pr->length + 1);
				mem += pr->length + 1;
				*((char*)mem-1) = 0;
			} else if (!strncmp(pr->name, "full_name", 9)) {
				dp->full_name = (char *) mem;
				strncpy(dp->full_name, pr->value, pr->length+1);
				mem += pr->length + 1;
				*((char*)mem-1) = 0;
			} else if (!strncmp(pr->name, "intrs", 5)) {
				dp->n_intrs = pr->length
					      / sizeof(struct interrupt_info);
				dp->intrs = (struct interrupt_info *)ALIGN_NBPMAC(mem);
				i = dp->n_intrs * sizeof(struct interrupt_info);
				mem = (unsigned long)(dp->intrs + dp->n_intrs);
				mcpy((void *)dp->intrs, (void *)pr->value, i);
			} else {
				if (!pp) {
					dp->properties =
						(struct property *)ALIGN_NBPMAC(mem);
					pp = dp->properties;
				} else {
					pp->next =
						(struct property *)ALIGN_NBPMAC(mem);
					pp = pp->next;
				}
				pp->next = 0;
				mem = (unsigned long)(pp + 1);
				pp->name = (char *) mem;
				i = strlen(pr->name) + 1;
				strncpy(pp->name, pr->name, i);
				mem += i;
				*((char*)mem-1) = 0;
				if (!strncmp(pr->name, "reg", 3)) {
					struct reg_property *reg;

					dp->n_addrs = pr->length /
						sizeof(struct address_range);
					dp->addrs = (struct address_range *)
						    ALIGN_NBPMAC(mem);
					i = dp->n_addrs
						*sizeof(struct address_range);
					mem = (unsigned long)(dp->addrs
							      + dp->n_addrs);
					mcpy((void *)dp->addrs,
					     (void *)pr->value, i);
					pp->length = dp->n_addrs
						*sizeof(struct reg_property);
					pp->value = (char *) ALIGN_NBPMAC(mem);
					reg = (struct reg_property *) pp->value;
					for (i = 0; i < dp->n_addrs; i++) {
						reg[i].address =
							dp->addrs[i].address;
						reg[i].size = dp->addrs[i].size;
					}
					mem = (unsigned long)(reg+dp->n_addrs);
				} else {
					pp->length = pr->length;
					pp->value = (char *) mem;
					mcpy((void *)pp->value,
					     (void *)pr->value, pr->length);
					mem += pr->length;
				}
				*((char*)(++mem)) = 0;
			}
			pr++;
		}
		*prevp = dp;
		prevp = &dp->allnext;
	}
	*prevp = 0;

	dev_tree_size = mem - (unsigned long) allnodes;
	klimit = (char *) mem;
}


static void __init
mcpy(void *dst, const void *src, int n)
{
	int i = n;
	unsigned char *d = (unsigned char *)dst;
	unsigned char *s = (unsigned char *)src;

	for (; i > 0; --i)
		*d++ = *s++;
}


static void __init
mset(void *adr, int val, int n)
{
	int i = n;
	unsigned char *a = (unsigned char *)adr;

	for(; i > 0; --i)
		*a++ = val;
}


void __init
amic_dma_init(void)
{
	struct device_node *dma;
	volatile unsigned char *ctrl;
	extern unsigned char *amic_dma_buf_base;

#define HPV_DEBUG
#define AV_DEBUG
//#define BART_DEBUG

#ifdef HPV_DEBUG
	printk("HPV card debug address @ 0x%p\n",
		ioremap(0xfec00000,0x1000));
#endif /* HPV_DEBUG */

#ifdef AV_DEBUG
	printk("AV card frame buffer   @ 0x%p\n",
		ioremap(0xe0100000,0x800000));
	printk("AV card control regs   @ 0x%p\n",
		ioremap(0xe0036000,0x1000));
	printk("AV card misc regs      @ 0x%p\n",
		ioremap(0xe0f20000,0x11000));
#endif

#ifdef BART_DEBUG
	printk("BART NuBus Controller @ 0x%p\n",
		ioremap(0xf0000000,0x100));
#endif

	dma = find_devices("amic-dma");
	if (!dma)
		return;

	/*
	 * Set up AMIC DMA buffer space
	 */
	ctrl = (volatile unsigned char *) ioremap(dma->addrs[0].address, 0x10);
	*ctrl++ = ((unsigned long) amic_dma_buf_base >> 24) & 0xff;
	*ctrl++ = ((unsigned long) amic_dma_buf_base >> 16) & 0xff;
	*ctrl++ = ((unsigned long) amic_dma_buf_base >> 8) & 0xff;
	*ctrl = (unsigned long) amic_dma_buf_base & 0xff;

	/*
	 * Reset RxDMA connected to MACE
	 */
	dma = find_devices("mace-amic");
	if (dma) {
		ctrl = (volatile unsigned char *)
			ioremap(dma->addrs[1].address + 0x408, 0x10);
		out_8(ctrl, 1);
	}
}
#endif

void __init
nbpbook_exp_init(void)
{
	u8 __iomem *dptr;
	struct device_node *keystone;
        const u32 *reg;
        unsigned long addr, size;

#define KEYSTONE_DEBUG

	keystone = of_find_node_by_name(NULL, "keystone");
	if (keystone) {
		reg = of_get_property(keystone, "reg", NULL);
                addr = reg[0];
                size = reg[1];
	} else
		return;

	dptr = (u8 __iomem *)ioremap(addr, size);
	if (!dptr) {
#ifdef KEYSTONE_DEBUG
		printk("0xFEE0000 can't be mapped\n");
#endif
		return;
	}

	/* check if the said location is readable */
	if (!hwreg_present(dptr + 0x13))
		goto out;

	/* if video irq is on, disable it */
	if (in_8(dptr + 0x13) == 2)
		out_8(dptr + 0x13, 0x7f);

#ifdef KEYSTONE_DEBUG
	reg = of_get_property(keystone, "address", NULL);
	addr = reg[0];
	size = 0x80000;

	printk("Keystone @ 0x%p\n", ioremap(addr,size));
#endif

out:
	iounmap((void *)dptr);
	return;
}

void __init
pfm_vlk_init(void)
{
        const u32 *reg;
        unsigned long addr, size;
	struct device_node *vlk;
	u8 __iomem *ctrl;
	if (ppc_md.progress) ppc_md.progress( "pfm_vlk_init(): enter\n", 0 );

	/*
	 * Disable valkyrie intrs
	 */
	vlk = of_find_node_by_name(NULL, "valkyrie");
	if (vlk) {
	    	if (ppc_md.progress)	
		   ppc_md.progress( "pfm_vlk_init(): Disable valkyrie intrs\n", 0 );
		reg = of_get_property(vlk, "reg", NULL);
                addr = reg[2];
                size = reg[3];
		ctrl = (u8 __iomem *)ioremap(addr, size);
		out_8(ctrl + 0x18, 0x7f);
		out_8(ctrl + 0x10, 3);
		iounmap((void *)ctrl);
	}
}

#if 0
static void __init mem_pieces_remove_zero(struct mem_pieces *mp)
{
	int i, j;

	for (i = 0; i < mp->n_regions; ++i) {
		if (mp->regions[i].size == 0) {
			for (j = i; j < mp->n_regions - 1; ++j) {
				mp->regions[j].address = mp->regions[j+1].address;
				mp->regions[j].size = mp->regions[j+1].size;
			}
			--mp->n_regions;
			--i;
		}
	}
}

/*
 * Read in a property describing some pieces of memory.
 */

static int __init
get_mem_prop(char *name, struct mem_pieces *mp)
{
	struct reg_property *rp;
	int s;

	rp = (struct reg_property *) get_property(memory_node, name, &s);
	if (rp == NULL) {
		printk(KERN_ERR "error: couldn't get %s property on /memory\n",
		       name);
		return 0;
	}
	mp->n_regions = s / sizeof(mp->regions[0]);
	memcpy(mp->regions, rp, s);

	/* remove zero sized entry */
	mem_pieces_remove_zero(mp);

	/* Make sure the pieces are sorted. */
	mem_pieces_sort(mp);
	mem_pieces_coalesce(mp);
	return 1;
}

#ifdef CONFIG_DISCONTIGMEM
/*
 * Set phys_avail to the amount of physical memory,
 * less the kernel text/data/bss.
 */
void __init
set_discontig_avail(struct mem_pieces *mp)
{
	unsigned long kstart, ksize;

	/*
	 * Initially, available phyiscal memory is equivalent to all
	 * physical memory.
	 */

	phys_avail = *mp;

	/*
	 * Map out the kernel text/data/bss from the available physical
	 * memory.
	 */

	kstart = __pa(_stext);
	ksize = PAGE_ALIGN(klimit - _stext);

	mem_pieces_remove(&phys_avail, kstart, ksize, 0);
	mem_pieces_remove(&phys_avail, 0, 0x4000, 0);

#if defined(CONFIG_BLK_DEV_INITRD)
	/* Remove the init RAM disk from the available memory. */
	if (initrd_start) {
		mem_pieces_remove(&phys_avail, __pa(initrd_start),
				  initrd_end - initrd_start, 1);
	}
#endif /* CONFIG_BLK_DEV_INITRD */
	/* Remove PDM VRAM area and pre-allocate AMIC dma buffer */
	if (boot_infos && (boot_infos->architecture & BOOT_ARCH_NUBUS_PDM)) {
		extern unsigned char *amic_dma_buf_base;
#if 0
		mem_pieces_remove(&phys_avail, 0x100000, 0x96000, 1);
		amic_dma_buf_base = (unsigned char *)
				__pa(mem_pieces_find(0x40000, 0x40000));
#else
		/*
		 * Things to consider:
		 * - we want to remap vram area to high address (may not be
		 *   necessary)
		 * - curretly use BAT with cache off
		 * - cache off memory can't be used as a part of main memory
		 *   easily
		 * - this code wastes about 150kB
		 */
		mem_pieces_remove(&phys_avail, 0x100000, 0x100000, 1);
		amic_dma_buf_base = (unsigned char *)0x1c0000;
#endif
	}
}
#endif /* CONFIG_DISCONTIGMEM */

/*
 * On systems with Open Firmware, collect information about
 * physical RAM and which pieces are already in use.
 * At this point, we have (at least) the first 8MB mapped with a BAT.
 * Our text, data, bss use something over 1MB, starting at 0.
 * Open Firmware may be using 1MB at the 4MB point.
 */
unsigned long __init
nbpmac_find_end_of_memory(void)
{
	unsigned long a, total;
#ifndef CONFIG_DISCONTIGMEM
	struct mem_pieces phys_mem;
#else
	unsigned long memend;
	int i;
#endif

	/*
	 * Find out where physical memory is, and check that it
	 * starts at 0 and is contiguous.  It seems that RAM is
	 * always physically contiguous on Power Macintoshes.
	 * (Note: PDM NuBus Power Macintoshes have discontiguous RAM.)
	 *
	 * Supporting discontiguous physical memory isn't hard,
	 * it just makes the virtual <-> physical mapping functions
	 * more complicated (or else you end up wasting space
	 * in mem_map).
	 */
	memory_node = find_devices("memory");
	if (memory_node == NULL || !get_mem_prop("reg", &phys_mem)
	    || phys_mem.regions == 0)
		panic("No RAM??");
	a = phys_mem.regions[0].address;
	if (a != 0)
		panic("RAM doesn't start at physical address 0");
	total = phys_mem.regions[0].size;

#ifndef CONFIG_DISCONTIGMEM
	if (phys_mem.n_regions > 1) {
		printk("RAM starting at 0x%x is not contiguous\n",
		       phys_mem.regions[1].address);
		printk("Using RAM from 0 to 0x%lx\n", total-1);
	}

	return total;
#else
	memend = total;
	for (i = 1; i < phys_mem.n_regions; i++) {
		if (i == MAX_MEM_MAP_SIZE) {
			printk(KERN_WARNING "pmac_find_end_of_memory: "
			       "Memory map being truncated\n");
       			break;
		}
		memend = phys_mem.regions[i].address + phys_mem.regions[i].size;
		total += phys_mem.regions[i].size;
		if (__max_memory && total > __max_memory) {
			phys_mem.regions[i].size -= total - __max_memory;
			memend -= total - __max_memory;
			total = __max_memory;
			break;
		}
	}
	if (i < phys_mem.n_regions)
		phys_mem.n_regions = i;
	real_memory = total;

	set_discontig_avail(&phys_mem);

	return memend;
#endif
}


EXPORT_SYMBOL(nbpmac_slot2irq);
EXPORT_SYMBOL(nbpmac_irq2slot);
#endif
EXPORT_SYMBOL(hwreg_present);

