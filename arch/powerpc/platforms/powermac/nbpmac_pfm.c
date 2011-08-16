/*
 *
 *  Interrupt handling routines for Performa 5/6200.
 *  These routines are roughly based on arch/ppc/kernel/pmac_pic.c
 *  which, I believe, was written mainly by Paul Mackerras.
 *
 *  Performa 5/6200's hardware is convoluted and this code wouldn't have
 *  existed without hardware information decoded for MkLinux by
 *  David Gatwood.
 *
 *  Copyright (C) 2001 Takashi Oe (toe@users.sf.net)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */

/*

	via1 (0 - 7)
	------------
	00| slot1
	01|
	02| via cuda
	03|
	04|
	05|
	06|
	07|

	via2
	------------------
	08| scsi1
	09| slot2
	10|
	11| scsi0
	12| audio out
	13|
	14|
	15|

	slot1
	------------------
	16|
	17|
	18|
	19|
	20|
	21|
	22|
	23| scc A

	slot2
	------------------
	24| nubus0
	25|
	26|
	27|
	28|
	29| pds
	30| f108
	31|

	f108 (0 - 6)
	---------------
	32|
	33|
	34| IR
	35| CC
	36| ATA1
	37| ATA0
	38| vbl
	39|

	----------------------
	40 possible irq sources

*/

//#include <linux/config.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/signal.h>
#include <linux/spinlock.h>

#include <asm/sections.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/machdep.h>

#include "nbpmac.h"


struct pfm_irq_hw {
	u8 __iomem *icr1;
	u8 __iomem *icr2;
	u8 __iomem *via1_flag;
	u8 __iomem *via1_ena;
	u8 __iomem *via1_pcr;
	u8 __iomem *slt1_flag;
	u8 __iomem *via2_flag;
	u8 __iomem *via2_ena;
	u8 __iomem *slt2_flag;
	u8 __iomem *f108_flag;
	u16 __iomem *f108_ack;
};

#define NR_MASK_WORDS   ((NR_IRQS + 31) / 32)
//static unsigned long ppc_lost_interrupts[NR_MASK_WORDS];
static unsigned long ppc_cached_irq_mask[NR_MASK_WORDS];
static struct irq_host *pmac_pfm_host;

//extern irq_desc_t irq_desc[];
//extern int ppc_spurious_interrupts;

//static int unhandled_irq[40];
static u8 __iomem *serial;
static u8 __iomem *scsi;

static struct pfm_irq_hw pfm_irq;
static DEFINE_SPINLOCK(pfm_lock);

//static int pfm_irq_got = 0;
static int max_irqs = 40;
static unsigned char f108_pen, via1_pen, slt1_pen, via2_pen, slt2_pen;
static unsigned char f108_ena, via1_ena, slt1_ena, via2_ena, slt2_ena;

#define VIA1_DEV	0x04
#define VIA1_SLOT	0x1
#define VIA2_SLOT	0x2
#define SLOT2_DEV	0x61
#define AMIC_SET	0x80
#define AMIC_CLR	0x00
#define AMIC_ACK	0x80
#define AMIC_MASK(a)	((a)&0x7f)

#if 0
static inline void
pfm_irq_dispatch_handler(struct pt_regs *regs, int irq)
{
	struct irqaction *action;
	int cpu;

	action = irq_desc[irq].action;
	cpu = smp_processor_id();
	kstat.irqs[cpu][irq]++;
	if (action && action->handler) {
		do {
			action->handler(irq, action->dev_id, regs);
			action = action->next;
		} while ( action );
	} else {
		if (unhandled_irq[irq] < 5) {
			ppc_spurious_interrupts++;
			printk("Unhandled interrupt %d\n", irq);
			unhandled_irq[irq]++;
		}
	}
}
#endif

static void
pfm_set_irq_mask(unsigned int irq_nr)
{
	int i = irq_nr >> 3, is_slot;
	volatile unsigned char *ctrler;
	unsigned char bit, *ena;
	unsigned char flag, mask;
	unsigned long irq_flags;

	if (irq_nr >= max_irqs)
		return;

	bit = 1U << (irq_nr & 0x7);
	flag = test_bit(irq_nr, ppc_cached_irq_mask)? bit: 0;

//printk(KERN_ERR "irq %d, i %d, bit %02X, flag %02X\n",
//(int)irq_nr, i, bit, flag);

	mask = bit;
	is_slot = 0;
	switch (i) {
	default:
		return;
	case 0:
		ctrler = pfm_irq.via1_ena;
		ena = &via1_ena;
		break;
	case 1:
		ctrler = pfm_irq.via2_ena;
		ena = &via2_ena;
		break;
	case 2:
		ena = &slt1_ena;
		ctrler = pfm_irq.via1_ena;
		mask = VIA1_SLOT;
		is_slot = 1;
		break;
	case 3:
		ena = &slt2_ena;
		ctrler = pfm_irq.via2_ena;
		mask = VIA2_SLOT;
		is_slot = 1;
		break;
	case 4:
		ena = &f108_ena;
		ctrler = pfm_irq.via2_ena;
		mask = VIA2_SLOT;
		is_slot = 1;
		break;
	}

	spin_lock_irqsave(&pfm_lock, irq_flags);
	if (flag) {
		*ena |= bit;
		out_8(ctrler, AMIC_SET | mask);
	} else {
		*ena &= ~bit;
		if (!is_slot)
			out_8(ctrler, AMIC_CLR | mask);
	}
	mb();
	spin_unlock_irqrestore(&pfm_lock, irq_flags);
}

static void
pfm_disable_irq(unsigned int irq_nr)
{
    printk( KERN_INFO "%s(): disable IRQ #%d\n", __func__, irq_nr );
	clear_bit(irq_nr, ppc_cached_irq_mask);
	pfm_set_irq_mask(irq_nr);
	mb();
}

static void
pfm_enable_irq(unsigned int irq_nr)
{
    printk( KERN_INFO "%s(): enable IRQ #%d\n", __func__, irq_nr );
	set_bit(irq_nr, ppc_cached_irq_mask);
	pfm_set_irq_mask(irq_nr);
}

#if 0
static void
pfm_end_irq(unsigned int irq_nr)
{
    return;
#if 0
    if (!(irq_desc[irq_nr].status & (IRQ_DISABLED|IRQ_INPROGRESS))
        && irq_desc[irq_nr].action) {
        set_bit(irq_nr, ppc_cached_irq_mask);
        pfm_set_irq_mask(irq_nr);
    }
#endif
}
#endif

static void
pfm_check_irq(void)
{
    via2_pen = AMIC_MASK(in_8(pfm_irq.via2_flag));
    out_8(pfm_irq.via2_flag, via2_pen & AMIC_MASK(in_8(pfm_irq.via2_ena)));

    f108_pen = in_8(pfm_irq.f108_flag);
    slt1_pen = ~in_8(pfm_irq.slt1_flag);
    slt2_pen = AMIC_MASK(~in_8(pfm_irq.slt2_flag));

    /*
     * clear bit 2 and/or 3 if bit 0 and/or 1 is set
     */
    f108_pen &= ~((~f108_pen & 0x3) << 2);
    f108_pen &= 0xfc;
    if (f108_pen) {
        out_be16(pfm_irq.f108_ack, f108_pen & 0x7c);
        out_be16(pfm_irq.f108_ack, 0);
    }

    /* remove all bits not directly connected to any device */
    via2_pen &= ~VIA2_SLOT;
    slt2_pen &= SLOT2_DEV;
    slt1_pen &= 0x80;
}

static unsigned int
pfm_get_irq(void)
{
    int cnt = 1;
    unsigned char scc, esp;

    //printk( KERN_INFO "%s(): enter\n", __func__ );

    via1_pen = AMIC_MASK(in_8(pfm_irq.via1_flag));
    out_8(pfm_irq.via1_flag, via1_pen & AMIC_MASK(in_8(pfm_irq.via1_ena)));
    via1_pen &= VIA1_DEV;

    //DO_IRQ(via1_pen, 0x04, 0, 2, 2);
    if( via1_pen )
    {
//        __do_IRQ(2, regs);
	return irq_linear_revmap(pmac_pfm_host, 2);
    }

more:
    /*
     * I don't know where to look at for SCC irq, so we ask device
     * if the irq is indeed coming from them.
     */
    out_8(serial, 3);
    scc = in_8(serial);

    /*
     * On ESP SCSI, we seem to lose an irq, and we get no irq any longer..
     * so check it out.
     */
    esp = in_8(scsi);

    if (scc & 0x3f)
        slt1_pen |= 0x80;

    if (esp & 0x80)
        via2_pen |= 0x08;

#if 0
    if (via1_pen || via2_pen || slt1_pen || slt2_pen || f108_pen)
    printk(KERN_ERR "%02d:via1:%02X,via2:%02X,slt1:%02X,slt2:%02X,"
        "f108:%02X,pcr:%02X,scc:%02X\n",
        cnt,via1_pen,via2_pen,
        0xff&(~in_8(pfm_irq.slt1_flag)),
        AMIC_MASK(~in_8(pfm_irq.slt2_flag)),
        in_8(pfm_irq.f108_flag),
        in_8(pfm_irq.via1_pcr),
        scc
    );
#endif

    if( via2_pen )
    {
        //DO_IRQ(via2_pen, 0x10, 8, 4, 3);
        if(via2_pen & 0x10)
        {
//            __do_IRQ(12, regs);
		return irq_linear_revmap(pmac_pfm_host, 12);
        }
        if(via2_pen & 0x08)
        {
//            __do_IRQ(11, regs);
		return irq_linear_revmap(pmac_pfm_host, 11);
        }

        //DO_IRQ(via2_pen, 0x01, 8, 0, 0);
        if(via2_pen & 0x01)
        {
//            __do_IRQ(8, regs);
		return irq_linear_revmap(pmac_pfm_host, 8);
        }
    }

    //DO_IRQ(f108_pen, 0x20, 32, 5, 2);
    if( f108_pen )
    {
        if(f108_pen & 0x20)
        {
//            __do_IRQ(37, regs);
		return irq_linear_revmap(pmac_pfm_host, 37);
        }
        if(f108_pen & 0x10)
        {
//            __do_IRQ(36, regs);
		return irq_linear_revmap(pmac_pfm_host, 36);
        }
        if(f108_pen & 0x08)
        {
//            __do_IRQ(35, regs);
		return irq_linear_revmap(pmac_pfm_host, 35);
        }
        if(f108_pen & 0x04)
        {
//            __do_IRQ(34, regs);
		return irq_linear_revmap(pmac_pfm_host, 34);
        }
    }

    //DO_IRQ(slt1_pen, 0x80, 16, 7, 7);
    if( slt1_pen & 0x80 )
    {
//        __do_IRQ(23, regs);
		return irq_linear_revmap(pmac_pfm_host, 23);
    }

    //DO_IRQ(slt2_pen, 0x20, 24, 5, 0);
    if( slt2_pen )
    {
        if( slt2_pen & 0x20 )
        {
//            __do_IRQ(29, regs);
		return irq_linear_revmap(pmac_pfm_host, 29);
        }
        if( slt2_pen & 0x10 )
        {
//            __do_IRQ(28, regs);
		return irq_linear_revmap(pmac_pfm_host, 28);
        }
        if( slt2_pen & 0x08 )
        {
//            __do_IRQ(27, regs);
		return irq_linear_revmap(pmac_pfm_host, 27);
        }
        if( slt2_pen & 0x04 )
        {
//            __do_IRQ(26, regs);
		return irq_linear_revmap(pmac_pfm_host, 26);
        }
        if( slt2_pen & 0x02 )
        {
//            __do_IRQ(25, regs);
		return irq_linear_revmap(pmac_pfm_host, 25);
        }
        if( slt2_pen & 0x01 )
        {
//            __do_IRQ(24, regs);
		return irq_linear_revmap(pmac_pfm_host, 24);
        }
    }

    if (cnt++ > 10)
    {
        //printk( KERN_INFO "%s(): handled more than enough\n", __func__ );
        return NO_IRQ;
    }

    pfm_check_irq();

    if (slt1_pen || slt2_pen || via2_pen)
    {
        //printk( KERN_INFO "%s(): still pending IRQs available\n", __func__ );
        goto more;
    }

    //printk( KERN_INFO "%s(): done\n", __func__ );

    /* TODO: For now just assume that there has been at least a single IRQ */
    return NO_IRQ;
}

#if 0
static void
pfm_dummy(unsigned int irq_nr)
{
	/*
	 * do nothing
	 */
}

struct hw_interrupt_type pfm_root = {
	"ROOT",
	NULL,
	NULL,
	pfm_dummy,
	pfm_dummy,
	pfm_dummy,
	NULL,
	NULL
};

static struct irqaction pfm_root_action = {
    .handler    = pfm_action,
//    .flags      = SA_INTERRUPT,
//    .mask       = CPU_MASK_NONE,
    .name       = "Performa root",
};

struct hw_interrupt_type pmac_pfm = {
	"Performa",
	NULL,
	NULL,
	pfm_enable_irq,
	pfm_disable_irq,
	pfm_dummy,
	NULL,
	NULL
};
struct hw_interrupt_type pmac_pfm = {
    "Performa",
    .startup = NULL,           /* TODO: provide a startup handler */
    .shutdown = NULL,
    .enable = pfm_enable_irq,
    .disable = pfm_disable_irq,
    .ack = NULL,
    .end = pfm_end_irq,
    .set_affinity = NULL,
#ifdef CONFIG_IRQ_RELEASE_METHOD
    .release = NULL
#endif
};
#endif

static struct irq_chip pmac_pfm = {
        .typename       = "Performa" ,
        .startup        = NULL,
        .mask           = NULL,
        .ack            = NULL,
        .mask_ack       = pfm_disable_irq,
        .unmask         = pfm_enable_irq,
        .retrigger      = NULL
};
        
static int pmac_pfm_host_match(struct irq_host *h, struct device_node *node)
{
        /* We match all, we don't always have a node anyway */
        return 1;
} 

static int pmac_pfm_host_map(struct irq_host *h, unsigned int virq,
                             irq_hw_number_t hw)
{
        struct irq_desc *desc = get_irq_desc(virq);
        int level = 0;

        if (hw >= max_irqs)
                return -EINVAL;

        /* Mark level interrupts, set delayed disable for edge ones and set
         * handlers
         */                  
//        level = !!(level_mask[hw >> 5] & (1UL << (hw & 0x1f)));
//        if (level)
//                desc->status |= IRQ_LEVEL;
        desc->status &= ~IRQ_LEVEL;
        set_irq_chip_and_handler(virq, &pmac_pfm, level ?
                                 handle_level_irq : handle_edge_irq);
        return 0;
}

static int pmac_pfm_host_xlate(struct irq_host *h, struct device_node *ct,
                               u32 *intspec, unsigned int intsize,
                               irq_hw_number_t *out_hwirq,
                               unsigned int *out_flags)
        
{
        *out_flags = IRQ_TYPE_NONE;
        *out_hwirq = *intspec;
        return 0;
}

static struct irq_host_ops pmac_pfm_host_ops = {
        .match = pmac_pfm_host_match,
        .map = pmac_pfm_host_map,
        .xlate = pmac_pfm_host_xlate,
};
 
#if 0
static unsigned int
pfm_get_irq(void)
{
	unsigned char x;

	//printk( KERN_INFO "%s(): enter\n", __func__ );

	/* are we ready? */
	if (!pfm_irq.icr1)
	{
	    if (ppc_md.progress)
		ppc_md.progress( "pfm_get_irq(): called too early!\n", 0 );
	    printk( KERN_ERR "%s(): called too early!\n", __func__ );
	    return -2;
	}

#if 0
	/* once is enough */
	if (pfm_irq_got) {
		pfm_irq_got = 0;
		return -1;
	}
	pfm_irq_got = 1;
#endif

	/* acknowledge irq */
	out_8(pfm_irq.icr1, 1);
	x = in_8(pfm_irq.icr1);
	out_8(pfm_irq.icr2, 7);

	pfm_check_irq();

	return max_irqs-1;
}

#define DO_IRQ(PEND,BIT,OFF,INI,FIN)					\
do {									\
	unsigned int bit;						\
	int i;								\
	if (PEND) {							\
		for (i = INI, bit = BIT; i >= FIN; --i, bit >>= 1) {	\
			if (PEND & bit)					\
				pfm_irq_dispatch_handler(regs, i + OFF);\
		}							\
	}								\
} while (0)
#endif

#if 0
static void
pfm_root_action(int cpl, void *dev_id, struct pt_regs *regs)
{
	int cnt = 1;
	unsigned char scc, esp;

	via1_pen = AMIC_MASK(in_8(pfm_irq.via1_flag));
	out_8(pfm_irq.via1_flag, via1_pen & AMIC_MASK(in_8(pfm_irq.via1_ena)));
	via1_pen &= VIA1_DEV;

	DO_IRQ(via1_pen, 0x04, 0, 2, 2);

more:
	/*
	 * I don't know where to look at for SCC irq, so we ask device
	 * if the irq is indeed coming from them.
	 */
	out_8(serial, 3);
	scc = in_8(serial);

	/*
	 * On ESP SCSI, we seem to lose an irq, and we get no irq any longer..
	 * so check it out.
	 */
	esp = in_8(scsi);

	if (scc & 0x3f)
		slt1_pen |= 0x80;

	if (esp & 0x80)
		via2_pen |= 0x08;

#if 0
	if (via1_pen || via2_pen || slt1_pen || slt2_pen || f108_pen)
	printk(KERN_ERR "%02d:via1:%02X,via2:%02X,slt1:%02X,slt2:%02X,"
		"f108:%02X,pcr:%02X,scc:%02X\n",
		cnt,via1_pen,via2_pen,
		0xff&(~in_8(pfm_irq.slt1_flag)),
		AMIC_MASK(~in_8(pfm_irq.slt2_flag)),
		in_8(pfm_irq.f108_flag),
		in_8(pfm_irq.via1_pcr),
		scc
	);
#endif

	DO_IRQ(via2_pen, 0x10, 8, 4, 3);
	DO_IRQ(via2_pen, 0x01, 8, 0, 0);
	DO_IRQ(f108_pen, 0x20, 32, 5, 2);
	DO_IRQ(slt1_pen, 0x80, 16, 7, 7);
	DO_IRQ(slt2_pen, 0x20, 24, 5, 0);

	if (cnt++ > 10)
		return;

	pfm_check_irq();

	if (slt1_pen || slt2_pen || via2_pen)
		goto more;
}
#endif

void __init
pmac_pfm_init(void)
{
//    int i;
    struct device_node *irqctrler;
    const u32 *reg;
    unsigned long addr, size;
    unsigned char x;
    unsigned int flags = 0;

	irqctrler = of_find_node_by_name(NULL, "performa-intr");
	if (irqctrler) {
                reg = of_get_property(irqctrler, "reg", NULL);
                addr = reg[0];
                size = reg[1];
                pfm_irq.icr1 = (u8 __iomem *)ioremap(addr, size);
                addr = reg[2];
                size = reg[3];
                pfm_irq.icr2 = (u8 __iomem *)ioremap(addr, size);
                addr = reg[4];
                size = reg[5];
                pfm_irq.via1_flag = (u8 __iomem *)ioremap(addr, size);
                addr = reg[6];
                size = reg[7];
                pfm_irq.via1_ena = (u8 __iomem *)ioremap(addr, size);
                addr = reg[8];
                size = reg[9];
                pfm_irq.via1_pcr = (u8 __iomem *)ioremap(addr, size);
                addr = reg[10];
                size = reg[11];
                pfm_irq.slt1_flag = (u8 __iomem *)ioremap(addr, size);
                addr = reg[12];
                size = reg[13];
                pfm_irq.via2_flag = (u8 __iomem *)ioremap(addr, size);
                addr = reg[14];
                size = reg[15];
                pfm_irq.via2_ena = (u8 __iomem *)ioremap(addr, size);
                addr = reg[16];
                size = reg[17];
                pfm_irq.slt2_flag = (u8 __iomem *)ioremap(addr, size);
                addr = reg[18];
                size = reg[19];
                pfm_irq.f108_flag = (u8 __iomem *)ioremap(addr, size);
                addr &= ~3;
                pfm_irq.f108_ack = (u16 __iomem *)ioremap(addr, size);
	} else {
	    if (ppc_md.progress)
		ppc_md.progress("Failed to obtain Performa irq addresses/interrupts\n", 0);
		panic("Failed to obtain Performa irq addresses/interrupts\n");
	}

	/*
	 * SCC channel A control reg anc ESP SCSI status reg.  We ask device
	 * if it caused the irq.  There must be a better way..
	 */
	serial = (u8 __iomem *)ioremap(0x50f0c002,0x10);
	scsi = (u8 __iomem *)ioremap(0x50f10040, 0x10);

	/* disable all interrupts */
	out_be32((u32 __iomem *)pfm_irq.icr1, 1);
	x = in_8(pfm_irq.icr1);
	out_8(pfm_irq.icr1, 1);
	x = in_8(pfm_irq.icr1);
	out_8(pfm_irq.icr2, 7);
	x = in_8(pfm_irq.icr2);

	out_8(pfm_irq.via1_pcr, 0);
	out_8(pfm_irq.via1_ena, 0x7f);
	out_8(pfm_irq.via1_flag, 0x7f);
	out_8(pfm_irq.via2_flag, 0x7f);

	out_8(pfm_irq.f108_flag, 3);
	out_8(pfm_irq.f108_flag, 0xfc);

	f108_ena = via1_ena = via2_ena = slt1_ena = slt2_ena = 0;

	/* register root irq handler */
	//irq_desc[max_irqs].handler = &pfm_root;
	//request_irq(max_irqs, pfm_root_action, SA_INTERRUPT, "Performa", 0);
/*
	for (i = 0; i < max_irqs ; i++ )
		irq_desc[i].handler = &pmac_pfm;

	if( setup_irq(max_irqs-1, &pfm_root_action) != 0 )
	{
	    if (ppc_md.progress)
	        ppc_md.progress( "pmac_pfm_init(): Failed register root handler!\n",
                             0 );
	    printk( KERN_ERR "%s(): Failed register root handler!\n", __func__ );
	}
*/
        /* We configure the OF parsing based on our
         * platform type and wether we were booted by BootX.
         */
        flags |= OF_IMAP_OLDWORLD_MAC;
        if (of_get_property(of_chosen, "linux,bootx", NULL) != NULL)
                flags |= OF_IMAP_NO_PHANDLE;

        of_irq_map_init(flags);

        spin_lock_init(&pfm_lock);

        ppc_md.get_irq = pfm_get_irq;

        pmac_pfm_host = irq_alloc_host(irqctrler, IRQ_HOST_MAP_LINEAR, max_irqs,
                                       &pmac_pfm_host_ops,
                                       max_irqs);
        BUG_ON(pmac_pfm_host == NULL);
        irq_set_default_host(pmac_pfm_host);

        of_node_put(irqctrler);

	printk("System has %d possible interrupts\n", max_irqs);

//#ifdef CONFIG_XMON
//	/* fake irq line - 5 */
//	request_irq(5, xmon_irq, 0, "NMI - XMON", 0);
//	icr_ena |= 0x20;
//#endif	/* CONFIG_XMON */
}

