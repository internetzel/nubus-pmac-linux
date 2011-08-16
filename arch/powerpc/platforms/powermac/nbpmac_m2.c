#define CONFIG_ADB_PMU_NBPMAC_ALT
#define USE_SPINLOCK
#undef DEBUG
#undef HANDLE_BABOON
/*
 *
 *  Interrupt handling routines for PPC-capable NuBus PowerBooks.
 *  These routines are roughly based on arch/ppc/kernel/pmac_pic.c
 *  which, I believe, was written mainly by Paul Mackerras.
 *
 *  Copyright (C) 2001 Takashi Oe (toe@users.sourceforge.net)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */

/*

Some info on M2 interrupt table used in here.


icr @ root -> via1, via2 @ secondary
                      |
                      +-> slot @ third level


Note for the cryptic tables below:

The first number is the interrupt number to be used in this source.  The second
number after "|", if any, is the device number that Apple uses.  For
example, "irq = 19" in this source refers to SCSI0 device interrupt which
resides at bit 3 of pseudo VIA2 register.  In Apple's numbering, this has a
device number of 128.

	icr (0 - 7) -- all non-maskable?
	-----------
	00| bit one of autovector interrupt level
	01| bit two of autovector interrupt level
	02| bit three of autovector interrupt level
	level 0| no interrupt?
	level 1| VIA1
	level 2| VIA2
	level 3| N/A?
	level 4| SCC
	level 5| N/A?
	level 6| N/A?
	level 7| NMI
	03|
	04|
	05|
	06| ?, mode bit
	07| ?, ack bit

	via1 (0 - 6) -- all maskable
	------------
	08| ?, cascade
	09| 300: dev_hz
	10| shift register interrupt
	11| ?,
	12| ???: PMU, ADB interrupt (among others)
	13| 302: dev_timer2
	14| 301: dev_timer1
	15| any interrupt pending

	via2 (0 - 6) -- all maskable (?)
	------------------
	16| SCSI DRQ
	17| slot
	18| N/A
	19| 128: dev_scsi0
	20| ?, possibly audio
	21| 136: dev_fd
	22| 129: dev_scsi1 (not connected?)
	23| any interrupt pending

	slot (0 - 7) -- all non-maskable
	---------------
	24| ?, PCMCIA (TREX) (on 2300: baboon)
	25| ?, CSC (hardware cursor?)
	26| 259: dev_nubus3 (modem?)
	27| ???: baboon
	28| ???: N/A
	29| 303: dev_vbl (internal expansion card; video or ethernet)
	30| ???: N/A
	31| ???: N/A

	baboon (0 - 2) -- all non-maskable (?)
	---------------
	32| IDE1 (media bay drive)
	33| IDE0 (internal hard disk)
	34| media bay (insertion/ejection)

	----------------------
	35 possible irq sources

*/

#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/signal.h>
#ifdef USE_SPINLOCK
#include <linux/spinlock.h>
#endif
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include <asm/sections.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/machdep.h>
#include <asm/time.h>
#include <asm/xmon.h>

#include "pmac.h"

struct m2_irq_hw {
	u8 __iomem *icr;
	u8 __iomem *via1;
	u8 __iomem *via2;
	u8 __iomem *baboon_flag;
};

#define NR_MASK_WORDS   ((NR_IRQS + 31) / 32)
static unsigned long ppc_lost_interrupts[NR_MASK_WORDS];
static unsigned long ppc_cached_irq_mask[NR_MASK_WORDS];
static struct irq_host *pmac_m2_host;

static struct m2_irq_hw m2_irq;

static int m2_irq_got = 0;
static int max_irqs = 35;
static int last_irq = -1;
static u8 icr_pen, via1_pen, via2_pen, slot_pen, baboon_pen;
static u8 icr_ena, via1_ena, via2_ena, slot_ena, baboon_ena;

#ifdef USE_SPINLOCK
static DEFINE_SPINLOCK( m2_lock );
#endif

#if 0
static struct timer_list m2_timer;
#define M2_TIMER (jiffies + 5*HZ/1000)
#endif

static u8 deactivated_slot_interrupt_pending = 0;
static u8 is_PB500PPC = 0;

static inline void __m2_check_irq(u8 bits, s8 mask);

#define ICR_DEVICE	0x24	/* bits on ICR which give interrupt direct */
				/* from devices (other bits are cascaded) */
#define VIA1_MASK	0x14	/* handle pmu and shift-reg interrupts */
#define VIA2_SLOT	0x2
#ifdef HANDLE_BABOON
static u8 SLOT_BABOON =	0x8;
static u8 IRQ_BABOON =	0;
#define BABOON_MASK	0x7
#endif
#define M2_IRQ_PENDING	0x80
#define M2_SET		0x80
#define M2_CLR		0x00
#define M2_ACK		0x80
#define M2_MASK(a)	((a)&0x7f)

#define RS		0x200		/* skip between registers */
#define B		0		/* B-side data */
#define A		RS		/* A-side data */
#define DIRB		(2*RS)		/* B-side direction (1=output) */
#define DIRA		(3*RS)		/* A-side direction (1=output) */
#define T1CL		(4*RS)		/* Timer 1 ctr/latch (low 8 bits) */
#define T1CH		(5*RS)		/* Timer 1 counter (high 8 bits) */
#define T1LL		(6*RS)		/* Timer 1 latch (low 8 bits) */
#define T1LH		(7*RS)		/* Timer 1 latch (high 8 bits) */
#define T2CL		(8*RS)		/* Timer 2 ctr/latch (low 8 bits) */
#define T2CH		(9*RS)		/* Timer 2 counter (high 8 bits) */
#define SR		(10*RS)		/* Shift register */
#define ACR		(11*RS)		/* Auxiliary control register */
#define PCR		(12*RS)		/* Peripheral control register */
#define IFR		(13*RS)		/* Interrupt flag register */
#define IER		(14*RS)		/* Interrupt enable register */
#define ANH		(15*RS)		/* A-side data, no handshake */

/* Bits in ACR */
#define T1MODE		0xe0		/* Timer 1 mode control bits */
#define T1MODE_CONT	0x40		/* Timer 1 continuous interrupts */
#define T2MODE_CONT	0x20		/* Timer 2 mode  control bits */
#define SR_CTRL		0x1c		/* Shift register control bits */
#define SR_OUT		0x10		/* Shift out on 1 (CB1?)*/
#define SR_OUT_1	0x08		/* Shift out on ? (CB2?) */
#define SR_EXT		0x04		/* Shift on external clock */
#define B_LATCH		0x02		/* Port B latch */
#define A_LATCH		0x01		/* Port A latch */

/* Bits in PCR */
#define CB2_DIRECTION	0x80		/* CB2 input(0)/output(1) mode */
#define CB2_IN_EDGE	0x40		/* CB2 interrupt on raising(1)/falling(0) edge */
#define CB2_IN_IRQ_CLR	0x20		/* CB2 interrupt clear on Port B access enable(0)/disable(1)  */
#define CB2_OUT_MODE1	0x40		/* Determines what happens on CB2_OUT_MODE2 */
#define CB2_OUT_MODE2	0x20		/* If CB2_OUT_MODE1=1 CB2 will be constantly set to (CB2_OUT_MODE1) AND (CB2_OUT_MODE2) */
					/* If CB2_OUT_MODE1=CB2_OUT_MODE2=0 CB2 will be set on transition on CB1 and be reset on write to Port B */
					/* If (CB2_OUT_MODE1=0) AND (CB2_OUT_MODE2=1) CB2 will be set high and be set low for one cycle on write to Port B */
#define CB1_EDGE	0x10		/* CB1 interrupt on raising(1)/falling(0) edge */
#define CA2_DIRECTION	0x08		/* CA2 input(0)/output(1) mode*/
#define CA2_IN_EDGE	0x04		/* CA2 interrupt on raising(1)/falling(0) edge */
#define CA2_IN_IRQ_CLR	0x02		/* CA2 interrupt clear on Port A access enable(0)/disable(1)  */
#define CA2_OUT_MODE1	0x04		/* Determines what happens on CA2_OUT_MODE2 */
#define CA2_OUT_MODE2	0x02		/* If CA2_OUT_MODE1=1 CA2 will be constantly set to (CA2_OUT_MODE1) AND (CA2_OUT_MODE2) */
					/* If CA2_OUT_MODE1=CA2_OUT_MODE2=0 CA2 will be set on transition on CA1 and be reset on write to Port A */
					/* If (CA2_OUT_MODE1=0) AND (CA2_OUT_MODE2=1) CA2 will be set high and be set low for one cycle on write to Port A */
#define CA1_EDGE	0x01		/* CA1 interrupt on raising(1)/falling(0) edge */

/* Bits in IFR and IER */
#define IER_SET		0x80		/* set bits in IER */
#define T1_INT		0x40		/* Timer 1 interrupt */
#define T2_INT		0x20		/* Timer 2 interrupt */
#define CB1_INT		0x10		/* CB1 interrupt (Port B, no handshaking) */
#define CB2_INT		0x08		/* CB2 interrupt (Port B, handshaking) */
#define SR_INT		0x04		/* Shift register full/empty */
#define CA1_INT		0x02		/* CA1 interrupt (Port A, no handshaking) */
#define CA2_INT		0x01		/* CA2 interrupt (Port A, handshaking) */
#define IER_CLR		0		/* clear bits in IER */

#if 0
static inline void
__m2_retrigger(unsigned int irq_nr)
{
        if (!__test_and_set_bit(irq_nr, ppc_lost_interrupts)) {
                atomic_inc(&ppc_n_lost_interrupts);
                set_dec(1);
        }
}
#endif

#define toggle_enable(a)	\
	if (flag)		\
		*ena |= (a);	\
	else			\
		*ena &= ~(a);

static inline void
__m2_set_irq_mask(unsigned int irq_nr, u8 shutdown)
{
	int i = irq_nr >> 3;
	volatile __iomem u8 *ctrler;
	u8 bit, *ena, flag;

	if (irq_nr >= max_irqs)
		return;

	bit = 1U << ((i)?(irq_nr & 0x7):0);
	flag = test_bit(irq_nr, ppc_cached_irq_mask)? bit: 0;

	switch (i) {
	case 0:
		ena = &icr_ena;
		toggle_enable(bit);
		return;
	case 1:
		ena = &icr_ena;
		toggle_enable(0x01);
#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
		ctrler = &m2_irq.via1[IER];
		ena = &via1_ena;
#else
		return;
#endif
		break;
#ifdef HANDLE_BABOON
	case 4:
		if (is_PB500PPC)
			return;
		ena = &baboon_ena;
		toggle_enable(bit);
		bit = SLOT_BABOON;
		if (flag)
			flag = bit;
#endif
	case 3:
		ena = &slot_ena;
		toggle_enable(bit);
		bit = CA1_INT;
		if (flag)
			flag = bit;
		// Only permanently disable VIA2 CA1_INT when all slot interrupts disabled
		else if (shutdown && slot_ena)
			// There still is some slot interrupt enabled, so return
			return;
	case 2:
		ena = &icr_ena;
		toggle_enable(0x02);
		ctrler = &m2_irq.via2[IER];
		ena = &via2_ena;
		break;
	default:
		return;
	}
	if (flag && (!(*ena & bit))) {
		if ((in_8(ctrler) & bit) != bit) {
			out_8(ctrler, IER_SET | bit);
		}
		*ena |= bit;
	} else if (!flag && (*ena & bit)) {
		if ((in_8(ctrler) & bit) == bit) {
			out_8(ctrler, IER_CLR | bit);
		}
		*ena &= ~bit;
	}
}

static inline void
__m2_ack_irq(unsigned int irq_nr)
{
	u8 bit;
	int i = irq_nr >> 3;

	bit = 1U << (irq_nr & 0x07);

	switch (i) {
	case 1:
#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
		out_8(&m2_irq.via1[IFR], bit);
#endif
		break;
#ifdef HANDLE_BABOON
	case 4:
		if (is_PB500PPC)
			break;
		out_8(m2_irq.baboon_flag, in_8(m2_irq.baboon_flag) & ~bit);
#endif
	case 3:
		bit = CA1_INT;
	case 2:
		out_8(&m2_irq.via2[IFR], bit);
		break;
	default:
		break;
	}
#if 0
	if (in_8(m2_irq.icr) & 0x80) {
		out_8(m2_irq.icr, M2_ACK);
	}
#endif
}

static void
m2_ack_irq(unsigned int virq)
{
#ifdef USE_SPINLOCK
	unsigned long flags;
#endif
        unsigned int src = irq_map[virq].hwirq;

#ifdef DEBUG
	printk("|A%d", src);
#endif

#ifdef USE_SPINLOCK
	spin_lock_irqsave(&m2_lock, flags);
#endif

        if (__test_and_clear_bit(src, ppc_lost_interrupts))
                atomic_dec(&ppc_n_lost_interrupts);
	__m2_ack_irq(irq_map[virq].hwirq);

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
}

/* When an irq gets requested for the first client, if it's an
 * edge interrupt, we clear any previous one on the controller
 */
static unsigned int m2_startup_irq(unsigned int virq)
{
#ifdef USE_SPINLOCK
	unsigned long flags;
#endif
	unsigned int src = irq_map[virq].hwirq;

#ifdef DEBUG
	printk("|SU%d", src);
#endif
#ifdef USE_SPINLOCK
	spin_lock_irqsave(&m2_lock, flags);
#endif

        if ((irq_to_desc(virq)->status & IRQ_LEVEL) == 0)
		__m2_ack_irq(src);
        __set_bit(src, ppc_cached_irq_mask);
        __m2_set_irq_mask(src, 0);

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif

        return 0;
}

static void
m2_disable_irq(unsigned int virq)
{
#ifdef USE_SPINLOCK
	unsigned long flags;
#endif
        unsigned int src = irq_map[virq].hwirq;

#ifdef DEBUG
	printk("|D%d", src);
#endif

#ifdef USE_SPINLOCK
	spin_lock_irqsave(&m2_lock, flags);
#endif

	__clear_bit(src, ppc_cached_irq_mask);
	__m2_set_irq_mask(src, 1);

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
}

static void
m2_enable_irq(unsigned int virq)
{
#ifdef USE_SPINLOCK
	unsigned long flags;
#endif
        unsigned int src = irq_map[virq].hwirq;
#if 0
	unsigned int irq = 0;
#endif

	if (irq_to_desc(virq)->status & IRQ_REPLAY)
		return;
#ifdef DEBUG
	printk("|E%d", src);
#endif

	if (deactivated_slot_interrupt_pending == 2) {
		deactivated_slot_interrupt_pending = 0;
		return;
	}

#ifdef USE_SPINLOCK
	spin_lock_irqsave(&m2_lock, flags);
#endif

	__set_bit(src, ppc_cached_irq_mask);
	__m2_set_irq_mask(src, 0);

#if 0
	icr_pen = in_8(m2_irq.icr);
	__m2_check_irq(icr_pen, 0);
	__m2_check_irq(icr_pen, 1);
	icr_pen &= 0x87;

	if ((icr_pen & 0x8) || (via2_pen & ~CA1_INT) || slot_pen) {
		u8 i, bit;

		for (i = 6, bit = 0x40; i >= 0; --i, bit >>= 1) {
			if (via2_pen & bit) {
				irq = 16 + i;
				break;
			}
		}
		if (irq == 17) {
			for (i = 6, bit = 0x40; i >= 0; --i, bit >>= 1) {
				if (slot_pen & bit) {
					irq = 24 + i;
					break;
				}
			}
		}
		printk(KERN_DEBUG "m2_enable_irq: IRQ %d pending after enabling interrupt %d (icr_pen=0x%x via2_pen=0x%x slot_pen=0x%x)\n", irq, src, icr_pen, via2_pen, slot_pen);
	}
#endif

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif

#if 0
	if (irq) {
		irq_to_desc(virq)->status |= IRQ_PENDING;
		check_irq_resend(irq_to_desc(virq), irq_linear_revmap(pmac_m2_host, irq));
	}
#endif
}

static void
m2_disable_and_ack_irq(unsigned int virq)
{
#ifdef USE_SPINLOCK
	unsigned long flags;
#endif
        unsigned int src = irq_map[virq].hwirq;

#ifdef DEBUG
	printk("|DA%d", src);
#endif

	if (deactivated_slot_interrupt_pending == 1) {
		deactivated_slot_interrupt_pending = 2;
		return;
	}

#ifdef USE_SPINLOCK
	spin_lock_irqsave(&m2_lock, flags);
#endif

	__clear_bit(src, ppc_cached_irq_mask);
        if (__test_and_clear_bit(src, ppc_lost_interrupts))
                atomic_dec(&ppc_n_lost_interrupts);
	__m2_set_irq_mask(src, 0);
	__m2_ack_irq(src);

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
}

static int m2_retrigger(unsigned int virq)
{
#ifdef DEBUG
	unsigned int src = irq_map[virq].hwirq;

	printk("|R%d", src);
#endif
 
        return 0;	// We cannot retrigger IRQs in hardware
}

static struct irq_chip pmac_m2 = {
	.typename	= " M2" ,
	.startup	= m2_startup_irq,
	.shutdown	= m2_disable_irq,
	.enable		= m2_enable_irq,
	.disable	= m2_disable_irq,
	.unmask		= m2_enable_irq,
	.mask		= m2_disable_irq,
	.ack		= m2_ack_irq,
	.mask_ack	= m2_disable_and_ack_irq,
	.retrigger	= m2_retrigger
};

static int pmac_m2_host_match(struct irq_host *h, struct device_node *node)
{
        /* We match all, we don't always have a node anyway */
        return 1;
}
                
static int pmac_m2_host_map(struct irq_host *h, unsigned int virq,
                             irq_hw_number_t hw)
{
	int level = (/*(hw >= 24) || */(hw <= 7))? 0 : 1;
//	int level = 1;
//	int level = 0;

        if (hw >= max_irqs)
                return -EINVAL;  

        /* Mark level interrupts, set delayed disable for edge ones and set
         * handlers
         */
	if (level)
		irq_to_desc(virq)->status |= IRQ_LEVEL;
	set_irq_chip_and_handler(virq, &pmac_m2, level ?
				handle_level_irq : handle_edge_irq);
	return 0;
}

static int pmac_m2_host_xlate(struct irq_host *h, struct device_node *ct,
                               const u32 *intspec, unsigned int intsize,
                               irq_hw_number_t *out_hwirq,
                               unsigned int *out_flags)

{
        *out_flags = IRQ_TYPE_NONE;
        *out_hwirq = *intspec;
        return 0;
}

static struct irq_host_ops pmac_m2_host_ops = {
        .match = pmac_m2_host_match,
        .map = pmac_m2_host_map,
        .xlate = pmac_m2_host_xlate,
};

static inline void
__m2_check_irq(u8 bits, s8 mask)
{
	if (!mask) {
#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
		via1_pen |= in_8(&m2_irq.via1[IFR]);
#endif
		via2_pen |= in_8(&m2_irq.via2[IFR]);
		slot_pen |= in_8(&m2_irq.via2[ANH]);
#ifdef HANDLE_BABOON
		if (!is_PB500PPC) {
			baboon_pen |= in_8(m2_irq.baboon_flag);
		}
#endif
	} else {
#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
		via1_pen = M2_MASK(via1_pen) & in_8(&m2_irq.via1[IER]);
#endif
		via2_pen = M2_MASK(via2_pen) & in_8(&m2_irq.via2[IER]);
		slot_pen = M2_MASK(~slot_pen);
#ifdef HANDLE_BABOON
		if (!is_PB500PPC)
			baboon_pen = baboon_pen/* & baboon_ena*/ & BABOON_MASK;
#endif
#if 0
		if ((via1_pen & M2_IRQ_PENDING) && !(icr_pen & 0x7)) {
			icr_pen &= ~0x7;
			icr_pen |= 1;
			via1_pen &= ~M2_IRQ_PENDING;
		}
		if ((via2_pen & M2_IRQ_PENDING) && !(icr_pen & 0x7)) {
			icr_pen &= ~0x7;
			icr_pen |= 2;
			via2_pen &= ~M2_IRQ_PENDING;
		}
#endif

		/* NMI is autovector interrupt level 7 */
		if ((icr_pen & 0x7) == 7) {
			if (!is_PB500PPC)
				printk(KERN_INFO "\nicr_pen: 0x%02X, *icr: 0x%02X, icr_ena: 0x%02X\nvia1_pen: 0x%02X, *via1_pen: 0x%02X, via1_ena: 0x%02X, *via1_ena: 0x%02X\nvia2_pen: 0x%02X, *via2_pen: 0x%02X, via2_ena: 0x%02X, *via2_ena: 0x%02X\nslot_pen: 0x%02X, *slot_pen: 0x%02X, slot_ena: 0x%02X\nbaboon_pen: 0x%02X, *baboon_pen: 0x%02X, baboon_ena: 0x%02X\n", icr_pen, in_8(m2_irq.icr), icr_ena, via1_pen, in_8(&m2_irq.via1[IFR]), via1_ena, in_8(&m2_irq.via1[IER]), via2_pen, in_8(&m2_irq.via2[IFR]), via2_ena, in_8(&m2_irq.via2[IER]), slot_pen, in_8(&m2_irq.via2[ANH]), slot_ena, baboon_pen, in_8(m2_irq.baboon_flag), baboon_ena);
			else
				printk(KERN_INFO "\nicr_pen: 0x%02X, *icr: 0x%02X, icr_ena: 0x%02X\nvia1_pen: 0x%02X, *via1_pen: 0x%02X, via1_ena: 0x%02X, *via1_ena: 0x%02X\nvia2_pen: 0x%02X, *via2_pen: 0x%02X, via2_ena: 0x%02X, *via2_ena: 0x%02X\nslot_pen: 0x%02X, *slot_pen: 0x%02X, slot_ena: 0x%02X\n", icr_pen, in_8(m2_irq.icr), icr_ena, via1_pen, in_8(&m2_irq.via1[IFR]), via1_ena, in_8(&m2_irq.via1[IER]), via2_pen, in_8(&m2_irq.via2[IFR]), via2_ena, in_8(&m2_irq.via2[IER]), slot_pen, in_8(&m2_irq.via2[ANH]), slot_ena);

			printk(KERN_DEBUG "ADB interrupt will be set to pending...\n");
			via1_pen |= CB1_INT;
			icr_pen &= ~0x7;
			icr_pen |= 7;
		}

	}
}

static unsigned int
m2_get_irq(void)
{
	unsigned int counter = 0;
	u8 bit, recheck = 0;
	u8 i;
	int irq;
#ifdef USE_SPINLOCK
	unsigned long flags;
#endif

#ifdef USE_SPINLOCK
	spin_lock_irqsave(&m2_lock, flags);
#endif

again:
	icr_pen = in_8(m2_irq.icr);
	for(counter = 0 ;((icr_pen & 0x87) == 0) && (counter < 0xFFFFFFFF) && (m2_irq_got == 0); counter++) {
//		mb();
		ndelay(30);
		icr_pen = in_8(m2_irq.icr);
	}
	if (counter > 0)
		printk("|%d|", counter);

	if (icr_pen & M2_IRQ_PENDING) {
		out_8(m2_irq.icr, M2_ACK);
	}

	if (m2_irq_got == 1000) {
		printk(KERN_ERR "aborting m2_get_irq\n");
		if (!is_PB500PPC)
			printk(KERN_DEBUG "\nicr_pen: 0x%02X, *icr: 0x%02X, icr_ena: 0x%02X\nvia1_pen: 0x%02X, *via1_pen: 0x%02X, via1_ena: 0x%02X, *via1_ena: 0x%02X\nvia2_pen: 0x%02X, *via2_pen: 0x%02X, via2_ena: 0x%02X, *via2_ena: 0x%02X\nslot_pen: 0x%02X, *slot_pen: 0x%02X, slot_ena: 0x%02X\nbaboon_pen: 0x%02X, *baboon_pen: 0x%02X, baboon_ena: 0x%02X\n", icr_pen, in_8(m2_irq.icr), icr_ena, via1_pen, in_8(&m2_irq.via1[IFR]), via1_ena, in_8(&m2_irq.via1[IER]), via2_pen, in_8(&m2_irq.via2[IFR]), via2_ena, in_8(&m2_irq.via2[IER]), slot_pen, in_8(&m2_irq.via2[ANH]), slot_ena, baboon_pen, in_8(m2_irq.baboon_flag), baboon_ena);
		else
			printk(KERN_DEBUG "\nicr_pen: 0x%02X, *icr: 0x%02X, icr_ena: 0x%02X\nvia1_pen: 0x%02X, *via1_pen: 0x%02X, via1_ena: 0x%02X, *via1_ena: 0x%02X\nvia2_pen: 0x%02X, *via2_pen: 0x%02X, via2_ena: 0x%02X, *via2_ena: 0x%02X\nslot_pen: 0x%02X, *slot_pen: 0x%02X, slot_ena: 0x%02X\n", icr_pen, in_8(m2_irq.icr), icr_ena, via1_pen, in_8(&m2_irq.via1[IFR]), via1_ena, in_8(&m2_irq.via1[IER]), via2_pen, in_8(&m2_irq.via2[IFR]), via2_ena, in_8(&m2_irq.via2[IER]), slot_pen, in_8(&m2_irq.via2[ANH]), slot_ena);
		m2_irq_got = 0;
//		out_8(m2_irq.icr, 0x40);
		last_irq = -1;
#ifdef USE_SPINLOCK
		spin_unlock_irqrestore(&m2_lock, flags);
#endif
		return NO_IRQ;
	}
	m2_irq_got++;

	__m2_check_irq(icr_pen, 0);
	__m2_check_irq(icr_pen, 1);

again_slot:
	irq = icr_pen & 0x7;

#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
	if (irq == 1) {
		for (i = 0, bit = 0x01; i <= 6; ++i, bit <<= 1) {
			if (via1_pen & bit) {
				irq = 8 + i;
				break;
			}
		}
	}
	else
#endif
	if (irq == 2) {
		for (i = 6, bit = 0x40; i >= 0; --i, bit >>= 1) {
			if (via2_pen & bit) {
				irq = 16 + i;
				break;
			}
		}
		if (irq == 17) {
			for (i = 6, bit = 0x40; i >= 0; --i, bit >>= 1) {
				if (slot_pen & bit) {
					irq = 24 + i;
					break;
				}
			}
#ifdef HANDLE_BABOON
			if (irq == IRQ_BABOON) {
				for (i = 2, bit = 0x4; i >= 0; --i, bit >>= 1) {
					if (baboon_pen & bit) {
						irq = 32 + i;
						break;
					}
				}
			}
#endif
		}
	}

	switch (irq) {
		case 17:
			if (slot_pen && ((slot_pen & slot_ena) == 0)) {
				printk(KERN_ERR "disabled slot interrupt pending\n");
				printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
				recheck = 0;
				irq = -1;
			} else
				recheck = 1;
			break;
		case 2:
			printk(KERN_ERR "unknown via2 interrupt pending\n");
			printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
			recheck = 0;
			break;
		case 1:
#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
			printk(KERN_ERR "unknown via1 interrupt pending\n");
			printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
#endif
			recheck = 0;
			break;

		case 6:
		case 5:
		case 3:
			printk(KERN_DEBUG "unknown autovector IRQ level: %d\n", irq);
		case 0:
			if (m2_irq_got == 1)
				printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
			irq = -1;

			recheck = 0;
//			break;
		case -1:
			if (m2_irq_got == 1) {
				if((slot_pen & slot_ena) && (via2_pen |= in_8(&m2_irq.via2[IFR])) & CA1_INT) {
					printk(KERN_DEBUG "m2_irq_get: enabled slot interrupt pending while VIA2 CA1_INT disabled\n");
					printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
					icr_pen &= ~0x7;
					icr_pen |= 2;
					deactivated_slot_interrupt_pending = 1;
					goto again_slot;
				}
#ifdef HANDLE_BABOON
				if((baboon_pen & baboon_ena) && ((slot_pen |= ~in_8(&m2_irq.via2[ANH])) & SLOT_BABOON) && ((via2_pen |= in_8(&m2_irq.via2[IFR])) & CA1_INT)) {
					printk(KERN_DEBUG "m2_irq_get: enabled baboon interrupt pending while VIA2 CA1_INT disabled\n");
					printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
					icr_pen &= ~0x7;
					icr_pen |= 2;
					deactivated_slot_interrupt_pending = 1;
					goto again_slot;
				}
#endif
			}
			recheck = 0;

			break;
		default:
#ifdef HANDLE_BABOON
			if (irq == IRQ_BABOON) {
				if (baboon_pen && ((baboon_pen & baboon_ena) == 0)) {
					printk(KERN_ERR "disabled baboon interrupt pending\n");
					printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
					recheck = 0;
					irq = -1;
				}
				else
					recheck = 1;
				break;
			} else {
#endif
				recheck = 0;
				break;
#ifdef HANDLE_BABOON
			}
#endif
	}
	if (recheck) {
		printk(KERN_DEBUG "checking interrupts again (%d)\n", m2_irq_got);
		printk(KERN_DEBUG "autovector IRQ level: %d\n", irq);
		recheck = 0;
		goto again;
	}

	last_irq = irq;

	if (unlikely(irq < 0)) {
#ifdef USE_SPINLOCK
		spin_unlock_irqrestore(&m2_lock, flags);
#endif
		return NO_IRQ;
	} else {
		via1_pen = via2_pen = 0;
		slot_pen = baboon_pen = 0;
//		if (last_irq != irq)
			m2_irq_got = 0;
	}

//	virq = irq_linear_revmap(pmac_m2_host, irq);
//	printk("IRQ: %d -> %d\n", irq, virq);

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
//	return virq;
	return irq_linear_revmap(pmac_m2_host, irq);
}

#ifdef CONFIG_XMON
static struct irqaction xmon_action = {
	.handler	= xmon_irq,
	.flags		= 0,
	.name		= "NMI - XMON"
};
#endif

void __init
pmac_m2_init(void)
{
        unsigned int flags = 0;
	struct device_node *irqctrler, *other;
	const u32 *reg;
	unsigned long addr, size;
	u32 __iomem *sonic = NULL;

	irqctrler = of_find_node_by_name(NULL, "m2");
	if (irqctrler) {
		reg = of_get_property(irqctrler, "reg", NULL);
		addr = reg[0];
		size = reg[1];
		m2_irq.icr = (u8 __iomem *)ioremap(addr, size);
		addr = reg[2];
		size = reg[3];
		m2_irq.via1 = (u8 __iomem *)ioremap(addr, size);
		addr = reg[4];
		size = reg[5];
		m2_irq.via2 = (u8 __iomem *)ioremap(addr, size);
#ifdef HANDLE_BABOON
		reg = of_get_property(irqctrler, "AAPL,interrupts", NULL);
		IRQ_BABOON = (u8)reg[4];
#endif
	} else {
//		panic("Failed to obtain M2 irq addresses/interrupts\n");
		printk(KERN_DEBUG "pmac_m2_init: Failed to obtain M2 irq addresses/interrupts\n");
	}

	other = of_find_node_by_name(NULL, "baboon");
	if (other) {
		reg = of_get_property(other, "reg", NULL);
		addr = reg[4];
		size = reg[5];
		m2_irq.baboon_flag = (u8 __iomem *)
				ioremap(addr + 0x9, size);
		of_node_put(other);
		printk(KERN_DEBUG "pmac_m2_init: found baboon\n");
	} else {
		printk(KERN_DEBUG "pmac_m2_init: no baboon in device tree. Assuming PB500\n");
		is_PB500PPC = 1;
		baboon_pen = 0;
		sonic = (u32 __iomem *)ioremap((phys_addr_t)0x50f0a000, 0x100);
		out_be32(sonic, 0x7fffL);
		out_be32(sonic, 0L);
	}

        /* We configure the OF parsing based on our
         * platform type and wether we were booted by BootX.
         */
	flags |= OF_IMAP_OLDWORLD_MAC;
        if (of_get_property(of_chosen, "linux,bootx", NULL) != NULL)
                flags |= OF_IMAP_NO_PHANDLE;

        of_irq_map_init(flags);

#ifdef USE_SPINLOCK
	spin_lock_init(&m2_lock);
#endif

	ppc_md.get_irq = m2_get_irq;

	pmac_m2_host = irq_alloc_host(irqctrler, IRQ_HOST_MAP_LINEAR, max_irqs,
                                       &pmac_m2_host_ops,
                                       max_irqs);
	BUG_ON(pmac_m2_host == NULL);
        irq_set_default_host(pmac_m2_host);

	of_node_put(irqctrler);

	/* disable all interrupts */
	out_8(m2_irq.icr, M2_ACK);
#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
	out_8(&m2_irq.via1[IER], 0x7f);
	out_8(&m2_irq.via1[IFR], 0x7f);
#endif
	out_8(&m2_irq.via2[IER], 0x7f);
	out_8(&m2_irq.via2[IFR], 0x7f);
	out_8(&m2_irq.via2[ANH], 0x7f);

	out_8(&m2_irq.via1[ACR], (in_8(&m2_irq.via1[ACR]) & 0x1F));
	out_8(&m2_irq.via1[T1LL], 0xFF);
	out_8(&m2_irq.via1[T1LH], 0xFF);
	out_8(&m2_irq.via1[T1CL], 0xFF);
	out_8(&m2_irq.via1[T1CH], 0xFF);
	out_8(&m2_irq.via1[T2CL], 0xFF);
	out_8(&m2_irq.via1[T2CH], 0xFF);
	(void)in_8(&m2_irq.via1[T1CL]);
	(void)in_8(&m2_irq.via1[T2CL]);
#ifdef CONFIG_ADB_PMU_NBPMAC_ALT
	out_8(&m2_irq.via1[IFR], 0x7f);
#endif

	if (!is_PB500PPC)
	{
		out_8(m2_irq.baboon_flag, 0);
	}

	icr_ena = via1_ena = via2_ena = slot_ena = baboon_ena = 0;

	printk(KERN_DEBUG "System has %d possible interrupts\n", max_irqs);

#ifdef CONFIG_XMON
	printk(KERN_DEBUG "pmac_m2_init: installing NMI IRQ handler for XMON\n");
	setup_irq(irq_create_mapping(NULL, 7), &xmon_action);
#endif	/* CONFIG_XMON */
}

#if 0
/*
 * These procedures are used in implementing sleep on the powerbooks.
 * pmac_sleep_save_intrs() stops irq polling.
 * sleep_restore_intrs() restores irq polling.
 */

void
pmac_sleep_save_intrs(int viaint)
{
#ifdef USE_SPINLOCK
	unsigned long flags;

	spin_lock_irqsave(&m2_lock, flags);
#endif

	out_8(&m2_irq.via1[IER], ~(CB1_INT | SR_INT | IER_SET));
	out_8(&m2_irq.via2[IER], 0x7f);

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
}

void
pmac_sleep_restore_intrs(void)
{
#ifdef USE_SPINLOCK
	unsigned long flags;

	spin_lock_irqsave(&m2_lock, flags);
#endif

	out_8(&m2_irq.via1[IER], via1_ena);
	out_8(&m2_irq.via2[IER], via2_ena);

#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
}

void m2_poll_start(void)
{
#ifdef USE_SPINLOCK
	unsigned long flags;

	spin_lock_irqsave(&m2_lock, flags);
#endif
	m2_timer.expires = M2_TIMER;
	add_timer(&m2_timer);
#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
}

void m2_poll_stop(void)
{
#ifdef USE_SPINLOCK
	unsigned long flags;

	spin_lock_irqsave(&m2_lock, flags);
#endif
	del_timer(&m2_timer);
#ifdef USE_SPINLOCK
	spin_unlock_irqrestore(&m2_lock, flags);
#endif
}

EXPORT_SYMBOL(m2_poll_start);
EXPORT_SYMBOL(m2_poll_stop);
#endif
