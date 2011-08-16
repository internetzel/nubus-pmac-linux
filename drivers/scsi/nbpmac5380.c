/*
 * Copied over from mac_scsi.c and hacked a bit.
 * The following is the original copyright notice.
 */
/*
 * Generic Macintosh NCR5380 driver
 *
 * Copyright 1998, Michael Schmitz <mschmitz@lbl.gov>
 *
 * derived in part from:
 */
/*
 * Generic Generic NCR5380 driver
 *
 * Copyright 1995, Russell King
 *
 * ALPHA RELEASE 1.
 *
 * For more information, please consult
 *
 * NCR 5380 Family
 * SCSI Protocol Controller
 * Databook
 *
 * NCR Microelectronics
 * 1635 Aeroplaza Drive
 * Colorado Springs, CO 80916
 * 1+ (719) 578-3400
 * 1+ (800) 334-5454
 */

/*
 * $Log: mac_NCR5380.c,v $
 */

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/ctype.h>
#include <linux/delay.h>

#include <linux/module.h>
#include <linux/signal.h>
//#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/adb.h>
#include <linux/pmu.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/prom.h>
#include <asm/nubus.h>

#if 1
//#define NDEBUG (NDEBUG_ANY)
#define NDEBUG (NDEBUG_INTR | NDEBUG_INFORMATION | NDEBUG_PSEUDO_DMA | NDEBUG_DMA | NDEBUG_PIO | NDEBUG_ARBITRATION | NDEBUG_SELECTION | NDEBUG_RESELECTION)
#else
#define NDEBUG (NDEBUG_ABORT)
#endif
#define NDEBUG_ABORT	0x800000
#define NDEBUG_TAGS	0x1000000
#define NDEBUG_MERGING	0x2000000

#include "scsi.h"
//#include "hosts.h"
#include "nbpmac5380.h"
//#include "constants.h"

/* These control the behaviour of the generic 5380 core */
#define AUTOSENSE
#define PSEUDO_DMA
#define SUPPORT_TAGS

#include "NCR5380.h"

#define RESET_BOOT
//#define DRIVER_SETUP

//#define DRQ_INTERRUPT

#ifdef RESET_BOOT
static void mac_scsi_reset_boot(struct Scsi_Host *instance);
#endif

static int setup_can_queue = -1;
static int setup_cmd_per_lun = -1;
static int setup_sg_tablesize = -1;
#ifdef PSEUDO_DMA
static int setup_use_pdma = 1;
#else
static int setup_use_pdma = -1;
#endif
#ifdef SUPPORT_TAGS
static int setup_use_tagged_queuing = -1;
#endif
static int setup_hostid = -1;

#if 0
static int mac_read_overruns = 0;
static int mac_pdma_residual = 0;
#endif

#ifdef DRQ_INTERRUPT
static void macscsi_dma_intr(int irq, void *dev_id, struct pt_regs *regs);
#endif

extern void m2_poll_start(void);
extern void m2_poll_stop(void);

/* Time (in jiffies) to wait after a reset; the SCSI standard calls for 250ms,
 * we usually do 0.5s to be on the safe side. But Toshiba CD-ROMs once more
 * need ten times the standard value... */
//#define TOSHIBA_DELAY

#ifdef TOSHIBA_DELAY
#define	AFTER_RESET_DELAY	(5*HZ/2)
#else
#define	AFTER_RESET_DELAY	(HZ/2)
#endif

static volatile unsigned char *mac_scsi_regp = NULL;
static volatile unsigned char *mac_scsi_drq  = NULL;
static volatile unsigned char *mac_scsi_nodrq = NULL;

/*
 * NCR 5380 register access functions
 */

#if 0
/* Debug versions */
#define CTRL(p,v) (*ctrl = (v))

static char macscsi_read(struct Scsi_Host *instance, int reg)
{
  int iobase = instance->io_port;
  int i;
  int *ctrl = &((struct NCR5380_hostdata *)instance->hostdata)->ctrl;

  CTRL(iobase, 0);
  i = in_8((void *)(iobase + (reg<<4)));
  CTRL(iobase, 0x40);

  return i;
}

static void macscsi_write(struct Scsi_Host *instance, int reg, int value)
{
  int iobase = instance->io_port;
  int *ctrl = &((struct NCR5380_hostdata *)instance->hostdata)->ctrl;

  CTRL(iobase, 0);
  out_8((void *)(iobase + (reg<<4)), value);
  CTRL(iobase, 0x40);
}
#else

/* Fast versions */
static __inline__ char macscsi_read(struct Scsi_Host *instance, int reg)
{
  return in_8((void *)instance->io_port + (reg<<4));
}

static __inline__ void macscsi_write(struct Scsi_Host *instance, int reg, int value)
{
  out_8((void *)instance->io_port + (reg<<4), value);
}
#endif

/*
 * Function : mac_scsi_setup(char *str)
 *
 * Purpose : booter command line initialization of the overrides array,
 *
 * Inputs : str - comma delimited list of options
 *
 */

#ifdef DRIVER_SETUP
static int setup_called = 0;
	
static int __init mac_scsi_setup(char *str) {
	int ints[7];
	
	(void)get_options( str, ARRAY_SIZE(ints), ints);
	
	if (setup_called++ || ints[0] < 1 || ints[0] > 6) {
	    printk(KERN_WARNING "scsi: <mac5380>"
		" Usage: mac5380=<can_queue>[,<cmd_per_lun>,<sg_tablesize>,<hostid>,<use_tags>,<use_pdma>]\n");
	    printk(KERN_ALERT "scsi: <mac5380> Bad Penguin parameters?\n");
	    return 0;
	}
	    
	if (ints[0] >= 1) {
		if (ints[1] > 0)
			/* no limits on this, just > 0 */
			setup_can_queue = ints[1];
	}
	if (ints[0] >= 2) {
		if (ints[2] > 0)
			setup_cmd_per_lun = ints[2];
	}
	if (ints[0] >= 3) {
		if (ints[3] >= 0) {
			setup_sg_tablesize = ints[3];
			/* Must be <= SG_ALL (255) */
			if (setup_sg_tablesize > SG_ALL)
				setup_sg_tablesize = SG_ALL;
		}
	}
	if (ints[0] >= 4) {
		/* Must be between 0 and 7 */
		if (ints[4] >= 0 && ints[4] <= 7)
			setup_hostid = ints[4];
		else if (ints[4] > 7)
			printk(KERN_WARNING "mac_scsi_setup: invalid host ID %d !\n", ints[4] );
	}
#ifdef SUPPORT_TAGS	
	if (ints[0] >= 5) {
		if (ints[5] >= 0)
			setup_use_tagged_queuing = !!ints[5];
	}
	
	if (ints[0] == 6) {
	    if (ints[6] >= 0)
		setup_use_pdma = ints[6];
	}
#else
	if (ints[0] == 5) {
	    if (ints[5] >= 0)
		setup_use_pdma = ints[5];
	}
#endif /* SUPPORT_TAGS */
	
	return 1;
}

__setup("nbpmac5380=", mac_scsi_setup);
#endif /* DRIVER_SETUP */


/*
 * If you want to find the instance with (k)gdb ...
 */
#if NDEBUG
static struct Scsi_Host *default_instance;
#endif

/*
 * Function : int nbpmacscsi_detect(struct scsi_host_template * tpnt)
 *
 * Purpose : initializes mac NCR5380 driver based on the
 *	command line / compile time port and irq definitions.
 *
 * Inputs : tpnt - template for this SCSI adapter.
 *
 * Returns : 1 if a host adapter was found, 0 if not.
 *
 */
 
static int called = 0;
#ifdef DRQ_INTERRUPT
static int drq = 0;
#endif

int nbpmacscsi_detect(struct scsi_host_template * tpnt)
{
    int flags = 0;
	struct Scsi_Host *instance;
	struct device_node *dp;
	const u32 *reg;

    if (called)
	return( 0 );

//	dp = find_devices("5380");
	dp = of_find_node_by_name(NULL, "5380");
	if (!dp)
		return 0;
	reg = of_get_property(dp, "reg", NULL);
        if (reg == NULL) {
                printk(KERN_ERR "nbpmac5380: No \"reg\" property !\n");
		of_node_put(dp);
                return 0;
        }

	tpnt->proc_name = "nbpmac5380";

	/* setup variables */
	tpnt->can_queue = (setup_can_queue > 0) ? setup_can_queue : CAN_QUEUE;
	tpnt->cmd_per_lun = (setup_cmd_per_lun > 0) ? setup_cmd_per_lun : CMD_PER_LUN;
	tpnt->sg_tablesize = (setup_sg_tablesize >= 0) ? setup_sg_tablesize : SG_TABLESIZE;

	if (setup_hostid >= 0)
		tpnt->this_id = setup_hostid;
	else
		/* use 7 as default */
		tpnt->this_id = 7;

#ifdef SUPPORT_TAGS
	if (setup_use_tagged_queuing < 0)
		setup_use_tagged_queuing = USE_TAGGED_QUEUING;
#endif

	/* Once we support multiple 5380s (e.g. DuoDock) we'll do
		something different here */
	instance = scsi_register (tpnt, sizeof(struct NCR5380_hostdata));

#if NDEBUG
    default_instance = instance;
#endif
	/* mac68k source says multiple device support is broken */

	mac_scsi_regp  = (volatile unsigned char *)
			 of_iomap(dp, 0);
	mac_scsi_drq   = (volatile unsigned char *)
			 of_iomap(dp, 1);
	mac_scsi_nodrq = (volatile unsigned char *)
			 of_iomap(dp, 2);

    if (! setup_use_pdma)
	flags = FLAG_NO_PSEUDO_DMA;

	instance->io_port = (unsigned long) mac_scsi_regp;
	instance->irq = irq_of_parse_and_map(dp, 0);

	instance->n_io_port = 255;

	((struct NCR5380_hostdata *)instance->hostdata)->ctrl = 0;

	if (instance->irq != SCSI_IRQ_NONE)
		if (request_irq(instance->irq, NCR5380_intr, 0, "ncr5380", instance)) {
			printk("scsi%d : IRQ%d not free, interrupts disabled\n",
				instance->host_no, instance->irq);
			instance->irq = SCSI_IRQ_NONE;
		}

#ifdef DRQ_INTERRUPT
	drq = irq_of_parse_and_map(vias, 1);
	request_irq(drq, macscsi_dma_intr, 0, "ncr5380 DRQ", instance);
#endif
	of_node_put(dp);

#ifdef RESET_BOOT
	mac_scsi_reset_boot(instance);
#endif

	flags |= FLAG_HAS_LAST_BYTE_SENT;


    NCR5380_init(instance, flags);

    printk(KERN_INFO "scsi%d : generic 5380 at port %lX irq", instance->host_no, instance->io_port);
    if (instance->irq == SCSI_IRQ_NONE)
	printk (KERN_INFO "s disabled");
    else
	printk (KERN_INFO " %d", instance->irq);
    printk(KERN_INFO " options CAN_QUEUE=%d CMD_PER_LUN=%d release=%d",
	   instance->can_queue, instance->cmd_per_lun, MACSCSI_PUBLIC_RELEASE);
    printk(KERN_INFO "\nscsi%d :", instance->host_no);
    NCR5380_print_options(instance);
    printk("\n");
    called = 1;
	return 1;
}

int nbpmacscsi_release (struct Scsi_Host *shpnt)
{
	if (shpnt->irq != SCSI_IRQ_NONE)
		free_irq (shpnt->irq, shpnt);
#ifdef DRQ_INTERRUPT
	free_irq (drq, shpnt);
#endif
	NCR5380_exit(shpnt);
	scsi_unregister(shpnt);


	return 0;
}

#ifdef RESET_BOOT
/*
 * Our 'bus reset on boot' function
 */

static void mac_scsi_reset_boot(struct Scsi_Host *instance)
{
	unsigned long end;

	NCR5380_local_declare();
	NCR5380_setup(instance);
	
	/*
	 * Do a SCSI reset to clean up the bus during initialization. No messing
	 * with the queues, interrupts, or locks necessary here.
	 */

	printk(KERN_INFO "scsi%d : NCR5380 on PowerBook ", instance->host_no);
	printk("resetting the SCSI bus...");

	/* switch off SCSI IRQ - catch an interrupt without IRQ bit set else */
       	disable_irq(instance->irq);

	/* get in phase */
	NCR5380_write( TARGET_COMMAND_REG,
		      PHASE_SR_TO_TCR( NCR5380_read(STATUS_REG) ));

	/* assert RST */
	NCR5380_write( INITIATOR_COMMAND_REG, ICR_BASE | ICR_ASSERT_RST );
	/* The min. reset hold time is 25us, so 40us should be enough */
	udelay( 50 );
	/* reset RST and interrupt */
	NCR5380_write( INITIATOR_COMMAND_REG, ICR_BASE );
	NCR5380_read( RESET_PARITY_INTERRUPT_REG );

	for( end = jiffies + AFTER_RESET_DELAY; time_before(jiffies, end); )
		barrier();

	/* switch on SCSI IRQ again */
       	enable_irq(instance->irq);

	printk(" done\n");
}
#endif

const char * nbpmacscsi_info (struct Scsi_Host *spnt)
{
    return "";
}

#if 0
/*
 * pseudo-DMA transfer functions, copied and modified from Russel King's
 * ARM 5380 driver (cumana_1)
 *
 * Work in progress (sort of), didn't work last time I checked, don't use!
 */

#define CTRL(p,v) 	(*ctrl = (v))
#define STAT(p)		(in_8((void *)(&p[1<<4])))
#define IN(p)		IN2(p)
#define IN2(p)		in_be16((void *)(p))
#define OUT(v,p)	OUT2(v,p)
#define OUT2(v,p)	out_be16((void *)(p), (v))
#define L(v)		(((v)<<16)|((v) & 0x0000ffff))
#define H(v)		(((v)>>16)|((v) & 0xffff0000))
#define ioaddr(v)	(v)

static char pdma_write = 0;
static int pdma_read_residue = 0;
static int pdma_write_residue = 0;
static unsigned char *pdma_dst = NULL;
static unsigned char *pdma_src = NULL;
static unsigned int pdma_read_count = 0;
static unsigned int pdma_write_count = 0;

unsigned long int counter = 0;
unsigned int average = 0;
unsigned long int cycle_counter = 0;

static __inline__ int macscsi_pwrite(struct Scsi_Host *instance, unsigned char *addr,
              int len)
{
  volatile unsigned char *dma_io = mac_scsi_drq;

   NCR5380_local_declare();
   NCR5380_setup(instance);

	len = pdma_write_count;

	dprintk(NDEBUG_PSEUDO_DMA, ("scsi%d : about to pwrite %d bytes...", instance->host_no, len));

  if(!len) return 0;

	counter = 0;
   while (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ) 
         && (!(NCR5380_read(STATUS_REG) & SR_REQ) 
            || (NCR5380_read(BUS_AND_STATUS_REG) & BASR_PHASE_MATCH))) {
		counter++;
	}
	if (counter)
		printk("-%d-", (unsigned int)counter);
	counter = 0;
   if (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ)) {
      printk(KERN_ERR "Error in macscsi_pwrite\n");
	pdma_write_count = 0;
      return -1;
   }

//	m2_poll_start();
//	pmu_suspend();

	pdma_write_residue = len;

	dprintk(NDEBUG_PSEUDO_DMA, ("pwriting..."));

	cycle_counter = 0;
  while(pdma_write_residue >= 1)
  {
    int status;
#ifdef DRQ_INTERRUPT
	spin_lock_irq(&io_request_lock);
#endif
	if(pdma_write_residue == 0) {
#ifdef DRQ_INTERRUPT
		spin_unlock_irq(&io_request_lock);
#endif
		break;
	}
	cycle_counter++;
	counter = 0;
	while (!((status = NCR5380_read(BUS_AND_STATUS_REG)) & BASR_DRQ)) {
//		pmu_poll();
		counter++;
		if (!(status & BASR_PHASE_MATCH)) {
			(void) NCR5380_read(RESET_PARITY_INTERRUPT_REG);
			len = pdma_write_residue;
			pdma_write_residue = 0;
#ifdef DRQ_INTERRUPT
			spin_unlock_irq(&io_request_lock);
#endif
//			m2_poll_stop();
//			pmu_resume();
			return len;
		}
		if (!(NCR5380_read(MODE_REG) & MR_DMA_MODE)) {
			len = pdma_write_residue;
			pdma_write_residue = 0;
#ifdef DRQ_INTERRUPT
			spin_unlock_irq(&io_request_lock);
#endif
//			m2_poll_stop();
//			pmu_resume();
			return len;
		}
//		udelay(1);
	}

	out_8((void *)dma_io, *pdma_src++);
	pdma_write_residue -= 1;

	average += counter;
#ifdef DRQ_INTERRUPT
	spin_unlock_irq(&io_request_lock);
#endif
//	while (NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ);
  }
	average /= cycle_counter;
//	printk(";%d;\n", average);
	average = 0;

	dprintk(NDEBUG_PSEUDO_DMA, ("done\n"));

	len = pdma_write_residue;
	pdma_write_residue = 0;
//	m2_poll_stop();
//	pmu_resume();
	return len;
}

static __inline__ int macscsi_pread(struct Scsi_Host *instance, unsigned char *addr,
              int len)
{
  volatile unsigned char *dma_io = mac_scsi_drq + 0x60;

   NCR5380_local_declare();
   NCR5380_setup(instance);

	len = pdma_read_count;

	dprintk(NDEBUG_PSEUDO_DMA, ("scsi%d : about to pread %d bytes...", instance->host_no, len));

  if(!len) return 0;

	counter = 0;
	while (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ) 
		&& !(NCR5380_read(STATUS_REG) & SR_REQ)) {
		counter++;
	}
	if (counter)
		printk("_%d_", (unsigned int)counter);
	counter = 0;
	if (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ) 
		&& (NCR5380_read(BUS_AND_STATUS_REG) & BASR_PHASE_MATCH)) {
		printk(KERN_ERR "Error in macscsi_pread\n");
		pdma_read_count = 0;
		return -1;
	}

//	m2_poll_start();
//	pmu_suspend();

	pdma_read_residue = len;

	dprintk(NDEBUG_PSEUDO_DMA, ("preading: "));

	cycle_counter = 0;
  while(pdma_read_residue >= 1)
  {
    int status;
#ifdef DRQ_INTERRUPT
	spin_lock_irq(&io_request_lock);
#endif
	if(pdma_read_residue == 0) {
#ifdef DRQ_INTERRUPT
		spin_unlock_irq(&io_request_lock);
#endif
		break;
	}
	cycle_counter++;
	counter = 0;
	while (!((status = NCR5380_read(BUS_AND_STATUS_REG)) & BASR_DRQ)) {
//		pmu_poll();
		counter++;
		if (!(status & BASR_PHASE_MATCH)) {
			(void) NCR5380_read(RESET_PARITY_INTERRUPT_REG);
			len = pdma_read_residue;
			pdma_read_residue = 0;
#ifdef DRQ_INTERRUPT
			spin_unlock_irq(&io_request_lock);
#endif
//			m2_poll_stop();
//			pmu_resume();
			return len;
		}
		if (!(NCR5380_read(MODE_REG) & MR_DMA_MODE)) {
			len = pdma_read_residue;
			pdma_read_residue = 0;
#ifdef DRQ_INTERRUPT
			spin_unlock_irq(&io_request_lock);
#endif
//			m2_poll_stop();
//			pmu_resume();
			return len;
		}
//		udelay(1);
	}

	*pdma_dst++ = in_8((void *)dma_io);
	pdma_read_residue -= 1;

	average += counter;
#ifdef DRQ_INTERRUPT
	spin_unlock_irq(&io_request_lock);
#endif
//	while (NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ);
	}

	average /= cycle_counter;
//	printk(".%d.\n", average);
	average = 0;

	len = pdma_read_residue;
	pdma_read_residue = 0;
//	m2_poll_stop();
//	pmu_resume();
	return len;
}

#undef STAT
#undef CTRL
#undef IN
#undef OUT

static __inline__ int macscsi_pdma_residual(struct Scsi_Host *instance)
{
	if(!pdma_write) {
		pdma_read_count = 0;
		return pdma_read_residue;
	} else {
		pdma_write_count = 0;
		return pdma_write_residue;
	}
}

static __inline__ int macscsi_pdma_read_setup(struct Scsi_Host *instance, unsigned char *dst, unsigned int count)
{
	pdma_write = 0;
	pdma_dst = dst;
	pdma_read_count = count;
	return count;
}

static __inline__ int macscsi_pdma_write_setup(struct Scsi_Host *instance, unsigned char *src, unsigned int count)
{
	pdma_write = 1;
	pdma_src = src;
	pdma_write_count = count;
	return count;
}

#endif
#if 1
/* 
   Pseudo-DMA: (Ove Edlund)
   The code attempts to catch bus errors that occur if one for example
   "trips over the cable".
   XXX: Since bus errors in the PDMA routines never happen on my 
   computer, the bus error code is untested. 
   If the code works as intended, a bus error results in Pseudo-DMA 
   beeing disabled, meaning that the driver switches to slow handshake. 
   If bus errors are NOT extremely rare, this has to be changed. 
*/

#if 0
#define CP_IO_TO_MEM(s,d,len)				\
__asm__ __volatile__					\
    ("    cmp.w  #4,%2\n"				\
     "    bls    8f\n"					\
     "    move.w %1,%%d0\n"				\
     "    neg.b  %%d0\n"				\
     "    and.w  #3,%%d0\n"				\
     "    sub.w  %%d0,%2\n"				\
     "    bra    2f\n"					\
     " 1: move.b (%0),(%1)+\n"				\
     " 2: dbf    %%d0,1b\n"				\
     "    move.w %2,%%d0\n"				\
     "    lsr.w  #5,%%d0\n"				\
     "    bra    4f\n"					\
     " 3: move.l (%0),(%1)+\n"				\
     "31: move.l (%0),(%1)+\n"				\
     "32: move.l (%0),(%1)+\n"				\
     "33: move.l (%0),(%1)+\n"				\
     "34: move.l (%0),(%1)+\n"				\
     "35: move.l (%0),(%1)+\n"				\
     "36: move.l (%0),(%1)+\n"				\
     "37: move.l (%0),(%1)+\n"				\
     " 4: dbf    %%d0,3b\n"				\
     "    move.w %2,%%d0\n"				\
     "    lsr.w  #2,%%d0\n"				\
     "    and.w  #7,%%d0\n"				\
     "    bra    6f\n"					\
     " 5: move.l (%0),(%1)+\n"				\
     " 6: dbf    %%d0,5b\n"				\
     "    and.w  #3,%2\n"				\
     "    bra    8f\n"					\
     " 7: move.b (%0),(%1)+\n"				\
     " 8: dbf    %2,7b\n"				\
     "    moveq.l #0, %2\n"				\
     " 9: \n"						\
     ".section .fixup,\"ax\"\n"				\
     "    .even\n"					\
     "90: moveq.l #1, %2\n"				\
     "    jra 9b\n"					\
     ".previous\n"					\
     ".section __ex_table,\"a\"\n"			\
     "   .align 4\n"					\
     "   .long  1b,90b\n"				\
     "   .long  3b,90b\n"				\
     "   .long 31b,90b\n"				\
     "   .long 32b,90b\n"				\
     "   .long 33b,90b\n"				\
     "   .long 34b,90b\n"				\
     "   .long 35b,90b\n"				\
     "   .long 36b,90b\n"				\
     "   .long 37b,90b\n"				\
     "   .long  5b,90b\n"				\
     "   .long  7b,90b\n"				\
     ".previous"					\
     : "=a"(s), "=a"(d), "=d"(len)			\
     : "0"(s), "1"(d), "2"(len)				\
     : "d0")
#else
#define CP_IO_TO_MEM(s,d,len)				\
__asm__ __volatile__					\
    ("   oris    %%r12,%%r0,(4)&0xffff\n"			\
     "   rlwinm  %%r8,%2,%%r16,0xffff0000\n"		\
     "   subfco. %%r12,%%r12,%%r8\n"				\
     "   mcrxr   1\n"					\
     "   crnor   %%r6,%%r6,%%r6\n"				\
     "   cror    %%r4,%%r6,%%r2\n"				\
     "   bt      %%r4,8f\n"				\
     "   rlwimi  %%r16,%1,0,0x0000ffff\n"		\
     "   neg     %%r10,%%r16\n"				\
     "   rlwimi  %%r16,%%r10,0,0x000000ff\n"		\
     "   rlwinm  %%r16,%%r16,0,(3)|0xffff0000\n"		\
     "   subf    %2,%%r16,%2\n"				\
     "   b       1b\n"					\
     "1: lbz     %%r11,0(%0)\n"				\
     "   stb     %%r11,0(%1)\n"				\
     "   addi    %1,%1,1\n"				\
     "2: extsh   %%r12,%%r16\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %%r16,%%r12,0,0x0000ffff\n"		\
     "   bne     7,1b\n"				\
     "   rlwimi  %%r16,%2,0,0x0000ffff\n"		\
     "   rlwinm  %%r16,%%r16,(32-5),5+16,31\n"		\
     "   b       4f\n"					\
     "3: lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "31:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "32:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "33:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "34:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "35:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "36:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "37:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "4: extsh   %%r12,%%r16\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %%r16,%%r12,0,0x0000ffff\n"		\
     "   bne     7,3b\n"				\
     "   rlwimi  %%r16,%2,0,0x0000ffff\n"		\
     "   rlwinm  %%r16,%%r16,(32-2),2+16,31\n"		\
     "   rlwinm  %%r16,%%r16,0,(7)|0xffff0000\n"		\
     "   b       6f\n"					\
     "5: lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %1,%1,4\n"				\
     "6: extsh   %%r12,%%r16\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %%r16,%%r12,0,0x0000ffff\n"		\
     "   bne     7,5b\n"				\
     "   rlwinm  %2,%2,0,(3)|0xffff0000\n"		\
     "   b       8f\n"					\
     "7: lbz     %%r11,0(%0)\n"				\
     "   stb     %%r11,0(%1)\n"				\
     "   addi    %1,%1,1\n"				\
     "8: extsh   %%r12,%2\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %2,%%r12,0,0x0000ffff\n"			\
     "   bne     7,7b\n"				\
     "   li      %2,0\n"				\
     "9: \n"						\
     ".section .fixup,\"ax\"\n"				\
     "   .align  2\n"					\
     "90:li      %2,1\n"				\
     "   b       9b\n"					\
     ".previous\n"					\
     ".section __ex_table,\"a\"\n"			\
     "   .align  4\n"					\
     "   .long 1b,90b\n"				\
     "   .long 3b,90b\n"				\
     "   .long 31b,90b\n"				\
     "   .long 32b,90b\n"				\
     "   .long 33b,90b\n"				\
     "   .long 34b,90b\n"				\
     "   .long 35b,90b\n"				\
     "   .long 36b,90b\n"				\
     "   .long 37b,90b\n"				\
     "   .long 5b,90b\n"				\
     "   .long 7b,90b\n"				\
     ".previous"					\
     : "=r"(s), "=r"(d), "=r"(len)			\
     : "0"(s), "1"(d), "2"(len)				\
     : "r0", "r2", "r4", "r6", "r8", "r10", "r11", "r12", "r16", "cc", "memory")
#endif

static int macscsi_pread (struct Scsi_Host *instance,
			  unsigned char *dst, int len)
{
   unsigned char *d;
   volatile unsigned char *s;

   NCR5380_local_declare();
   NCR5380_setup(instance);

   s = mac_scsi_drq+0x60;
   d = dst;

/* These conditions are derived from MacOS */

   while (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ) 
         && !(NCR5380_read(STATUS_REG) & SR_REQ))
      ;
   if (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ) 
         && (NCR5380_read(BUS_AND_STATUS_REG) & BASR_PHASE_MATCH)) {
      printk(KERN_ERR "Error in macscsi_pread\n");
      return -1;
   }

   CP_IO_TO_MEM(s, d, len);
   
   if (len != 0) {
      printk(KERN_NOTICE "Bus error in macscsi_pread\n");
      return -1;
   }
   
   return 0;
}

#if 0
#define CP_MEM_TO_IO(s,d,len)				\
__asm__ __volatile__					\
    ("    cmp.w  #4,%2\n"				\
     "    bls    8f\n"					\
     "    move.w %0,%%d0\n"				\
     "    neg.b  %%d0\n"				\
     "    and.w  #3,%%d0\n"				\
     "    sub.w  %%d0,%2\n"				\
     "    bra    2f\n"					\
     " 1: move.b (%0)+,(%1)\n"				\
     " 2: dbf    %%d0,1b\n"				\
     "    move.w %2,%%d0\n"				\
     "    lsr.w  #5,%%d0\n"				\
     "    bra    4f\n"					\
     " 3: move.l (%0)+,(%1)\n"				\
     "31: move.l (%0)+,(%1)\n"				\
     "32: move.l (%0)+,(%1)\n"				\
     "33: move.l (%0)+,(%1)\n"				\
     "34: move.l (%0)+,(%1)\n"				\
     "35: move.l (%0)+,(%1)\n"				\
     "36: move.l (%0)+,(%1)\n"				\
     "37: move.l (%0)+,(%1)\n"				\
     " 4: dbf    %%d0,3b\n"				\
     "    move.w %2,%%d0\n"				\
     "    lsr.w  #2,%%d0\n"				\
     "    and.w  #7,%%d0\n"				\
     "    bra    6f\n"					\
     " 5: move.l (%0)+,(%1)\n"				\
     " 6: dbf    %%d0,5b\n"				\
     "    and.w  #3,%2\n"				\
     "    bra    8f\n"					\
     " 7: move.b (%0)+,(%1)\n"				\
     " 8: dbf    %2,7b\n"				\
     "    moveq.l #0, %2\n"				\
     " 9: \n"						\
     ".section .fixup,\"ax\"\n"				\
     "    .even\n"					\
     "90: moveq.l #1, %2\n"				\
     "    jra 9b\n"					\
     ".previous\n"					\
     ".section __ex_table,\"a\"\n"			\
     "   .align 4\n"					\
     "   .long  1b,90b\n"				\
     "   .long  3b,90b\n"				\
     "   .long 31b,90b\n"				\
     "   .long 32b,90b\n"				\
     "   .long 33b,90b\n"				\
     "   .long 34b,90b\n"				\
     "   .long 35b,90b\n"				\
     "   .long 36b,90b\n"				\
     "   .long 37b,90b\n"				\
     "   .long  5b,90b\n"				\
     "   .long  7b,90b\n"				\
     ".previous"					\
     : "=a"(s), "=a"(d), "=d"(len)			\
     : "0"(s), "1"(d), "2"(len)				\
     : "d0")
#else
#define CP_MEM_TO_IO(s,d,len)				\
__asm__ __volatile__					\
    ("   oris    %%r12,%%r0,(4)&0xffff\n"			\
     "   rlwinm  %%r8,%2,%%r16,0xffff0000\n"			\
     "   subfco. %%r12,%%r12,%%r8\n"				\
     "   mcrxr   1\n"					\
     "   crnor   %%r6,%%r6,%%r6\n"				\
     "   cror    %%r4,%%r6,%%r2\n"				\
     "   bt      %%r4,8f\n"				\
     "   rlwimi  %%r16,%0,0,0x0000ffff\n"		\
     "   neg     %%r10,%%r16\n"				\
     "   rlwimi  %%r16,%%r10,0,0x000000ff\n"		\
     "   rlwinm  %%r16,%%r16,0,(3)|0xffff0000\n"		\
     "   subf    %2,%%r16,%2\n"				\
     "   b       1b\n"					\
     "1: lbz     %%r11,0(%1)\n"				\
     "   stb     %%r11,0(%0)\n"				\
     "   addi    %0,%0,1\n"				\
     "2: extsh   %%r12,%%r16\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %%r16,%%r12,0,0x0000ffff\n"		\
     "   bne     7,1b\n"				\
     "   rlwimi  %%r16,%2,0,0x0000ffff\n"		\
     "   rlwinm  %%r16,%%r16,(32-5),5+16,31\n"		\
     "   b       4f\n"					\
     "3: lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "31:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "32:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "33:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "34:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "35:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "36:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "37:lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "4: extsh   %%r12,%%r16\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %%r16,%%r12,0,0x0000ffff\n"		\
     "   bne     7,3b\n"				\
     "   rlwimi  %%r16,%2,0,0x0000ffff\n"		\
     "   rlwinm  %%r16,%%r16,(32-2),2+16,31\n"		\
     "   rlwinm  %%r16,%%r16,0,(7)|0xffff0000\n"		\
     "   b       6f\n"					\
     "5: lwz     %%r11,0(%0)\n"				\
     "   stw     %%r11,0(%1)\n"				\
     "   addi    %0,%0,4\n"				\
     "6: extsh   %%r12,%%r16\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %%r16,%%r12,0,0x0000ffff\n"		\
     "   bne     7,5b\n"				\
     "   rlwinm  %2,%2,0,(3)|0xffff0000\n"		\
     "   b       8f\n"					\
     "7: lbz     %%r11,0(%0)\n"				\
     "   stb     %%r11,0(%1)\n"				\
     "   addi    %0,%0,1\n"				\
     "8: extsh   %%r12,%2\n"				\
     "   subi    %%r12,%%r12,1\n"				\
     "   cmpi    7,0,%%r12,-1\n"				\
     "   rlwimi  %2,%%r12,0,0x0000ffff\n"			\
     "   bne     7,7b\n"				\
     "   li      %2,0\n"				\
     "9: \n"						\
     ".section .fixup,\"ax\"\n"				\
     "   .align  2\n"					\
     "90:li      %2,1\n"				\
     "   b       9b\n"					\
     ".previous\n"					\
     ".section __ex_table,\"a\"\n"			\
     "   .align  4\n"					\
     "   .long 1b,90b\n"				\
     "   .long 3b,90b\n"				\
     "   .long 31b,90b\n"				\
     "   .long 32b,90b\n"				\
     "   .long 33b,90b\n"				\
     "   .long 34b,90b\n"				\
     "   .long 35b,90b\n"				\
     "   .long 36b,90b\n"				\
     "   .long 37b,90b\n"				\
     "   .long 5b,90b\n"				\
     "   .long 7b,90b\n"				\
     ".previous"					\
     : "=r"(s), "=r"(d), "=r"(len)			\
     : "0"(s), "1"(d), "2"(len)				\
     : "r0", "r2", "r4", "r6", "r8", "r10", "r11", "r12", "r16", "cc", "memory")
#endif

static int macscsi_pwrite (struct Scsi_Host *instance,
				  unsigned char *src, int len)
{
   unsigned char *s;
   volatile unsigned char *d;

   NCR5380_local_declare();
   NCR5380_setup(instance);

   s = src;
   d = mac_scsi_drq;
   
/* These conditions are derived from MacOS */

   while (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ) 
         && (!(NCR5380_read(STATUS_REG) & SR_REQ) 
            || (NCR5380_read(BUS_AND_STATUS_REG) & BASR_PHASE_MATCH))) 
      ;
   if (!(NCR5380_read(BUS_AND_STATUS_REG) & BASR_DRQ)) {
      printk(KERN_ERR "Error in macscsi_pwrite\n");
      return -1;
   }

   CP_MEM_TO_IO(s, d, len);   

   if (len != 0) {
      printk(KERN_NOTICE "Bus error in macscsi_pwrite\n");
      return -1;
   }
   
   return 0;
}
#endif

#ifdef DRQ_INTERRUPT
static inline void macscsi_dma_read(void)
{
	volatile unsigned char *dma_io = mac_scsi_drq + 0x60;

	*pdma_dst++ = in_8((void *)dma_io);
	pdma_read_residue -= 1;
}

static inline void macscsi_dma_write(void)
{
	volatile unsigned char *dma_io = mac_scsi_drq;

	out_8((void *)dma_io, *pdma_src++);
	pdma_write_residue -= 1;
}

static void macscsi_dma_intr(int irq, void *dev_id, struct pt_regs *regs)
{
//	printk(KERN_DEBUG "SCSI DRQ\n");

	if(!pdma_write && pdma_read_residue)
		macscsi_dma_read();
	else if(pdma_write && pdma_write_residue)
		macscsi_dma_write();
}
#endif

/*
#define USLEEP
#define USLEEP_POLL	1
#define USLEEP_SLEEP	20
#define USLEEP_WAITLONG	500
*/

//#include "mac_NCR5380.c"
#include "NCR5380.c"

static struct scsi_host_template driver_template = NBPMAC_NCR5380;

MODULE_DESCRIPTION("NuBus PMac NCR5380 SCSI driver");
MODULE_AUTHOR("adapted from mac_scsi");
MODULE_LICENSE("GPL");
//MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");

#include "scsi_module.c"
