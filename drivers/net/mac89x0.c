/* mac89x0.c: A Crystal Semiconductor CS89[02]0 driver for linux. */
/*
	Written 1996 by Russell Nelson, with reference to skeleton.c
	written 1993-1994 by Donald Becker.

	This software may be used and distributed according to the terms
	of the GNU General Public License, incorporated herein by reference.

	The author may be reached at nelson@crynwr.com, Crynwr
	Software, 11 Grant St., Potsdam, NY 13676

  Changelog:

  Mike Cruse        : mcruse@cti-ltd.com
                    : Changes for Linux 2.0 compatibility.
                    : Added dev_id parameter in net_interrupt(),
                    : request_irq() and free_irq(). Just NULL for now.

  Mike Cruse        : Added MOD_INC_USE_COUNT and MOD_DEC_USE_COUNT macros
                    : in net_open() and net_close() so kerneld would know
                    : that the module is in use and wouldn't eject the
                    : driver prematurely.

  Mike Cruse        : Rewrote init_module() and cleanup_module using 8390.c
                    : as an example. Disabled autoprobing in init_module(),
                    : not a good thing to do to other devices while Linux
                    : is running from all accounts.

  Alan Cox          : Removed 1.2 support, added 2.1 extra counters.

  David Huggins-Daines <dhd@debian.org>

  Split this off into mac89x0.c, and gutted it of all parts which are
  not relevant to the existing CS8900 cards on the Macintosh
  (i.e. basically the Daynaport CS and LC cards).  To be precise:

    * Removed all the media-detection stuff, because these cards are
    TP-only.

    * Lobotomized the ISA interrupt bogosity, because these cards use
    a hardwired NuBus interrupt and a magic ISAIRQ value in the card.

    * Basically eliminated everything not relevant to getting the
    cards minimally functioning on the Macintosh.

  I might add that these cards are badly designed even from the Mac
  standpoint, in that Dayna, in their infinite wisdom, used NuBus slot
  I/O space and NuBus interrupts for these cards, but neglected to
  provide anything even remotely resembling a NuBus ROM.  Therefore we
  have to probe for them in a brain-damaged ISA-like fashion.

  Arnaldo Carvalho de Melo <acme@conectiva.com.br> - 11/01/2001
  check kmalloc and release the allocated memory on failure in
  mac89x0_probe and in init_module
  use local_irq_{save,restore}(flags) in net_get_stat, not just
  local_irq_{dis,en}able()
*/

static char *version =
"cs89x0.c:v1.02 11/26/96 Russell Nelson <nelson@crynwr.com>\n";

/* ======================= configure the driver here ======================= */

/* use 0 for production, 1 for verification, >2 for debug */
#ifndef NET_DEBUG
#define NET_DEBUG 0
#endif

/* ======================= end of configuration ======================= */


/* Always include 'config.h' first in case the user wants to turn on
   or override something. */
#include <linux/module.h>

/*
  Sources:

	Crynwr packet driver epktisa.

	Crystal Semiconductor data sheets.

*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/string.h>
#include <linux/nubus.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/gfp.h>

#include <asm/system.h>
#include <asm/io.h>
#ifndef CONFIG_NBPMAC
#include <asm/hwtest.h>
#include <asm/macints.h>
#endif

#include "cs89x0.h"

#ifdef CONFIG_NBPMAC
#undef LOCK
#ifdef LOCK
#include <linux/spinlock.h>
#endif
extern int nbpmac_slot2irq(int);
#define SLOT2IRQ(SLOT)	nbpmac_slot2irq(SLOT)
#endif

static unsigned int net_debug = NET_DEBUG;

/* Information that need to be kept for each board. */
struct net_local {
	struct net_device_stats stats;
	int chip_type;		/* one of: CS8900, CS8920, CS8920M */
	char chip_revision;	/* revision letter of the chip ('A'...) */
	int send_cmd;		/* the propercommand used to send a packet. */
	int rx_mode;
	int curr_rx_cfg;
        int send_underrun;      /* keep track of how many underruns in a row we get */
	struct sk_buff *skb;
#ifdef LOCK
	spinlock_t lock;
#endif
};

/* Index to functions, as function prototypes. */

#if 0
extern void reset_chip(struct net_device *dev);
#endif
static int net_open(struct net_device *dev);
static int net_send_packet(struct sk_buff *skb, struct net_device *dev);
static irqreturn_t net_interrupt(int irq, void *dev_id);
static void set_multicast_list(struct net_device *dev);
static void net_rx(struct net_device *dev);
static int net_close(struct net_device *dev);
static struct net_device_stats *net_get_stats(struct net_device *dev);
static int set_mac_address(struct net_device *dev, void *addr);


/* Example routines you must write ;->. */
#define tx_done(dev) 1

/* For reading/writing registers ISA-style */
static inline int
readreg_io(struct net_device *dev, int portno)
{
	nubus_writew(swab16(portno), dev->base_addr + ADD_PORT);
	return swab16(nubus_readw(dev->base_addr + DATA_PORT));
}

static inline void
writereg_io(struct net_device *dev, int portno, int value)
{
	nubus_writew(swab16(portno), dev->base_addr + ADD_PORT);
	nubus_writew(swab16(value), dev->base_addr + DATA_PORT);
}

/* These are for reading/writing registers in shared memory */
static inline int
readreg(struct net_device *dev, int portno)
{
#ifdef CONFIG_NBPMAC
	return readreg_io(dev, portno);
#else
	return swab16(nubus_readw(dev->mem_start + portno));
#endif
}

static inline void
writereg(struct net_device *dev, int portno, int value)
{
#ifdef CONFIG_NBPMAC
	writereg_io(dev, portno, value);
#else
	nubus_writew(swab16(value), dev->mem_start + portno);
#endif
}

static const struct net_device_ops mac89x0_netdev_ops = {
	.ndo_open		= net_open,
	.ndo_stop		= net_close,
	.ndo_start_xmit		= net_send_packet,
	.ndo_get_stats		= net_get_stats,
	.ndo_set_multicast_list	= set_multicast_list,
	.ndo_set_mac_address	= set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
};

/* Probe for the CS8900 card in slot E.  We won't bother looking
   anywhere else until we have a really good reason to do so. */
struct net_device * __init mac89x0_probe(int unit)
{
	struct net_device *dev;
	static int once_is_enough;
	struct net_local *lp;
	static unsigned version_printed;
	int i;
#ifndef CONFIG_NBPMAC
	int slot;
#endif
	unsigned rev_type = 0;
	unsigned long ioaddr = 0;
	unsigned short sig;
	int err = -ENODEV;

#ifdef CONFIG_NBPMAC
	struct nubus_dev * ndev = NULL;
	struct nubus_dir dir;
	struct nubus_dirent ent;
#else
	if (!MACH_IS_MAC)
		return ERR_PTR(-ENODEV);
#endif

	dev = alloc_etherdev(sizeof(struct net_local));
	if (!dev)
		return ERR_PTR(-ENOMEM);

	if (unit >= 0) {
		sprintf(dev->name, "eth%d", unit);
		netdev_boot_setup_check(dev);
	}

	if (once_is_enough)
		goto out;
	once_is_enough = 1;

#ifdef CONFIG_NBPMAC
	/* Detect all Ethernet cards, figure out which one is ours */
	while ((ndev = nubus_find_type(NUBUS_CAT_NETWORK,
				       NUBUS_TYPE_ETHERNET, ndev)) != NULL)
	{
		/* Now do the hardware-specific stuff */
		printk(KERN_INFO "mac89x0: Found a %s in slot %X\n",
		       ndev->board->name, ndev->board->slot);
		ioaddr = ndev->board->slot_addr + 0x600000;

		/* Get the functional resource for this device */
		if (nubus_get_func_dir(ndev, &dir) == -1)
			goto out;
		if (nubus_find_rsrc(&dir, NUBUS_RESID_MAC_ADDRESS, &ent) == -1)
			goto out;

		nubus_get_rsrc_mem(dev->dev_addr, &ent, 6);

		/* print the ethernet address. */
		printk(KERN_INFO "mac89x0: MAC address in NuBus ROM: ");
		for (i = 0; i < ETH_ALEN; i++)
			printk("%2.2x%s", dev->dev_addr[i],
			       ((i < ETH_ALEN-1) ? ":" : ""));
		printk("\n");

		dev->irq = SLOT2IRQ(ndev->board->slot);
		irq_create_mapping(NULL, dev->irq);
	}
#else
	/* We might have to parameterize this later */
	slot = 0xE;
	/* Get out now if there's a real NuBus card in slot E */
	if (nubus_find_slot(slot, NULL) != NULL)
		goto out;

	/* The pseudo-ISA bits always live at offset 0x300 (gee,
           wonder why...) */
	ioaddr = (unsigned long)
		nubus_slot_addr(slot) | (((slot&0xf) << 20) + DEFAULTIOBASE);
#endif
	{
		unsigned long flags;
		int card_present;

		local_irq_save(flags);
		card_present = (hwreg_present((void*) ioaddr+4) &&
				hwreg_present((void*) ioaddr + DATA_PORT));
		local_irq_restore(flags);

		if (!card_present) {
			printk(KERN_DEBUG "mac89x0: can't access registers!\n");
			goto out;
		}
	}

	nubus_writew(0, ioaddr + ADD_PORT);
	sig = nubus_readw(ioaddr + DATA_PORT);
	if (sig != swab16(CHIP_EISA_ID_SIG)) {
		printk(KERN_DEBUG "mac89x0: found signature 0x%x instead of 0x%x\n", sig, swab16(CHIP_EISA_ID_SIG));
		goto out;
	}

	/* Initialize the net_device structure. */
	lp = netdev_priv(dev);

	/* Fill in the 'dev' fields. */
	dev->base_addr = ioaddr;
	dev->mem_start = (unsigned long)
#ifndef CONFIG_NBPMAC
		nubus_slot_addr(slot) | (((slot&0xf) << 20) + MMIOBASE);
#else
		(dev->base_addr & 0xffff0000) + MMIOBASE;
#endif
	dev->mem_end = dev->mem_start + 0x1000;

	/* Turn on shared memory */
	writereg_io(dev, PP_BusCTL, MEMORY_ON);

	/* get the chip type */
	rev_type = readreg(dev, PRODUCT_ID_ADD);
	lp->chip_type = rev_type &~ REVISON_BITS;
	lp->chip_revision = ((rev_type & REVISON_BITS) >> 8) + 'A';

	/* Check the chip type and revision in order to set the correct send command
	CS8920 revision C and CS8900 revision F can use the faster send. */
	lp->send_cmd = TX_AFTER_381;
	if (lp->chip_type == CS8900 && lp->chip_revision >= 'F')
		lp->send_cmd = TX_NOW;
	if (lp->chip_type != CS8900 && lp->chip_revision >= 'C')
		lp->send_cmd = TX_NOW;

	if (net_debug && version_printed++ == 0)
		printk(version);

	printk(KERN_INFO "%s: cs89%c0%s rev %c found at %#8lx",
	       dev->name,
	       lp->chip_type==CS8900?'0':'2',
	       lp->chip_type==CS8920M?"M":"",
	       lp->chip_revision,
	       dev->base_addr);

#ifndef CONFIG_NBPMAC
	/* Try to read the MAC address */
	if ((readreg(dev, PP_SelfST) & (EEPROM_PRESENT | EEPROM_OK)) == 0) {
		printk("\nmac89x0: No EEPROM, giving up now.\n");
		goto out1;
        } else {
                for (i = 0; i < ETH_ALEN; i += 2) {
			/* Big-endian (why??!) */
			unsigned short s = readreg(dev, PP_IA + i);
                        dev->dev_addr[i] = s >> 8;
                        dev->dev_addr[i+1] = s & 0xff;
                }
        }

	dev->irq = SLOT2IRQ(slot);
#elif defined(LOCK)
	spin_lock_init(&lp->lock);
#endif

	/* print the IRQ and ethernet address. */

	printk(" IRQ %d ADDR %pM\n", dev->irq, dev->dev_addr);

	dev->netdev_ops		= &mac89x0_netdev_ops;

	err = register_netdev(dev);
	if (err)
		goto out1;
	return NULL;
out1:
	nubus_writew(0, dev->base_addr + ADD_PORT);
out:
	free_netdev(dev);
	return ERR_PTR(err);
}

#if 0
/* This is useful for something, but I don't know what yet. */
void __init reset_chip(struct net_device *dev)
{
	int reset_start_time;

	writereg(dev, PP_SelfCTL, readreg(dev, PP_SelfCTL) | POWER_ON_RESET);

	/* wait 30 ms */
	msleep_interruptible(30);

	/* Wait until the chip is reset */
	reset_start_time = jiffies;
	while( (readreg(dev, PP_SelfST) & INIT_DONE) == 0 && jiffies - reset_start_time < 2)
		;
}
#endif

/* Open/initialize the board.  This is called (in the current kernel)
   sometime after booting when the 'ifconfig' program is run.

   This routine should set everything up anew at each open, even
   registers that "should" only need to be set once at boot, so that
   there is non-reboot way to recover if something goes wrong.
   */
static int
net_open(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	int i;

	/* Disable the interrupt for now */
	writereg(dev, PP_BusCTL, readreg(dev, PP_BusCTL) & ~ENABLE_IRQ);

	/* Grab the interrupt */
	if (request_irq(dev->irq, net_interrupt, 0, "cs89x0", dev))
	{
		printk(KERN_DEBUG "mac89x0: couldn't get irq!\n");
		return -EAGAIN;
	}

	/* Set up the IRQ - Apparently magic */
	if (lp->chip_type == CS8900)
		writereg(dev, PP_CS8900_ISAINT, 0);
	else
		writereg(dev, PP_CS8920_ISAINT, 0);

	/* set the Ethernet address */
	for (i=0; i < ETH_ALEN/2; i++)
		writereg(dev, PP_IA+i*2, dev->dev_addr[i*2] | (dev->dev_addr[i*2+1] << 8));

	/* Turn on both receive and transmit operations */
	writereg(dev, PP_LineCTL, readreg(dev, PP_LineCTL) | SERIAL_RX_ON | SERIAL_TX_ON);

	/* Receive only error free packets addressed to this card */
	lp->rx_mode = 0;
	writereg(dev, PP_RxCTL, DEF_RX_ACCEPT);

	lp->curr_rx_cfg = RX_OK_ENBL | RX_CRC_ERROR_ENBL;

	writereg(dev, PP_RxCFG, lp->curr_rx_cfg);

	writereg(dev, PP_TxCFG, TX_LOST_CRS_ENBL | TX_SQE_ERROR_ENBL | TX_OK_ENBL |
	       TX_LATE_COL_ENBL | TX_JBR_ENBL | TX_ANY_COL_ENBL | TX_16_COL_ENBL);

	writereg(dev, PP_BufCFG, READY_FOR_TX_ENBL | RX_MISS_COUNT_OVRFLOW_ENBL |
		 TX_COL_COUNT_OVRFLOW_ENBL | TX_UNDERRUN_ENBL);

	netif_start_queue(dev);
	/* now that we've got our act together, enable everything */
	writereg(dev, PP_BusCTL, readreg(dev, PP_BusCTL) | ENABLE_IRQ);
	return 0;
}

static int
net_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	unsigned long flags;

	if (net_debug > 3)
		printk("%s: sent %d byte packet of type %x\n",
		       dev->name, skb->len,
		       (skb->data[ETH_ALEN+ETH_ALEN] << 8)
		       | skb->data[ETH_ALEN+ETH_ALEN+1]);

	/* keep the upload from being interrupted, since we
	   ask the chip to start transmitting before the
	   whole packet has been completely uploaded. */
#ifdef LOCK
	spin_lock_irqsave(&lp->lock, flags);
#else
	local_irq_save(flags);
#endif
	netif_stop_queue(dev);

	/* initiate a transmit sequence */
	writereg(dev, PP_TxCMD, lp->send_cmd);
	writereg(dev, PP_TxLength, skb->len);

	/* Test to see if the chip has allocated memory for the packet */
	if ((readreg(dev, PP_BusST) & READY_FOR_TX_NOW) == 0) {
		/* Gasp!  It hasn't.  But that shouldn't happen since
		   we're waiting for TxOk, so return 1 and requeue this packet. */
#ifdef LOCK
		spin_unlock_irqrestore(&lp->lock, flags);
#else
		local_irq_restore(flags);
#endif
		return NETDEV_TX_BUSY;
	}

#ifndef CONFIG_NBPMAC
	/* Write the contents of the packet */
	skb_copy_from_linear_data(skb, (void *)(dev->mem_start + PP_TxFrame),
				  skb->len+1);

#else
	outsw(dev->base_addr + TX_FRAME_PORT, skb->data, (skb->len + 1) / 2);
#endif
#ifdef LOCK
	spin_unlock_irqrestore(&lp->lock, flags);
#else
	local_irq_restore(flags);
#endif
	dev->trans_start = jiffies;
	dev_kfree_skb (skb);

	return NETDEV_TX_OK;
}

/* The typical workload of the driver:
   Handle the network interface interrupts. */
static irqreturn_t net_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_local *lp;
	int ioaddr, status;

	if (dev == NULL) {
		printk ("net_interrupt(): irq %d for unknown device.\n", irq);
		return IRQ_NONE;
	}

	ioaddr = dev->base_addr;
	lp = netdev_priv(dev);

	/* we MUST read all the events out of the ISQ, otherwise we'll never
           get interrupted again.  As a consequence, we can't have any limit
           on the number of times we loop in the interrupt handler.  The
           hardware guarantees that eventually we'll run out of events.  Of
           course, if you're on a slow machine, and packets are arriving
           faster than you can read them off, you're screwed.  Hasta la
           vista, baby!  */
	while ((status = swab16(nubus_readw(dev->base_addr + ISQ_PORT)))) {
		if (net_debug > 4)printk("%s: event=%04x\n", dev->name, status);
		switch(status & ISQ_EVENT_MASK) {
		case ISQ_RECEIVER_EVENT:
			/* Got a packet(s). */
			net_rx(dev);
			break;
		case ISQ_TRANSMITTER_EVENT:
			lp->stats.tx_packets++;
			netif_wake_queue(dev);
			if ((status & TX_OK) == 0) lp->stats.tx_errors++;
			if (status & TX_LOST_CRS) lp->stats.tx_carrier_errors++;
			if (status & TX_SQE_ERROR) lp->stats.tx_heartbeat_errors++;
			if (status & TX_LATE_COL) lp->stats.tx_window_errors++;
			if (status & TX_16_COL) lp->stats.tx_aborted_errors++;
			break;
		case ISQ_BUFFER_EVENT:
			if (status & READY_FOR_TX) {
				/* we tried to transmit a packet earlier,
                                   but inexplicably ran out of buffers.
                                   That shouldn't happen since we only ever
                                   load one packet.  Shrug.  Do the right
                                   thing anyway. */
				netif_wake_queue(dev);
			}
			if (status & TX_UNDERRUN) {
				if (net_debug > 0) printk("%s: transmit underrun\n", dev->name);
                                lp->send_underrun++;
                                if (lp->send_underrun == 3) lp->send_cmd = TX_AFTER_381;
                                else if (lp->send_underrun == 6) lp->send_cmd = TX_AFTER_ALL;
                        }
			break;
		case ISQ_RX_MISS_EVENT:
			lp->stats.rx_missed_errors += (status >>6);
			break;
		case ISQ_TX_COL_EVENT:
			lp->stats.collisions += (status >>6);
			break;
		}
	}
	return IRQ_HANDLED;
}

/* We have a good packet(s), get it/them out of the buffers. */
static void
net_rx(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	struct sk_buff *skb;
	int status, length;

	status = readreg(dev, PP_RxStatus);
	if ((status & RX_OK) == 0) {
		lp->stats.rx_errors++;
		if (status & RX_RUNT) lp->stats.rx_length_errors++;
		if (status & RX_EXTRA_DATA) lp->stats.rx_length_errors++;
		if (status & RX_CRC_ERROR) if (!(status & (RX_EXTRA_DATA|RX_RUNT)))
			/* per str 172 */
			lp->stats.rx_crc_errors++;
		if (status & RX_DRIBBLE) lp->stats.rx_frame_errors++;
		return;
	}

	length = readreg(dev, PP_RxLength);
	/* Malloc up new buffer. */
#ifndef CONFIG_NBPMAC
	skb = alloc_skb(length, GFP_ATOMIC);
#else
	skb = dev_alloc_skb(length + 4);
#endif
	if (skb == NULL) {
		printk("%s: Memory squeeze, dropping packet.\n", dev->name);
		lp->stats.rx_dropped++;
		return;
	}
#ifndef CONFIG_NBPMAC
	skb_put(skb, length);

	skb_copy_to_linear_data(skb, (void *)(dev->mem_start + PP_RxFrame),
				length);
#else
	skb->dev = dev;
	skb_reserve(skb, 2);	/* longword align L3 header */
	insw(dev->base_addr + RX_FRAME_PORT, skb_put(skb, length), (length + 1) / 2);
#endif

	if (net_debug > 3)printk("%s: received %d byte packet of type %x\n",
                                 dev->name, length,
                                 (skb->data[ETH_ALEN+ETH_ALEN] << 8)
				 | skb->data[ETH_ALEN+ETH_ALEN+1]);

        skb->protocol=eth_type_trans(skb,dev);
	netif_rx(skb);
	lp->stats.rx_packets++;
	lp->stats.rx_bytes += length;
}

/* The inverse routine to net_open(). */
static int
net_close(struct net_device *dev)
{

	writereg(dev, PP_RxCFG, 0);
	writereg(dev, PP_TxCFG, 0);
	writereg(dev, PP_BufCFG, 0);
	writereg(dev, PP_BusCTL, 0);

	netif_stop_queue(dev);

	free_irq(dev->irq, dev);

	/* Update the statistics here. */

	return 0;

}

/* Get the current statistics.	This may be called with the card open or
   closed. */
static struct net_device_stats *
net_get_stats(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	unsigned long flags;

	local_irq_save(flags);
	/* Update the statistics from the device registers. */
	lp->stats.rx_missed_errors += (readreg(dev, PP_RxMiss) >> 6);
	lp->stats.collisions += (readreg(dev, PP_TxCol) >> 6);
	local_irq_restore(flags);

	return &lp->stats;
}

static void set_multicast_list(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	if(dev->flags&IFF_PROMISC)
	{
		lp->rx_mode = RX_ALL_ACCEPT;
	} else if ((dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev)) {
		/* The multicast-accept list is initialized to accept-all, and we
		   rely on higher-level filtering for now. */
		lp->rx_mode = RX_MULTCAST_ACCEPT;
	}
	else
		lp->rx_mode = 0;

	writereg(dev, PP_RxCTL, DEF_RX_ACCEPT | lp->rx_mode);

	/* in promiscuous mode, we accept errored packets, so we have to enable interrupts on them also */
	writereg(dev, PP_RxCFG, lp->curr_rx_cfg |
	     (lp->rx_mode == RX_ALL_ACCEPT? (RX_CRC_ERROR_ENBL|RX_RUNT_ENBL|RX_EXTRA_DATA_ENBL) : 0));
}


static int set_mac_address(struct net_device *dev, void *addr)
{
	int i;
	printk("%s: Setting MAC address to ", dev->name);
	for (i = 0; i < 6; i++)
		printk(" %2.2x", dev->dev_addr[i] = ((unsigned char *)addr)[i]);
	printk(".\n");
	/* set the Ethernet address */
	for (i=0; i < ETH_ALEN/2; i++)
		writereg(dev, PP_IA+i*2, dev->dev_addr[i*2] | (dev->dev_addr[i*2+1] << 8));

	return 0;
}

#ifdef MODULE

static struct net_device *dev_cs89x0;
static int debug;

module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "CS89[02]0 debug level (0-5)");
MODULE_LICENSE("GPL");

int __init
init_module(void)
{
	net_debug = debug;
        dev_cs89x0 = mac89x0_probe(-1);
	if (IS_ERR(dev_cs89x0)) {
                printk(KERN_WARNING "mac89x0.c: No card found\n");
		return PTR_ERR(dev_cs89x0);
	}
	return 0;
}

void
cleanup_module(void)
{
	unregister_netdev(dev_cs89x0);
	nubus_writew(0, dev_cs89x0->base_addr + ADD_PORT);
	free_netdev(dev_cs89x0);
}
#endif /* MODULE */
