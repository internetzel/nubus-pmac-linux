/*
 *  Macintosh IDE Driver
 *
 *     Copyright (C) 1998 by Michael Schmitz
 *
 *  This driver was written based on information obtained from the MacOS IDE
 *  driver binary by Mikael Forselius
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/ide.h>

#undef DEBUG

/*
 * Generic IDE registers as offsets from the base
 * These match MkLinux so they should be correct.
 */

#define IDE_CONTROL	0x38	/* control/altstatus */

static void __init nbpmacide_setup_ports(struct ide_hw *hw, unsigned long base,
					 int irq)
{
	int i;

	memset(hw, 0, sizeof(*hw));

	for (i = 0; i < 8; i++)
		hw->io_ports_array[i] = base + i * 4;  

	hw->io_ports.ctl_addr = base + IDE_CONTROL;

	hw->irq = irq;
}

u8 __iomem *ide_ifr;

int nbpmacide_test_irq(ide_hwif_t *hwif)
{
	u8 val = in_8(ide_ifr);
	u8 flag = (hwif->channel == 1)?  0x1 : 0x2;
	int res = ((val & flag) > 0)? 1 : 0;

#ifdef DEBUG
	printk("|T%d.0x%x|", hwif->channel, val);
#endif
	return res;
}

static void nbpmacide_clear_irq(ide_drive_t *drive)
{
	u8 val = in_8(ide_ifr);
	u8 flag = (drive->hwif->channel == 1)?  0x1 : 0x2;

	out_8(ide_ifr, val & ~flag);
#ifdef DEBUG
	printk("|C%d.0x%x.0x%x|", drive->hwif->channel, val, in_8(ide_ifr));
#endif
}

static const struct ide_port_ops nbpmacide_port_ops = {
	.clear_irq		= nbpmacide_clear_irq,
	.test_irq		= nbpmacide_test_irq,
};

static const struct ide_port_info nbpmacide_port_info = {
	.port_ops		= &nbpmacide_port_ops,
	.host_flags		= IDE_HFLAG_MMIO | IDE_HFLAG_NO_DMA | IDE_HFLAG_UNMASK_IRQS,
	.irq_flags		= /*IRQF_DISABLED |*/ IRQF_SHARED,
	.chipset		= ide_generic,
};

extern int hwreg_present(void __iomem *regp);

/*
 * Probe for a NuBus Power Macintosh IDE interface
 */

static int __init nbpmacide_init(void)
{
	unsigned long base;
	int irq;
	unsigned int nr_ports = 0;
	struct ide_hw hw, hw2, *hws[] = { &hw, &hw2 };
	struct ide_port_info d = nbpmacide_port_info;
	const u32 *reg;
	struct device_node *device = NULL;

	if (of_machine_is_compatible("Performa")) {
		device = of_find_node_by_name(NULL, "f108");
		d.port_ops = NULL;
	} else if (of_machine_is_compatible("M2")) {
		device = of_find_node_by_name(NULL, "baboon");
	}
	if (device == NULL)
		return -ENODEV;

	reg = of_get_property(device, "reg", NULL);
	if (reg == NULL) {
		printk(KERN_ERR "nbpmacide: No \"reg\" property !\n");
		of_node_put(device);
		return -ENODEV;
	}

	base = (unsigned long)of_iomap(device, 0);
	irq = irq_of_parse_and_map(device, 0);

	printk(KERN_INFO "ide: NuBus Power Macintosh IDE controller\n");
	nbpmacide_setup_ports(&hw, base, irq);
	nr_ports++;

	/* Currently on baboon only we use its interrupt register */
	/* We should change this for Performas also, see macide.c */
	if (d.port_ops) {
		ide_ifr = (u8 __iomem *)of_iomap(device, 2) + 0x9;
	} else {
		irq = irq_of_parse_and_map(device, 1);
	}

	base = (unsigned long)of_iomap(device, 1);

	if (hwreg_present((u8 __iomem *)base)) {
		printk(KERN_INFO "ide: NuBus Power Macintosh IDE controller, 2nd port\n");
		nbpmacide_setup_ports(&hw2, base, irq);
		nr_ports++;
	}

	of_node_put(device);

	return ide_host_add(&d, hws, nr_ports, NULL);
}

module_init(nbpmacide_init);

MODULE_LICENSE("GPL");
