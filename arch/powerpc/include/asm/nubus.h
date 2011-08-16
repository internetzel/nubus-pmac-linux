#ifndef _ASM_PPC_NUBUS_H
#define _ASM_PPC_NUBUS_H

#include <asm/io.h>

#define nubus_readb(port)	in_8((u8 __iomem *)(port))
#define nubus_readw(port)	in_be16((u16 __iomem *)(port))
#define nubus_readl(port)	in_be32((u32 __iomem *)(port))

#define nubus_writeb(val, port)	out_8(((u8 __iomem *)(port)),(val))
#define nubus_writew(val, port)	out_be16(((u16 __iomem *)(port)),(val))
#define nubus_writel(val, port)	out_be32(((u32 __iomem *)(port)),(val))

extern int hwreg_present(void __iomem *);
extern int nbpmac_slot2irq(int);
extern int nbpmac_irq2slot(int);
#define SLOT2IRQ(SLOT) nbpmac_slot2irq(SLOT)
#define IRQ2SLOT(IRQ)  nbpmac_irq2slot(IRQ)

#define IRQ_NUBUS_9	  (24)
#define IRQ_NUBUS_A	  (25)
#define IRQ_NUBUS_B	  (26)
#define IRQ_NUBUS_C	  (27)
#define IRQ_NUBUS_D	  (28)
#define IRQ_NUBUS_E	  (29)
#define IRQ_NUBUS_F	  (30)

#endif /* _ASM_NUBUS_H */
