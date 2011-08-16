#ifndef _PPC_KERNEL_PMAC_NUBUS_H
#define _PPC_KERNEL_PMAC_NUBUS_H

#if 0
/* PDM */
void __init pmac_amic_init(void);
void amic_dma_init(void);
#endif

/* PowerBook */
extern void __init pmac_m2_init(void);
extern void nbpbook_exp_init(void);
//void m2_poll_init(void);

/* Performa 5/6200 */
extern void __init pmac_pfm_init(void);
void pfm_vlk_init(void);

extern int is_PM6100;

extern int __init nbpmac_pic_probe(void);
extern long nbpmac_time_init(void);
extern unsigned long nbpmac_find_end_of_memory(void);

#endif /* _PPC_KERNEL_PMAC_NUBUS_H */
