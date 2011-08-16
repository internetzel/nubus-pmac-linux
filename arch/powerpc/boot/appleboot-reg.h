#define MSR_SF_LG	63              /* Enable 64 bit mode */
#define MSR_ISF_LG	61              /* Interrupt 64b mode valid on 630 */
#define MSR_HV_LG 	60              /* Hypervisor state */
#define MSR_VEC_LG	25	        /* Enable AltiVec */
#define MSR_VSX_LG	23		/* Enable VSX */
#define MSR_POW_LG	18		/* Enable Power Management */
#define MSR_WE_LG	18		/* Wait State Enable */
#define MSR_TGPR_LG	17		/* TLB Update registers in use */
#define MSR_CE_LG	17		/* Critical Interrupt Enable */
#define MSR_ILE_LG	16		/* Interrupt Little Endian */
#define MSR_EE_LG	15		/* External Interrupt Enable */
#define MSR_PR_LG	14		/* Problem State / Privilege Level */
#define MSR_FP_LG	13		/* Floating Point enable */
#define MSR_ME_LG	12		/* Machine Check Enable */
#define MSR_FE0_LG	11		/* Floating Exception mode 0 */
#define MSR_SE_LG	10		/* Single Step */
#define MSR_BE_LG	9		/* Branch Trace */
#define MSR_DE_LG	9 		/* Debug Exception Enable */
#define MSR_FE1_LG	8		/* Floating Exception mode 1 */
#define MSR_IP_LG	6		/* Exception prefix 0x000/0xFFF */
#define MSR_IR_LG	5 		/* Instruction Relocate */
#define MSR_DR_LG	4 		/* Data Relocate */
#define MSR_PE_LG	3		/* Protection Enable */
#define MSR_PX_LG	2		/* Protection Exclusive Mode */
#define MSR_PMM_LG	2		/* Performance monitor */
#define MSR_RI_LG	1		/* Recoverable Exception */
#define MSR_LE_LG	0 		/* Little Endian */

#ifdef __ASSEMBLY__
#define __MASK(X)	(1<<(X))
#else
#define __MASK(X)	(1UL<<(X))
#endif

#define MSR_VEC		__MASK(MSR_VEC_LG)	/* Enable AltiVec */
#define MSR_VSX		__MASK(MSR_VSX_LG)	/* Enable VSX */
#define MSR_POW		__MASK(MSR_POW_LG)	/* Enable Power Management */
#define MSR_WE		__MASK(MSR_WE_LG)	/* Wait State Enable */
#define MSR_TGPR	__MASK(MSR_TGPR_LG)	/* TLB Update registers in use */
#define MSR_CE		__MASK(MSR_CE_LG)	/* Critical Interrupt Enable */
#define MSR_ILE		__MASK(MSR_ILE_LG)	/* Interrupt Little Endian */
#define MSR_EE		__MASK(MSR_EE_LG)	/* External Interrupt Enable */
#define MSR_PR		__MASK(MSR_PR_LG)	/* Problem State / Privilege Level */
#define MSR_FP		__MASK(MSR_FP_LG)	/* Floating Point enable */
#define MSR_ME		__MASK(MSR_ME_LG)	/* Machine Check Enable */
#define MSR_FE0		__MASK(MSR_FE0_LG)	/* Floating Exception mode 0 */
#define MSR_SE		__MASK(MSR_SE_LG)	/* Single Step */
#define MSR_BE		__MASK(MSR_BE_LG)	/* Branch Trace */
#define MSR_DE		__MASK(MSR_DE_LG)	/* Debug Exception Enable */
#define MSR_FE1		__MASK(MSR_FE1_LG)	/* Floating Exception mode 1 */
#define MSR_IP		__MASK(MSR_IP_LG)	/* Exception prefix 0x000/0xFFF */
#define MSR_IR		__MASK(MSR_IR_LG)	/* Instruction Relocate */
#define MSR_DR		__MASK(MSR_DR_LG)	/* Data Relocate */
#define MSR_PE		__MASK(MSR_PE_LG)	/* Protection Enable */
#define MSR_PX		__MASK(MSR_PX_LG)	/* Protection Exclusive Mode */
#ifndef MSR_PMM
#define MSR_PMM		__MASK(MSR_PMM_LG)	/* Performance monitor */
#endif
#define MSR_RI		__MASK(MSR_RI_LG)	/* Recoverable Exception */
#define MSR_LE		__MASK(MSR_LE_LG)	/* Little Endian */

/* Default MSR for kernel mode. */
#ifndef MSR_KERNEL	/* reg_booke.h also defines this */
#define MSR_KERNEL	(MSR_ME|MSR_RI|MSR_IR|MSR_DR)
#endif

#define MSR_USER	(MSR_KERNEL|MSR_PR|MSR_EE)

/* Floating Point Status and Control Register (FPSCR) Fields */
#define FPSCR_FX	0x80000000	/* FPU exception summary */
#define FPSCR_FEX	0x40000000	/* FPU enabled exception summary */
#define FPSCR_VX	0x20000000	/* Invalid operation summary */
#define FPSCR_OX	0x10000000	/* Overflow exception summary */
#define FPSCR_UX	0x08000000	/* Underflow exception summary */
#define FPSCR_ZX	0x04000000	/* Zero-divide exception summary */
#define FPSCR_XX	0x02000000	/* Inexact exception summary */
#define FPSCR_VXSNAN	0x01000000	/* Invalid op for SNaN */
#define FPSCR_VXISI	0x00800000	/* Invalid op for Inv - Inv */
#define FPSCR_VXIDI	0x00400000	/* Invalid op for Inv / Inv */
#define FPSCR_VXZDZ	0x00200000	/* Invalid op for Zero / Zero */
#define FPSCR_VXIMZ	0x00100000	/* Invalid op for Inv * Zero */
#define FPSCR_VXVC	0x00080000	/* Invalid op for Compare */
#define FPSCR_FR	0x00040000	/* Fraction rounded */
#define FPSCR_FI	0x00020000	/* Fraction inexact */
#define FPSCR_FPRF	0x0001f000	/* FPU Result Flags */
#define FPSCR_FPCC	0x0000f000	/* FPU Condition Codes */
#define FPSCR_VXSOFT	0x00000400	/* Invalid op for software request */
#define FPSCR_VXSQRT	0x00000200	/* Invalid op for square root */
#define FPSCR_VXCVI	0x00000100	/* Invalid op for integer convert */
#define FPSCR_VE	0x00000080	/* Invalid op exception enable */
#define FPSCR_OE	0x00000040	/* IEEE overflow exception enable */
#define FPSCR_UE	0x00000020	/* IEEE underflow exception enable */
#define FPSCR_ZE	0x00000010	/* IEEE zero divide exception enable */
#define FPSCR_XE	0x00000008	/* FP inexact exception enable */
#define FPSCR_NI	0x00000004	/* FPU non IEEE-Mode */
#define FPSCR_RN	0x00000003	/* FPU rounding control */

/* Bit definitions for SPEFSCR. */
#define SPEFSCR_SOVH	0x80000000	/* Summary integer overflow high */
#define SPEFSCR_OVH	0x40000000	/* Integer overflow high */
#define SPEFSCR_FGH	0x20000000	/* Embedded FP guard bit high */
#define SPEFSCR_FXH	0x10000000	/* Embedded FP sticky bit high */
#define SPEFSCR_FINVH	0x08000000	/* Embedded FP invalid operation high */
#define SPEFSCR_FDBZH	0x04000000	/* Embedded FP div by zero high */
#define SPEFSCR_FUNFH	0x02000000	/* Embedded FP underflow high */
#define SPEFSCR_FOVFH	0x01000000	/* Embedded FP overflow high */
#define SPEFSCR_FINXS	0x00200000	/* Embedded FP inexact sticky */
#define SPEFSCR_FINVS	0x00100000	/* Embedded FP invalid op. sticky */
#define SPEFSCR_FDBZS	0x00080000	/* Embedded FP div by zero sticky */
#define SPEFSCR_FUNFS	0x00040000	/* Embedded FP underflow sticky */
#define SPEFSCR_FOVFS	0x00020000	/* Embedded FP overflow sticky */
#define SPEFSCR_MODE	0x00010000	/* Embedded FP mode */
#define SPEFSCR_SOV	0x00008000	/* Integer summary overflow */
#define SPEFSCR_OV	0x00004000	/* Integer overflow */
#define SPEFSCR_FG	0x00002000	/* Embedded FP guard bit */
#define SPEFSCR_FX	0x00001000	/* Embedded FP sticky bit */
#define SPEFSCR_FINV	0x00000800	/* Embedded FP invalid operation */
#define SPEFSCR_FDBZ	0x00000400	/* Embedded FP div by zero */
#define SPEFSCR_FUNF	0x00000200	/* Embedded FP underflow */
#define SPEFSCR_FOVF	0x00000100	/* Embedded FP overflow */
#define SPEFSCR_FINXE	0x00000040	/* Embedded FP inexact enable */
#define SPEFSCR_FINVE	0x00000020	/* Embedded FP invalid op. enable */
#define SPEFSCR_FDBZE	0x00000010	/* Embedded FP div by zero enable */
#define SPEFSCR_FUNFE	0x00000008	/* Embedded FP underflow enable */
#define SPEFSCR_FOVFE	0x00000004	/* Embedded FP overflow enable */
#define SPEFSCR_FRMC 	0x00000003	/* Embedded FP rounding mode control */

/* Special Purpose Registers (SPRNs)*/
#define SPRN_CTR	0x009	/* Count Register */
#define SPRN_DSCR	0x11
#define SPRN_CTRLF	0x088
#define SPRN_CTRLT	0x098
#define   CTRL_CT	0xc0000000	/* current thread */
#define   CTRL_CT0	0x80000000	/* thread 0 */
#define   CTRL_CT1	0x40000000	/* thread 1 */
#define   CTRL_TE	0x00c00000	/* thread enable */
#define   CTRL_RUNLATCH	0x1
#define SPRN_DABR	0x3F5	/* Data Address Breakpoint Register */
#define   DABR_TRANSLATION	(1UL << 2)
#define   DABR_DATA_WRITE	(1UL << 1)
#define   DABR_DATA_READ	(1UL << 0)
#define SPRN_DABR2	0x13D	/* e300 */
#define SPRN_DABRX	0x3F7	/* Data Address Breakpoint Register Extension */
#define   DABRX_USER	(1UL << 0)
#define   DABRX_KERNEL	(1UL << 1)
#define SPRN_DAR	0x013	/* Data Address Register */
#define SPRN_DBCR	0x136	/* e300 Data Breakpoint Control Reg */
#define SPRN_DSISR	0x012	/* Data Storage Interrupt Status Register */
#define   DSISR_NOHPTE		0x40000000	/* no translation found */
#define   DSISR_PROTFAULT	0x08000000	/* protection fault */
#define   DSISR_ISSTORE		0x02000000	/* access was a store */
#define   DSISR_DABRMATCH	0x00400000	/* hit data breakpoint */
#define   DSISR_NOSEGMENT	0x00200000	/* STAB/SLB miss */
#define SPRN_TBRL	0x10C	/* Time Base Read Lower Register (user, R/O) */
#define SPRN_TBRU	0x10D	/* Time Base Read Upper Register (user, R/O) */
#define SPRN_TBWL	0x11C	/* Time Base Lower Register (super, R/W) */
#define SPRN_TBWU	0x11D	/* Time Base Upper Register (super, R/W) */
#define SPRN_SPURR	0x134	/* Scaled PURR */
#define SPRN_HIOR	0x137	/* 970 Hypervisor interrupt offset */
#define SPRN_LPCR	0x13E	/* LPAR Control Register */
#define SPRN_DBAT0L	0x219	/* Data BAT 0 Lower Register */
#define SPRN_DBAT0U	0x218	/* Data BAT 0 Upper Register */
#define SPRN_DBAT1L	0x21B	/* Data BAT 1 Lower Register */
#define SPRN_DBAT1U	0x21A	/* Data BAT 1 Upper Register */
#define SPRN_DBAT2L	0x21D	/* Data BAT 2 Lower Register */
#define SPRN_DBAT2U	0x21C	/* Data BAT 2 Upper Register */
#define SPRN_DBAT3L	0x21F	/* Data BAT 3 Lower Register */
#define SPRN_DBAT3U	0x21E	/* Data BAT 3 Upper Register */
#define SPRN_DBAT4L	0x239	/* Data BAT 4 Lower Register */
#define SPRN_DBAT4U	0x238	/* Data BAT 4 Upper Register */
#define SPRN_DBAT5L	0x23B	/* Data BAT 5 Lower Register */
#define SPRN_DBAT5U	0x23A	/* Data BAT 5 Upper Register */
#define SPRN_DBAT6L	0x23D	/* Data BAT 6 Lower Register */
#define SPRN_DBAT6U	0x23C	/* Data BAT 6 Upper Register */
#define SPRN_DBAT7L	0x23F	/* Data BAT 7 Lower Register */
#define SPRN_DBAT7U	0x23E	/* Data BAT 7 Upper Register */

#define SPRN_DEC	0x016		/* Decrement Register */
#define SPRN_DER	0x095		/* Debug Enable Regsiter */
#define DER_RSTE	0x40000000	/* Reset Interrupt */
#define DER_CHSTPE	0x20000000	/* Check Stop */
#define DER_MCIE	0x10000000	/* Machine Check Interrupt */
#define DER_EXTIE	0x02000000	/* External Interrupt */
#define DER_ALIE	0x01000000	/* Alignment Interrupt */
#define DER_PRIE	0x00800000	/* Program Interrupt */
#define DER_FPUVIE	0x00400000	/* FP Unavailable Interrupt */
#define DER_DECIE	0x00200000	/* Decrementer Interrupt */
#define DER_SYSIE	0x00040000	/* System Call Interrupt */
#define DER_TRE		0x00020000	/* Trace Interrupt */
#define DER_SEIE	0x00004000	/* FP SW Emulation Interrupt */
#define DER_ITLBMSE	0x00002000	/* Imp. Spec. Instruction TLB Miss */
#define DER_ITLBERE	0x00001000	/* Imp. Spec. Instruction TLB Error */
#define DER_DTLBMSE	0x00000800	/* Imp. Spec. Data TLB Miss */
#define DER_DTLBERE	0x00000400	/* Imp. Spec. Data TLB Error */
#define DER_LBRKE	0x00000008	/* Load/Store Breakpoint Interrupt */
#define DER_IBRKE	0x00000004	/* Instruction Breakpoint Interrupt */
#define DER_EBRKE	0x00000002	/* External Breakpoint Interrupt */
#define DER_DPIE	0x00000001	/* Dev. Port Nonmaskable Request */
#define SPRN_DMISS	0x3D0		/* Data TLB Miss Register */
#define SPRN_EAR	0x11A		/* External Address Register */
#define SPRN_HASH1	0x3D2		/* Primary Hash Address Register */
#define SPRN_HASH2	0x3D3		/* Secondary Hash Address Resgister */
#define SPRN_HID0	0x3F0		/* Hardware Implementation Register 0 */
#define HID0_EMCP	(1<<31)		/* Enable Machine Check pin */
#define HID0_EBA	(1<<29)		/* Enable Bus Address Parity */
#define HID0_EBD	(1<<28)		/* Enable Bus Data Parity */
#define HID0_SBCLK	(1<<27)
#define HID0_EICE	(1<<26)
#define HID0_TBEN	(1<<26)		/* Timebase enable - 745x */
#define HID0_ECLK	(1<<25)
#define HID0_PAR	(1<<24)
#define HID0_STEN	(1<<24)		/* Software table search enable - 745x */
#define HID0_HIGH_BAT	(1<<23)		/* Enable high BATs - 7455 */
#define HID0_DOZE	(1<<23)
#define HID0_NAP	(1<<22)
#define HID0_SLEEP	(1<<21)
#define HID0_DPM	(1<<20)
#define HID0_BHTCLR	(1<<18)		/* Clear branch history table - 7450 */
#define HID0_XAEN	(1<<17)		/* Extended addressing enable - 7450 */
#define HID0_NHR	(1<<16)		/* Not hard reset (software bit-7450)*/
#define HID0_ICE	(1<<15)		/* Instruction Cache Enable */
#define HID0_DCE	(1<<14)		/* Data Cache Enable */
#define HID0_ILOCK	(1<<13)		/* Instruction Cache Lock */
#define HID0_DLOCK	(1<<12)		/* Data Cache Lock */
#define HID0_ICFI	(1<<11)		/* Instr. Cache Flash Invalidate */
#define HID0_DCI	(1<<10)		/* Data Cache Invalidate */
#define HID0_SPD	(1<<9)		/* Speculative disable */
#define HID0_DAPUEN	(1<<8)		/* Debug APU enable */
#define HID0_SGE	(1<<7)		/* Store Gathering Enable */
#define HID0_SIED	(1<<7)		/* Serial Instr. Execution [Disable] */
#define HID0_DCFA	(1<<6)		/* Data Cache Flush Assist */
#define HID0_LRSTK	(1<<4)		/* Link register stack - 745x */
#define HID0_BTIC	(1<<5)		/* Branch Target Instr Cache Enable */
#define HID0_ABE	(1<<3)		/* Address Broadcast Enable */
#define HID0_FOLD	(1<<3)		/* Branch Folding enable - 745x */
#define HID0_BHTE	(1<<2)		/* Branch History Table Enable */
#define HID0_BTCD	(1<<1)		/* Branch target cache disable */
#define HID0_NOPDST	(1<<1)		/* No-op dst, dstt, etc. instr. */
#define HID0_NOPTI	(1<<0)		/* No-op dcbt and dcbst instr. */

#define SPRN_HID1	0x3F1		/* Hardware Implementation Register 1 */
#define HID1_EMCP	(1<<31)		/* 7450 Machine Check Pin Enable */
#define HID1_DFS	(1<<22)		/* 7447A Dynamic Frequency Scaling */
#define HID1_PC0	(1<<16)		/* 7450 PLL_CFG[0] */
#define HID1_PC1	(1<<15)		/* 7450 PLL_CFG[1] */
#define HID1_PC2	(1<<14)		/* 7450 PLL_CFG[2] */
#define HID1_PC3	(1<<13)		/* 7450 PLL_CFG[3] */
#define HID1_SYNCBE	(1<<11)		/* 7450 ABE for sync, eieio */
#define HID1_ABE	(1<<10)		/* 7450 Address Broadcast Enable */
#define HID1_PS		(1<<16)		/* 750FX PLL selection */
#define SPRN_HID2	0x3F8		/* Hardware Implementation Register 2 */
#define SPRN_IABR	0x3F2	/* Instruction Address Breakpoint Register */
#define SPRN_IABR2	0x3FA		/* 83xx */
#define SPRN_IBCR	0x135		/* 83xx Insn Breakpoint Control Reg */
#define SPRN_HID4	0x3F4		/* 970 HID4 */
#define SPRN_HID5	0x3F6		/* 970 HID5 */
#define SPRN_HID6	0x3F9	/* BE HID 6 */
#define   HID6_LB	(0x0F<<12) /* Concurrent Large Page Modes */
#define   HID6_DLP	(1<<20)	/* Disable all large page modes (4K only) */
#define SPRN_TSC_CELL	0x399	/* Thread switch control on Cell */
#define   TSC_CELL_DEC_ENABLE_0	0x400000 /* Decrementer Interrupt */
#define   TSC_CELL_DEC_ENABLE_1	0x200000 /* Decrementer Interrupt */
#define   TSC_CELL_EE_ENABLE	0x100000 /* External Interrupt */
#define   TSC_CELL_EE_BOOST	0x080000 /* External Interrupt Boost */
#define SPRN_TSC 	0x3FD	/* Thread switch control on others */
#define SPRN_TST 	0x3FC	/* Thread switch timeout on others */
#if !defined(SPRN_IAC1) && !defined(SPRN_IAC2)
#define SPRN_IAC1	0x3F4		/* Instruction Address Compare 1 */
#define SPRN_IAC2	0x3F5		/* Instruction Address Compare 2 */
#endif
#define SPRN_IBAT0L	0x211		/* Instruction BAT 0 Lower Register */
#define SPRN_IBAT0U	0x210		/* Instruction BAT 0 Upper Register */
#define SPRN_IBAT1L	0x213		/* Instruction BAT 1 Lower Register */
#define SPRN_IBAT1U	0x212		/* Instruction BAT 1 Upper Register */
#define SPRN_IBAT2L	0x215		/* Instruction BAT 2 Lower Register */
#define SPRN_IBAT2U	0x214		/* Instruction BAT 2 Upper Register */
#define SPRN_IBAT3L	0x217		/* Instruction BAT 3 Lower Register */
#define SPRN_IBAT3U	0x216		/* Instruction BAT 3 Upper Register */
#define SPRN_IBAT4L	0x231		/* Instruction BAT 4 Lower Register */
#define SPRN_IBAT4U	0x230		/* Instruction BAT 4 Upper Register */
#define SPRN_IBAT5L	0x233		/* Instruction BAT 5 Lower Register */
#define SPRN_IBAT5U	0x232		/* Instruction BAT 5 Upper Register */
#define SPRN_IBAT6L	0x235		/* Instruction BAT 6 Lower Register */
#define SPRN_IBAT6U	0x234		/* Instruction BAT 6 Upper Register */
#define SPRN_IBAT7L	0x237		/* Instruction BAT 7 Lower Register */
#define SPRN_IBAT7U	0x236		/* Instruction BAT 7 Upper Register */
#define SPRN_ICMP	0x3D5		/* Instruction TLB Compare Register */
#define SPRN_ICTC	0x3FB	/* Instruction Cache Throttling Control Reg */
#define SPRN_ICTRL	0x3F3	/* 1011 7450 icache and interrupt ctrl */
#define ICTRL_EICE	0x08000000	/* enable icache parity errs */
#define ICTRL_EDC	0x04000000	/* enable dcache parity errs */
#define ICTRL_EICP	0x00000100	/* enable icache par. check */
#define SPRN_IMISS	0x3D4		/* Instruction TLB Miss Register */
#define SPRN_IMMR	0x27E		/* Internal Memory Map Register */
#define SPRN_L2CR	0x3F9		/* Level 2 Cache Control Regsiter */
#define SPRN_L2CR2	0x3f8
#define L2CR_L2E		0x80000000	/* L2 enable */
#define L2CR_L2PE		0x40000000	/* L2 parity enable */
#define L2CR_L2SIZ_MASK		0x30000000	/* L2 size mask */
#define L2CR_L2SIZ_256KB	0x10000000	/* L2 size 256KB */
#define L2CR_L2SIZ_512KB	0x20000000	/* L2 size 512KB */
#define L2CR_L2SIZ_1MB		0x30000000	/* L2 size 1MB */
#define L2CR_L2CLK_MASK		0x0e000000	/* L2 clock mask */
#define L2CR_L2CLK_DISABLED	0x00000000	/* L2 clock disabled */
#define L2CR_L2CLK_DIV1		0x02000000	/* L2 clock / 1 */
#define L2CR_L2CLK_DIV1_5	0x04000000	/* L2 clock / 1.5 */
#define L2CR_L2CLK_DIV2		0x08000000	/* L2 clock / 2 */
#define L2CR_L2CLK_DIV2_5	0x0a000000	/* L2 clock / 2.5 */
#define L2CR_L2CLK_DIV3		0x0c000000	/* L2 clock / 3 */
#define L2CR_L2RAM_MASK		0x01800000	/* L2 RAM type mask */
#define L2CR_L2RAM_FLOW		0x00000000	/* L2 RAM flow through */
#define L2CR_L2RAM_PIPE		0x01000000	/* L2 RAM pipelined */
#define L2CR_L2RAM_PIPE_LW	0x01800000	/* L2 RAM pipelined latewr */
#define L2CR_L2DO		0x00400000	/* L2 data only */
#define L2CR_L2I		0x00200000	/* L2 global invalidate */
#define L2CR_L2CTL		0x00100000	/* L2 RAM control */
#define L2CR_L2WT		0x00080000	/* L2 write-through */
#define L2CR_L2TS		0x00040000	/* L2 test support */
#define L2CR_L2OH_MASK		0x00030000	/* L2 output hold mask */
#define L2CR_L2OH_0_5		0x00000000	/* L2 output hold 0.5 ns */
#define L2CR_L2OH_1_0		0x00010000	/* L2 output hold 1.0 ns */
#define L2CR_L2SL		0x00008000	/* L2 DLL slow */
#define L2CR_L2DF		0x00004000	/* L2 differential clock */
#define L2CR_L2BYP		0x00002000	/* L2 DLL bypass */
#define L2CR_L2IP		0x00000001	/* L2 GI in progress */
#define L2CR_L2IO_745x		0x00100000	/* L2 instr. only (745x) */
#define L2CR_L2DO_745x		0x00010000	/* L2 data only (745x) */
#define L2CR_L2REP_745x		0x00001000	/* L2 repl. algorithm (745x) */
#define L2CR_L2HWF_745x		0x00000800	/* L2 hardware flush (745x) */
#define SPRN_L3CR		0x3FA	/* Level 3 Cache Control Regsiter */
#define L3CR_L3E		0x80000000	/* L3 enable */
#define L3CR_L3PE		0x40000000	/* L3 data parity enable */
#define L3CR_L3APE		0x20000000	/* L3 addr parity enable */
#define L3CR_L3SIZ		0x10000000	/* L3 size */
#define L3CR_L3CLKEN		0x08000000	/* L3 clock enable */
#define L3CR_L3RES		0x04000000	/* L3 special reserved bit */
#define L3CR_L3CLKDIV		0x03800000	/* L3 clock divisor */
#define L3CR_L3IO		0x00400000	/* L3 instruction only */
#define L3CR_L3SPO		0x00040000	/* L3 sample point override */
#define L3CR_L3CKSP		0x00030000	/* L3 clock sample point */
#define L3CR_L3PSP		0x0000e000	/* L3 P-clock sample point */
#define L3CR_L3REP		0x00001000	/* L3 replacement algorithm */
#define L3CR_L3HWF		0x00000800	/* L3 hardware flush */
#define L3CR_L3I		0x00000400	/* L3 global invalidate */
#define L3CR_L3RT		0x00000300	/* L3 SRAM type */
#define L3CR_L3NIRCA		0x00000080	/* L3 non-integer ratio clock adj. */
#define L3CR_L3DO		0x00000040	/* L3 data only mode */
#define L3CR_PMEN		0x00000004	/* L3 private memory enable */
#define L3CR_PMSIZ		0x00000001	/* L3 private memory size */

#define SPRN_MSSCR0	0x3f6	/* Memory Subsystem Control Register 0 */
#define SPRN_MSSSR0	0x3f7	/* Memory Subsystem Status Register 1 */
#define SPRN_LDSTCR	0x3f8	/* Load/Store control register */
#define SPRN_LDSTDB	0x3f4	/* */
#define SPRN_LR		0x008	/* Link Register */
#ifndef SPRN_PIR
#define SPRN_PIR	0x3FF	/* Processor Identification Register */
#endif
#define SPRN_PTEHI	0x3D5	/* 981 7450 PTE HI word (S/W TLB load) */
#define SPRN_PTELO	0x3D6	/* 982 7450 PTE LO word (S/W TLB load) */
#define SPRN_PURR	0x135	/* Processor Utilization of Resources Reg */
#define SPRN_PVR	0x11F	/* Processor Version Register */
#define SPRN_RPA	0x3D6	/* Required Physical Address Register */
#define SPRN_SDA	0x3BF	/* Sampled Data Address Register */
#define SPRN_SDR1	0x019	/* MMU Hash Base Register */
#define SPRN_ASR	0x118   /* Address Space Register */
#define SPRN_SIA	0x3BB	/* Sampled Instruction Address Register */
#define SPRN_SPRG0	0x110	/* Special Purpose Register General 0 */
#define SPRN_SPRG1	0x111	/* Special Purpose Register General 1 */
#define SPRN_SPRG2	0x112	/* Special Purpose Register General 2 */
#define SPRN_SPRG3	0x113	/* Special Purpose Register General 3 */
#define SPRN_SPRG4	0x114	/* Special Purpose Register General 4 */
#define SPRN_SPRG5	0x115	/* Special Purpose Register General 5 */
#define SPRN_SPRG6	0x116	/* Special Purpose Register General 6 */
#define SPRN_SPRG7	0x117	/* Special Purpose Register General 7 */
#define SPRN_SRR0	0x01A	/* Save/Restore Register 0 */
#define SPRN_SRR1	0x01B	/* Save/Restore Register 1 */
#define   SRR1_WAKEMASK		0x00380000 /* reason for wakeup */
#define   SRR1_WAKERESET	0x00380000 /* System reset */
#define   SRR1_WAKESYSERR	0x00300000 /* System error */
#define   SRR1_WAKEEE		0x00200000 /* External interrupt */
#define   SRR1_WAKEMT		0x00280000 /* mtctrl */
#define   SRR1_WAKEDEC		0x00180000 /* Decrementer interrupt */
#define   SRR1_WAKETHERM	0x00100000 /* Thermal management interrupt */
#define SPRN_HSRR0	0x13A	/* Save/Restore Register 0 */
#define SPRN_HSRR1	0x13B	/* Save/Restore Register 1 */

#define SPRN_TBCTL	0x35f	/* PA6T Timebase control register */
#define   TBCTL_FREEZE		0x0000000000000000ull /* Freeze all tbs */
#define   TBCTL_RESTART		0x0000000100000000ull /* Restart all tbs */
#define   TBCTL_UPDATE_UPPER	0x0000000200000000ull /* Set upper 32 bits */
#define   TBCTL_UPDATE_LOWER	0x0000000300000000ull /* Set lower 32 bits */

#ifndef SPRN_SVR
#define SPRN_SVR	0x11E	/* System Version Register */
#endif
#define SPRN_THRM1	0x3FC		/* Thermal Management Register 1 */
/* these bits were defined in inverted endian sense originally, ugh, confusing */
#define THRM1_TIN	(1 << 31)
#define THRM1_TIV	(1 << 30)
#define THRM1_THRES(x)	((x&0x7f)<<23)
#define THRM3_SITV(x)	((x&0x3fff)<<1)
#define THRM1_TID	(1<<2)
#define THRM1_TIE	(1<<1)
#define THRM1_V		(1<<0)
#define SPRN_THRM2	0x3FD		/* Thermal Management Register 2 */
#define SPRN_THRM3	0x3FE		/* Thermal Management Register 3 */
#define THRM3_E		(1<<0)
#define SPRN_TLBMISS	0x3D4		/* 980 7450 TLB Miss Register */
#define SPRN_UMMCR0	0x3A8	/* User Monitor Mode Control Register 0 */
#define SPRN_UMMCR1	0x3AC	/* User Monitor Mode Control Register 0 */
#define SPRN_UPMC1	0x3A9	/* User Performance Counter Register 1 */
#define SPRN_UPMC2	0x3AA	/* User Performance Counter Register 2 */
#define SPRN_UPMC3	0x3AD	/* User Performance Counter Register 3 */
#define SPRN_UPMC4	0x3AE	/* User Performance Counter Register 4 */
#define SPRN_USIA	0x3AB	/* User Sampled Instruction Address Register */
#define SPRN_VRSAVE	0x100	/* Vector Register Save Register */
#define SPRN_XER	0x001	/* Fixed Point Exception Register */

#define SPRN_SCOMC	0x114	/* SCOM Access Control */
#define SPRN_SCOMD	0x115	/* SCOM Access DATA */

