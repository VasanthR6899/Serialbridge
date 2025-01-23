
#ifndef WILC_SBRIDGE_H
#define WILC_SBRIDGE_H

#include <linux/types.h>
#include <linux/bitfield.h>


#define DEV_MAX 	2

/*wlan*/
#define GPIO_NUM_CHIP_EN	94
#define GPIO_NUM_RESET		60



enum wilc_chip_type {
	WILC_1000,
	WILC_3000,
};

struct wilc_power_gpios {
	int reset;
	int chip_en;
};

struct wilc_power {
	struct wilc_power_gpios gpios;
	u8 status[DEV_MAX];
};

struct wilc {
	struct device *dev;
	struct device *dt_dev;
	int io_type;
	enum wilc_chip_type chip;
	struct mutex hif_cs;
	struct wilc_power power;
	void *bus_data;
	u32 chipid;

};

struct sdio_cmd52 {
	u32 read_write:		1;
	u32 function:		3;
	u32 raw:		1;
	u32 address:		17;
	u32 data:		8;
};

struct sdio_cmd53 {
	u32 read_write:		1;
	u32 function:		3;
	u32 block_mode:		1;
	u32 increment:		1;
	u32 address:		17;
	u32 count:		9;
	u8 *buffer;
	u32 block_size;
	bool use_global_buf;

};

enum {
	WILC_HIF_SDIO = 0,
	WILC_HIF_SPI = BIT(0),
	WILC_HIF_SDIO_GPIO_IRQ = BIT(1)
};

static inline bool is_wilc1000(u32 id)
{
	return ((id & 0xfffff000) == 0x100000 ? true : false);
}

static inline bool is_wilc3000(u32 id)
{
	return ((id & 0xfffff000) == 0x300000 ? true : false);
}

enum bus_acquire {
	WILC_BUS_ACQUIRE_ONLY = 0,
	WILC_BUS_ACQUIRE_AND_WAKEUP = 1,
};

enum bus_release {
	WILC_BUS_RELEASE_ONLY = 0,
	WILC_BUS_RELEASE_ALLOW_SLEEP = 1,
};

/* Functions IO enables bits */
#define WILC_SDIO_CCCR_IO_EN_FUNC1	BIT(1)

/* Function/Interrupt enables bits */
#define WILC_SDIO_CCCR_IEN_MASTER	BIT(0)
#define WILC_SDIO_CCCR_IEN_FUNC1	BIT(1)

/* Abort CCCR register bits */
#define WILC_SDIO_CCCR_ABORT_RESET	BIT(3)

/* Vendor specific CCCR registers */
/* WILC1000 */
#define WILC1000_SDIO_WAKEUP_REG	0xf0
#define WILC1000_SDIO_WAKEUP_BIT	BIT(0)

#define WILC1000_SDIO_CLK_STATUS_REG	0xf1
#define WILC1000_SDIO_CLK_STATUS_BIT	BIT(0)

#define WILC1000_SDIO_IRQ_FLAG_REG	0xf7
#define WILC1000_SDIO_IRQ_CLEAR_FLAG_REG	0xf8

/* WILC3000 specific */
#define WILC3000_SDIO_WAKEUP_REG	0xf0
#define WILC3000_SDIO_WAKEUP_BIT	BIT(0)

#define WILC3000_SDIO_CLK_STATUS_REG	0xf0 /* clk & wakeup are on same reg*/
#define WILC3000_SDIO_CLK_STATUS_BIT	BIT(4)

#define WILC3000_SDIO_IRQ_FLAG_REG	0xfe
#define WILC3000_SDIO_IRQ_CLEAR_FLAG_REG	0xfe
#define WILC3000_SDIO_VMM_TBL_CTRL_REG	0xf1

/* Common vendor specific CCCR register */
#define WILC_SDIO_INTERRUPT_DATA_SZ_REG	0xf2 /* Read size (2 bytes) */
#define WILC_SDIO_HOST_TO_FW_REG	0xfa
#define WILC_SDIO_HOST_TO_FW_BIT	BIT(0)

#define WILC_SDIO_FW_TO_HOST_REG	0xfc
#define WILC_SDIO_FW_TO_HOST_BIT	BIT(0)

#define WILC_SPI_REG_BASE		0xe800
#define WILC_SPI_CTL			WILC_SPI_REG_BASE
#define WILC_SPI_PROTOCOL_CONFIG	(WILC_SPI_REG_BASE + 0x24)

#define WILC_SPI_PROTOCOL_OFFSET	(WILC_SPI_PROTOCOL_CONFIG - \
					 WILC_SPI_REG_BASE)

#define MODALIAS		"WILC_SPI"


/* Function 1 specific FBR register */
#define WILC_SDIO_FBR_CSA_REG		0x10C /* CSA pointer (3 bytes) */
#define WILC_SDIO_FBR_DATA_REG		0x10F

#define WILC_SDIO_F1_DATA_REG		0x0
#define WILC_SDIO_EXT_IRQ_FLAG_REG	0x4

#define WILC_AHB_DATA_MEM_BASE		0x30000
#define WILC_AHB_SHARE_MEM_BASE		0xd0000

#define WILC_VMM_TBL_RX_SHADOW_BASE	WILC_AHB_SHARE_MEM_BASE
#define WILC_VMM_TBL_RX_SHADOW_SIZE	256

#define WILC_GP_REG_0			0x149c
#define WILC_GP_REG_1			0x14a0

#define GLOBAL_MODE_CONTROL		0x1614
#define PWR_SEQ_MISC_CTRL		0x3008

#define WILC_GLOBAL_MODE_ENABLE_WIFI	BIT(0)
#define WILC_PWR_SEQ_ENABLE_WIFI_SLEEP	BIT(28)

#define COE_AUTO_PS_ON_NULL_PKT		0x160468
#define COE_AUTO_PS_OFF_NULL_PKT	0x16046C
#define CCA_CTL_2 (0x160EF4)
#define CCA_CTL_7 (0x160F08)

#define WILC_HAVE_SDIO_IRQ_GPIO		BIT(0)
#define WILC_HAVE_SLEEP_CLK_SRC_RTC	BIT(2)
#define WILC_HAVE_SLEEP_CLK_SRC_XO	BIT(3)



/*******************************************/
/*        E0 and later Interrupt flags.    */
/*******************************************/
/*******************************************/
/*        E0 and later Interrupt flags.    */
/*           IRQ Status word               */
/* 15:0 = DMA count in words.              */
/* 16: INT0 flag                           */
/* 17: INT1 flag                           */
/* 18: INT2 flag                           */
/* 19: INT3 flag                           */
/* 20: INT4 flag                           */
/* 21: INT5 flag                           */
/*******************************************/
#define IRG_FLAGS_OFFSET	16
#define IRQ_DMA_WD_CNT_MASK	GENMASK(IRG_FLAGS_OFFSET - 1, 0)
#define INT_0			BIT(IRG_FLAGS_OFFSET)
#define INT_1			BIT(IRG_FLAGS_OFFSET + 1)
#define INT_2			BIT(IRG_FLAGS_OFFSET + 2)
#define INT_3			BIT(IRG_FLAGS_OFFSET + 3)
#define INT_4			BIT(IRG_FLAGS_OFFSET + 4)
#define MAX_NUM_INT		5
#define IRG_FLAGS_MASK		GENMASK(IRG_FLAGS_OFFSET + MAX_NUM_INT, \
					IRG_FLAGS_OFFSET)

/*******************************************/
/*        E0 and later Interrupt flags.    */
/*           IRQ Clear word                */
/* 0: Clear INT0                           */
/* 1: Clear INT1                           */
/* 2: Clear INT2                           */
/* 3: Clear INT3                           */
/* 4: Clear INT4                           */
/* 5: Clear INT5                           */
/* 6: Select VMM table 1                   */
/* 7: Select VMM table 2                   */
/* 8: Enable VMM                           */
/*******************************************/
#define CLR_INT0		BIT(0)
#define CLR_INT1		BIT(1)
#define CLR_INT2		BIT(2)
#define CLR_INT3		BIT(3)
#define CLR_INT4		BIT(4)
#define CLR_INT5		BIT(5)
#define SEL_VMM_TBL0		BIT(6)
#define SEL_VMM_TBL1		BIT(7)
#define EN_VMM			BIT(8)

#define DATA_INT_EXT		INT_0
#define ALL_INT_EXT		DATA_INT_EXT
#define NUM_INT_EXT		1
#define UNHANDLED_IRQ_MASK	GENMASK(MAX_NUM_INT - 1, NUM_INT_EXT)


/********************************************
 *
 *      Register Defines
 *
 ********************************************/
#define WILC_PERIPH_REG_BASE		0x1000
#define WILC_CHIPID			WILC_PERIPH_REG_BASE
#define WILC_GLB_RESET_0		(WILC_PERIPH_REG_BASE + 0x400)
#define WILC_PIN_MUX_0			(WILC_PERIPH_REG_BASE + 0x408)
#define WILC_HOST_TX_CTRL		(WILC_PERIPH_REG_BASE + 0x6c)
#define WILC_HOST_RX_CTRL_0		(WILC_PERIPH_REG_BASE + 0x70)
#define WILC_HOST_RX_CTRL_1		(WILC_PERIPH_REG_BASE + 0x74)
#define WILC_HOST_VMM_CTL		(WILC_PERIPH_REG_BASE + 0x78)
#define WILC_HOST_RX_CTRL		(WILC_PERIPH_REG_BASE + 0x80)
#define WILC_HOST_RX_EXTRA_SIZE		(WILC_PERIPH_REG_BASE + 0x84)
#define WILC_HOST_TX_CTRL_1		(WILC_PERIPH_REG_BASE + 0x88)
#define WILC_MISC			(WILC_PERIPH_REG_BASE + 0x428)
#define WILC_INTR_REG_BASE		(WILC_PERIPH_REG_BASE + 0xa00)

#define WILC_INTERRUPT_CORTUS_0		(WILC_PERIPH_REG_BASE + 0xa8)
#define WILC1000_CORTUS_INTERRUPT_1	(WILC_INTERRUPT_CORTUS_0 + 0x4)
#define WILC3000_CORTUS_INTERRUPT_1	(WILC_INTERRUPT_CORTUS_0 + 0x14)


#define WILC_MISC		(WILC_PERIPH_REG_BASE + 0x428)
#define WILC_INTR_ENABLE		WILC_INTR_REG_BASE
#define WILC_INTR2_ENABLE		(WILC_INTR_REG_BASE + 4)

#define WILC_CORTUS_RESET_MUX_SEL	0x1118
#define WILC_CORTUS_BOOT_REGISTER	0xc0000
#define WILC3000_CHIP_ID		0x3b0000


#define WILC_1000_BASE_ID		0x100000

#define WILC_1000_BASE_ID_2A		0x1002A0
#define WILC_1000_BASE_ID_2A_REV1	(WILC_1000_BASE_ID_2A + 1)

#define WILC_1000_BASE_ID_2B		0x1002B0
#define WILC_1000_BASE_ID_2B_REV1	(WILC_1000_BASE_ID_2B + 1)
#define WILC_1000_BASE_ID_2B_REV2	(WILC_1000_BASE_ID_2B + 2)

#define WILC_3000_BASE_ID		0x300000

#define WILC_CHIP_REV_FIELD		GENMASK(11, 0)
#define WILC_RF_REVISION_ID		0x13f4


#endif /** WILC_SBRIDGE_H */

