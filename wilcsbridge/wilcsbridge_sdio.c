// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#include <linux/clk.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/sdio.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/module.h>
//#include "netdev.h"
//#include "cfg80211.h"
#include "wilc_sbridge.h"



#define SDIO_VENDOR_ID_MICROCHIP_WILC		0x0296
#define SDIO_DEVICE_ID_MICROCHIP_WILC1000	0x5347


/* ioctl old */

typedef struct  {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;

}cmd_hdr;

typedef struct {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;
	uint8_t b_buffer[4*1024];
}block_cmd_hdr;

#define CMD_READ_REG 		_IOR('q', 1, cmd_hdr *)
#define CMD_WRITE_REG 		_IOW('q', 2, cmd_hdr *)
#define CMD_READ_BLOCK_REG 	_IOR('q', 3, cmd_hdr *)
#define CMD_WRITE_BLOCK_REG 	_IOW('q', 4, cmd_hdr *)

#define FIRST_MINOR 0
#define MINOR_CNT 1

static dev_t dev;
static struct cdev c_dev;
static struct class *cl;
struct wilc *wilc;
struct device *dt_dev= NULL;

static int query_ioctl_init( void );
static void query_ioctl_exit(void );
/*****/

enum sdio_host_lock {
	WILC_SDIO_HOST_NO_TAKEN = 0,
	WILC_SDIO_HOST_IRQ_TAKEN = 1,
	WILC_SDIO_HOST_DIS_TAKEN = 2,
};

static enum sdio_host_lock	sdio_intr_lock = WILC_SDIO_HOST_NO_TAKEN;
static wait_queue_head_t sdio_intr_waitqueue;

#define SDIO_MODALIAS "wilcsbridge_sdio"

static const struct sdio_device_id wilc_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_MICROCHIP_WILC, SDIO_DEVICE_ID_MICROCHIP_WILC1000) },
	{ },
};
MODULE_DEVICE_TABLE(sdio, wilc_sdio_ids);

#define WILC_SDIO_BLOCK_SIZE 512

struct wilc_sdio {
	bool irq_gpio;
	u32 block_size;
	struct wilc *wl;
	u8 *cmd53_buf;
};

static int wilc_sdio_cmd52(struct wilc *wilc, struct sdio_cmd52 *cmd)
{
	struct sdio_func *func = container_of(wilc->dev, struct sdio_func, dev);
	int ret;
	u8 data;

	sdio_claim_host(func);

	func->num = cmd->function;
	if (cmd->read_write) {  /* write */
		if (cmd->raw) {
			sdio_writeb(func, cmd->data, cmd->address, &ret);
			data = sdio_readb(func, cmd->address, &ret);
			cmd->data = data;
		} else {
			sdio_writeb(func, cmd->data, cmd->address, &ret);
		}
	} else {        /* read */
		data = sdio_readb(func, cmd->address, &ret);
		cmd->data = data;
	}

	sdio_release_host(func);

	if (ret)
		dev_err(&func->dev, "%s..failed, err(%d)\n", __func__, ret);
	return ret;
}

static int wilc_sdio_cmd53(struct wilc *wilc, struct sdio_cmd53 *cmd)
{
	struct sdio_func *func = container_of(wilc->dev, struct sdio_func, dev);
	int size, ret;
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	u8 *buf = cmd->buffer;

	sdio_claim_host(func);

	func->num = cmd->function;
	func->cur_blksize = cmd->block_size;
	if (cmd->block_mode)
		size = cmd->count * cmd->block_size;
	else
		size = cmd->count;

	if (cmd->use_global_buf) {
		if (size > sizeof(u32)) {
			ret = -EINVAL;
			goto out;
		}
		buf = sdio_priv->cmd53_buf;
	}

	if (cmd->read_write) {  /* write */
		if (cmd->use_global_buf)
			memcpy(buf, cmd->buffer, size);

		ret = sdio_memcpy_toio(func, cmd->address, buf, size);
	} else {        /* read */
		ret = sdio_memcpy_fromio(func, buf, cmd->address, size);

		if (cmd->use_global_buf)
			memcpy(cmd->buffer, buf, size);
	}
out:
	sdio_release_host(func);

	if (ret)
		dev_err(&func->dev, "%s..failed, err(%d)\n", __func__,  ret);

	return ret;
}


/**
 * wilc_of_parse_power_pins() - parse power sequence pins; to keep backward
 *		compatibility with old device trees that doesn't provide
 *		power sequence pins we check for default pins on proper boards
 *
 * @wilc:	wilc data structure
 *
 * Returns:	 0 on success, negative error number on failures.
 */
int wilc_of_parse_power_pins(struct wilc *wilc)
{
	static const struct wilc_power_gpios default_gpios[] = {
		{ .reset = GPIO_NUM_RESET,	.chip_en = GPIO_NUM_CHIP_EN, },
	};
	struct device_node *of = wilc->dt_dev->of_node;
	struct wilc_power *power = &wilc->power;
	const struct wilc_power_gpios *gpios = &default_gpios[0];
	int ret;

	power->gpios.reset = of_get_named_gpio(of, "reset-gpios", 0);
	if (!gpio_is_valid(power->gpios.reset))
		power->gpios.reset = gpios->reset;

	power->gpios.chip_en = of_get_named_gpio(of, "chip_en-gpios", 0);
	if (!gpio_is_valid(power->gpios.chip_en))
		power->gpios.chip_en = gpios->chip_en;

	if (!gpio_is_valid(power->gpios.chip_en) ||
			!gpio_is_valid(power->gpios.reset))
		return -EINVAL;

	ret = devm_gpio_request(wilc->dev, power->gpios.chip_en, "CHIP_EN");
	if (ret)
		return ret;

	ret = devm_gpio_request(wilc->dev, power->gpios.reset, "RESET");
	return ret;
}

/**
 * wilc_wlan_power() - handle power on/off commands
 *
 * @wilc:	wilc data structure
 * @on:		requested power status
 *
 * Returns:	none
 */
void wilc_wlan_power(struct wilc *wilc, bool on)
{
	if (!gpio_is_valid(wilc->power.gpios.chip_en) ||
	    !gpio_is_valid(wilc->power.gpios.reset)) {
		/* In case SDIO power sequence driver is used to power this
		 * device then the powering sequence is handled by the bus
		 * via pm_runtime_* functions. */
		return;
	}

	if (on) {
		gpio_direction_output(wilc->power.gpios.chip_en, 1);
		mdelay(5);
		gpio_direction_output(wilc->power.gpios.reset, 1);
	} else {
		gpio_direction_output(wilc->power.gpios.chip_en, 0);
		gpio_direction_output(wilc->power.gpios.reset, 0);
	}
}


/********************************************
 *
 *      Function 0
 *
 ********************************************/

static int wilc_sdio_set_func0_csa_address(struct wilc *wilc, u32 adr)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct sdio_cmd52 cmd;
	int ret;

	/**
	 *      Review: BIG ENDIAN
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = WILC_SDIO_FBR_CSA_REG;
	cmd.data = (u8)adr;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set %04x data...\n",
			cmd.address);
		return ret;
	}

	cmd.address = WILC_SDIO_FBR_CSA_REG + 1;
	cmd.data = (u8)(adr >> 8);
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set %04x data...\n",
			cmd.address);
		return ret;
	}

	cmd.address = WILC_SDIO_FBR_CSA_REG + 2;
	cmd.data = (u8)(adr >> 16);
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set %04x data...\n",
			cmd.address);
		return ret;
	}

	return 0;
}

static int wilc_sdio_set_block_size(struct wilc *wilc, u8 func_num,
				    u32 block_size)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct sdio_cmd52 cmd;
	int ret;

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = SDIO_FBR_BASE(func_num) + SDIO_CCCR_BLKSIZE;
	cmd.data = (u8)block_size;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set %04x data...\n",
			cmd.address);
		return ret;
	}

	cmd.address = SDIO_FBR_BASE(func_num) + SDIO_CCCR_BLKSIZE +  1;
	cmd.data = (u8)(block_size >> 8);
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set %04x data...\n",
			cmd.address);
		return ret;
	}

	return 0;
}




static int wilc_sdio_write(struct wilc *wilc, u32 addr, u8 *buf, u32 size)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	u32 block_size = sdio_priv->block_size;
	struct sdio_cmd53 cmd;
	int nblk, nleft, ret;

	cmd.read_write = 1;
	if (addr > 0) {
		/**
		 *      func 0 access
		 **/
		cmd.function = 0;
		cmd.address = WILC_SDIO_FBR_DATA_REG;
	} else {
		/**
		 *      func 1 access
		 **/
		cmd.function = 1;
		cmd.address = WILC_SDIO_F1_DATA_REG;
	}

	size = ALIGN(size, 4);
	nblk = size / block_size;
	nleft = size % block_size;

	cmd.use_global_buf = false;
	if (nblk > 0) {
		cmd.block_mode = 1;
		cmd.increment = 1;
		cmd.count = nblk;
		cmd.buffer = buf;
		cmd.block_size = block_size;
		if (addr > 0) {
			ret = wilc_sdio_set_func0_csa_address(wilc, addr);
			if (ret)
				return ret;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], block send...\n", addr);
			return ret;
		}
		if (addr > 0)
			addr += nblk * block_size;
		buf += nblk * block_size;
	}

	if (nleft > 0) {
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = nleft;
		cmd.buffer = buf;

		cmd.block_size = block_size;

		if (addr > 0) {
			ret = wilc_sdio_set_func0_csa_address(wilc, addr);
			if (ret)
				return ret;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], bytes send...\n", addr);
			return ret;
		}
	}

	return 0;
}

static int wilc_sdio_read_reg(struct wilc *wilc, u32 addr, u32 *data)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	int ret;

	if (addr >= 0xf0 && addr <= 0xff) { /* only vendor specific registers */
		struct sdio_cmd52 cmd;

		cmd.read_write = 0;
		cmd.function = 0;
		cmd.raw = 0;
		cmd.address = addr;
		ret = wilc_sdio_cmd52(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd 52, read reg (%08x) ...\n", addr);
			return ret;
		}
		*data = cmd.data;
	} else {
		struct sdio_cmd53 cmd;

		ret = wilc_sdio_set_func0_csa_address(wilc, addr);
		if (ret)
			return ret;

		cmd.read_write = 0;
		cmd.function = 0;
		cmd.address = WILC_SDIO_FBR_DATA_REG;
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = sizeof(u32);
		cmd.buffer = (u8 *)data;
		cmd.use_global_buf = true;

		cmd.block_size = sdio_priv->block_size;
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53, read reg (%08x)...\n", addr);
			return ret;
		}
	}

	le32_to_cpus(data);
	return 0;
}

/********************************************
 *
 *      Sdio interfaces
 *
 ********************************************/
static int wilc_sdio_write_reg(struct wilc *wilc, u32 addr, u32 data)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	int ret;

	cpu_to_le32s(&data);

	if (addr >= 0xf0 && addr <= 0xff) { /* only vendor specific registers */
		struct sdio_cmd52 cmd;

		cmd.read_write = 1;
		cmd.function = 0;
		cmd.raw = 0;
		cmd.address = addr;
		cmd.data = data;
		ret = wilc_sdio_cmd52(wilc, &cmd);
		if (ret)
			dev_err(&func->dev,
				"Failed cmd 52, write reg (%08x) ...\n", addr);
	} else {
		struct sdio_cmd53 cmd;

		/**
		 *      set the AHB address
		 **/
		ret = wilc_sdio_set_func0_csa_address(wilc, addr);
		if (ret)
			return ret;

		cmd.read_write = 1;
		cmd.function = 0;
		cmd.address = WILC_SDIO_FBR_DATA_REG;
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = sizeof(u32);
		cmd.buffer = (u8 *)&data;
		cmd.use_global_buf = true;
		cmd.block_size = sdio_priv->block_size;
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret)
			dev_err(&func->dev,
				"Failed cmd53, write reg (%08x)...\n", addr);
	}

	return ret;
}


static int wilc_sdio_read(struct wilc *wilc, u32 addr, u8 *buf, u32 size)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	u32 block_size = sdio_priv->block_size;
	struct sdio_cmd53 cmd;
	int nblk, nleft, ret;

	cmd.read_write = 0;
	if (addr > 0) {
		/**
		 *      func 0 access
		 **/
		cmd.function = 0;
		cmd.address = WILC_SDIO_FBR_DATA_REG;
	} else {
		/**
		 *      func 1 access
		 **/
		cmd.function = 1;
		cmd.address = WILC_SDIO_F1_DATA_REG;
	}

	size = ALIGN(size, 4);
	nblk = size / block_size;
	nleft = size % block_size;

	cmd.use_global_buf = false;
	if (nblk > 0) {
		cmd.block_mode = 1;
		cmd.increment = 1;
		cmd.count = nblk;
		cmd.buffer = buf;
		cmd.block_size = block_size;
		if (addr > 0) {
			ret = wilc_sdio_set_func0_csa_address(wilc, addr);
			if (ret)
				return ret;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], block read...\n", addr);
			return ret;
		}
		if (addr > 0)
			addr += nblk * block_size;
		buf += nblk * block_size;
	}       /* if (nblk > 0) */

	if (nleft > 0) {
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = nleft;
		cmd.buffer = buf;

		cmd.block_size = block_size;

		if (addr > 0) {
			ret = wilc_sdio_set_func0_csa_address(wilc, addr);
			if (ret)
				return ret;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], bytes read...\n", addr);
			return ret;
		}
	}

	return 0;
}

u32 wilc_get_chipid(struct wilc *wilc, bool update)
{
	int ret;
	u32 chipid = 0;
	u32 rfrevid = 0;

	if (wilc->chipid == 0 || update) {
		ret = wilc_sdio_read_reg(wilc, WILC3000_CHIP_ID,
						   &chipid);
		if (!is_wilc3000(chipid)) {
			wilc_sdio_read_reg(wilc, WILC_CHIPID, &chipid);
			wilc_sdio_read_reg(wilc, WILC_RF_REVISION_ID,
						     &rfrevid);

			if (!is_wilc1000(chipid)) {
				wilc->chipid = 0;
				return wilc->chipid;
			}
			if (chipid == WILC_1000_BASE_ID_2A) { /* 0x1002A0 */
				if (rfrevid != 0x1)
					chipid = WILC_1000_BASE_ID_2A_REV1;
			} else if (chipid == WILC_1000_BASE_ID_2B) { /* 0x1002B0 */
				if (rfrevid == 0x4)
					chipid = WILC_1000_BASE_ID_2B_REV1;
				else if (rfrevid != 0x3)
					chipid = WILC_1000_BASE_ID_2B_REV2;
			}
		}
		wilc->chipid = chipid;
	}
	return wilc->chipid;
}

int init_wilc_chip(struct wilc *wilc)
{
	u32 chipid;
	u32 reg, ret = 0;
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);

	chipid = wilc_get_chipid(wilc,true);
	printk("WILC ChipID: %x \n",chipid);

	ret = wilc_sdio_read_reg(wilc,WILC_CORTUS_RESET_MUX_SEL, &reg);
	if (ret) {
		dev_err(&func->dev, "fail read reg 0x1118\n");
		return ret;
	}
	reg |= BIT(0);
	ret = wilc_sdio_write_reg(wilc,WILC_CORTUS_RESET_MUX_SEL, reg);
	if (ret) {
		dev_err(&func->dev, "fail write reg 0x1118\n");
		return ret;
	}
	ret = wilc_sdio_write_reg(wilc, WILC_CORTUS_BOOT_REGISTER, 0x71);
	if (ret) {
		dev_err(&func->dev, "fail write reg 0xc0000\n");
		return ret;
	}		
	//if (is_wilc3000(chipid)) {
		ret = wilc_sdio_read_reg(wilc, 0x207ac, &reg);

		ret = wilc_sdio_write_reg(wilc, 0x4f0000,0x71);
		if (ret) {
			dev_err(&func->dev, "fail write reg 0x4f0000\n");
			return ret;
		}
	//}	
	return ret;
}





/********************************************
 *
 *      Bus interfaces
 *
 ********************************************/

static int wilc_sdio_deinit(struct wilc *wilc)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);

	pm_runtime_put_sync_autosuspend(mmc_dev(func->card->host));
	wilc_wlan_power(wilc, false);

	return 0;
}

static int wilc_sdio_init(struct wilc *wilc, bool resume)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	struct sdio_cmd52 cmd;
	int loop, ret;
	u32 chipid;

	dev_info(&func->dev, "SDIO speed: %d\n",
		func->card->host->ios.clock);

	/* Patch for sdio interrupt latency issue */
	ret = pm_runtime_get_sync(mmc_dev(func->card->host));
	if (ret < 0) {
		pm_runtime_put_noidle(mmc_dev(func->card->host));
		return ret;
	}

    init_waitqueue_head(&sdio_intr_waitqueue);
    sdio_priv->irq_gpio = (wilc->io_type == WILC_HIF_SDIO_GPIO_IRQ);
    
	/**
	 *      function 0 csa enable
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = SDIO_FBR_BASE(1);
	cmd.data = SDIO_FBR_ENABLE_CSA;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Fail cmd 52, enable csa...\n");
		goto pm_runtime_put;
	}

	/**
	 *      function 0 block size
	 **/
	ret = wilc_sdio_set_block_size(wilc, 0, WILC_SDIO_BLOCK_SIZE);
	if (ret) {
		dev_err(&func->dev, "Fail cmd 52, set func 0 block size...\n");
		goto pm_runtime_put;
	}
	sdio_priv->block_size = WILC_SDIO_BLOCK_SIZE;

	/**
	 *      enable func1 IO
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = SDIO_CCCR_IOEx;
	cmd.data = WILC_SDIO_CCCR_IO_EN_FUNC1;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev,
			"Fail cmd 52, set IOE register...\n");
		goto pm_runtime_put;
	}

	/**
	 *      make sure func 1 is up
	 **/
	cmd.read_write = 0;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = SDIO_CCCR_IORx;
	loop = 3;
	do {
		cmd.data = 0;
		ret = wilc_sdio_cmd52(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Fail cmd 52, get IOR register...\n");
			goto pm_runtime_put;
		}
		if (cmd.data == WILC_SDIO_CCCR_IO_EN_FUNC1)
			break;
	} while (loop--);

	if (loop <= 0) {
		dev_err(&func->dev, "Fail func 1 is not ready...\n");
		goto pm_runtime_put;
	}

	/**
	 *      func 1 is ready, set func 1 block size
	 **/
	ret = wilc_sdio_set_block_size(wilc, 1, WILC_SDIO_BLOCK_SIZE);
	if (ret) {
		dev_err(&func->dev, "Fail set func 1 block size...\n");
		goto pm_runtime_put;
	}

	/**
	 *      func 1 interrupt enable
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = SDIO_CCCR_IENx;
	cmd.data = WILC_SDIO_CCCR_IEN_MASTER | WILC_SDIO_CCCR_IEN_FUNC1;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Fail cmd 52, set IEN register...\n");
		goto pm_runtime_put;
	}

	/**
	 *      make sure can read back chip id correctly
	 **/
	if (!resume) {
		chipid = wilc_get_chipid(wilc, true);
		if (is_wilc3000(chipid)) {
			wilc->chip = WILC_3000;
		} else if (is_wilc1000(chipid)) {
			wilc->chip = WILC_1000;
		} else {
			dev_err(&func->dev, "Unsupported chipid: %x\n", chipid);
			goto pm_runtime_put;
		}
		dev_info(&func->dev, "chipid %08x\n", chipid);
	}

	return 0;

pm_runtime_put:
	pm_runtime_put_sync_autosuspend(mmc_dev(func->card->host));
	return ret;
}

static int wilc_sdio_read_size(struct wilc *wilc, u32 *size)
{
	u32 tmp;
	struct sdio_cmd52 cmd;

	/**
	 *      Read DMA count in words
	 **/
	cmd.read_write = 0;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = WILC_SDIO_INTERRUPT_DATA_SZ_REG;
	cmd.data = 0;
	wilc_sdio_cmd52(wilc, &cmd);
	tmp = cmd.data;

	cmd.address = WILC_SDIO_INTERRUPT_DATA_SZ_REG + 1;
	cmd.data = 0;
	wilc_sdio_cmd52(wilc, &cmd);
	tmp |= (cmd.data << 8);

	*size = tmp;
	return 0;
}

static int wilc_sdio_read_int(struct wilc *wilc, u32 *int_status)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	u32 tmp;
	struct sdio_cmd52 cmd;
	u32 irq_flags;

	if (sdio_priv->irq_gpio) {
		wilc_sdio_read_size(wilc, &tmp);

		cmd.read_write = 0;
		cmd.function = 0;
		cmd.raw = 0;
		cmd.data = 0;
		if (wilc->chip == WILC_1000) {
			cmd.address = WILC1000_SDIO_IRQ_FLAG_REG;
			wilc_sdio_cmd52(wilc, &cmd);
			irq_flags = cmd.data & 0x1f;
		} else {
			cmd.address = WILC3000_SDIO_IRQ_FLAG_REG;
			wilc_sdio_cmd52(wilc, &cmd);
			irq_flags = cmd.data & 0x0f;
		}
		tmp |= FIELD_PREP(IRG_FLAGS_MASK, cmd.data);

		*int_status = tmp;
	} else {
		wilc_sdio_read_size(wilc, &tmp);
		cmd.read_write = 0;
		cmd.function = 1;
		cmd.address = WILC_SDIO_EXT_IRQ_FLAG_REG;
		cmd.data = 0;
		wilc_sdio_cmd52(wilc, &cmd);

		irq_flags = cmd.data;
		tmp |= FIELD_PREP(IRG_FLAGS_MASK, cmd.data);

		if (FIELD_GET(UNHANDLED_IRQ_MASK, irq_flags)) {
			dev_err(&func->dev, "Unexpected interrupt (1) int=%lx\n",
				FIELD_GET(UNHANDLED_IRQ_MASK, irq_flags));
		}

		*int_status = tmp;
	}

	return 0;
}

static int wilc_sdio_clear_int_ext(struct wilc *wilc, u32 val)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	int ret;
	u32 reg = 0;

	if (wilc->chip == WILC_1000) {
		if (sdio_priv->irq_gpio)
			reg = val & (BIT(MAX_NUM_INT) - 1);

		/* select VMM table 0 */
		if (val & SEL_VMM_TBL0)
			reg |= BIT(5);
		/* select VMM table 1 */
		if (val & SEL_VMM_TBL1)
			reg |= BIT(6);
		/* enable VMM */
		if (val & EN_VMM)
			reg |= BIT(7);
		if (reg) {
			struct sdio_cmd52 cmd;

			cmd.read_write = 1;
			cmd.function = 0;
			cmd.raw = 0;
			cmd.address = WILC1000_SDIO_IRQ_CLEAR_FLAG_REG;
			cmd.data = reg;

			ret = wilc_sdio_cmd52(wilc, &cmd);
			if (ret) {
				dev_err(&func->dev,
					"Failed cmd52, set 0xf8 data (%d) ...\n",
					__LINE__);
				return ret;
			}
		}
	} else {
		if (sdio_priv->irq_gpio) {
			reg = val & (BIT(MAX_NUM_INT) - 1);
			if (reg) {
				struct sdio_cmd52 cmd;

				cmd.read_write = 1;
				cmd.function = 0;
				cmd.raw = 0;
				cmd.address = WILC3000_SDIO_IRQ_CLEAR_FLAG_REG;
				cmd.data = reg;

				ret = wilc_sdio_cmd52(wilc, &cmd);
				if (ret) {
					dev_err(&func->dev,
						"Failed cmd52, set 0xfe data (%d) ...\n",
						__LINE__);
					return ret;
				}
			}
		}
		reg = 0;
		/* select VMM table 0 */
		if (val & SEL_VMM_TBL0)
			reg |= BIT(0);
		/* select VMM table 1 */
		if (val & SEL_VMM_TBL1)
			reg |= BIT(1);
		/* enable VMM */
		if (val & EN_VMM)
			reg |= BIT(2);

		if (reg) {
			struct sdio_cmd52 cmd;

			cmd.read_write = 1;
			cmd.function = 0;
			cmd.raw = 0;
			cmd.address = WILC3000_SDIO_VMM_TBL_CTRL_REG;
			cmd.data = reg;

			ret = wilc_sdio_cmd52(wilc, &cmd);
			if (ret) {
				dev_err(&func->dev,
					"Failed cmd52, set 0xf6 data (%d) ...\n",
					__LINE__);
				return ret;
			}
		}
	}

	return 0;
}

static int wilc_sdio_sync_ext(struct wilc *wilc, int nint)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	u32 reg;
	int ret, i;

	if (nint > MAX_NUM_INT) {
		dev_err(&func->dev, "Too many interrupts %d\n", nint);
		return -EINVAL;
	}

/* WILC3000 only. Was removed in WILC1000 on revision 6200.
 * Might be related to suspend/resume
 */
	if (wilc->chip == WILC_3000) {
		/**
		 *      Disable power sequencer
		 **/
		if (wilc_sdio_read_reg(wilc, WILC_MISC, &reg)) {
			dev_err(&func->dev, "Failed read misc reg\n");
			return -EINVAL;
		}
		reg &= ~BIT(8);
		if (wilc_sdio_write_reg(wilc, WILC_MISC, reg)) {
			dev_err(&func->dev, "Failed write misc reg\n");
			return -EINVAL;
		}
	}

	if (sdio_priv->irq_gpio) {
		/**
		 *      interrupt pin mux select
		 **/
		ret = wilc_sdio_read_reg(wilc, WILC_PIN_MUX_0, &reg);
		if (ret) {
			dev_err(&func->dev, "Failed read reg (%08x)...\n",
				WILC_PIN_MUX_0);
			return ret;
		}
		reg |= BIT(8);
		ret = wilc_sdio_write_reg(wilc, WILC_PIN_MUX_0, reg);
		if (ret) {
			dev_err(&func->dev, "Failed write reg (%08x)...\n",
				WILC_PIN_MUX_0);
			return ret;
		}

		/**
		 *      interrupt enable
		 **/
		ret = wilc_sdio_read_reg(wilc, WILC_INTR_ENABLE, &reg);
		if (ret) {
			dev_err(&func->dev, "Failed read reg (%08x)...\n",
				WILC_INTR_ENABLE);
			return ret;
		}

		for (i = 0; (i < 5) && (nint > 0); i++, nint--)
			reg |= BIT((27 + i));
		ret = wilc_sdio_write_reg(wilc, WILC_INTR_ENABLE, reg);
		if (ret) {
			dev_err(&func->dev, "Failed write reg (%08x)...\n",
				WILC_INTR_ENABLE);
			return ret;
		}
		if (nint) {
			ret = wilc_sdio_read_reg(wilc, WILC_INTR2_ENABLE, &reg);
			if (ret) {
				dev_err(&func->dev,
					"Failed read reg (%08x)...\n",
					WILC_INTR2_ENABLE);
				return ret;
			}

			for (i = 0; (i < 3) && (nint > 0); i++, nint--)
				reg |= BIT(i);

			ret = wilc_sdio_write_reg(wilc, WILC_INTR2_ENABLE, reg);
			if (ret) {
				dev_err(&func->dev,
					"Failed write reg (%08x)...\n",
					WILC_INTR2_ENABLE);
				return ret;
			}
		}
	}
	return 0;
}

/**********ioctl */


static int my_open(struct inode *i, struct file *f)
{
    return 0;
}

static int my_close(struct inode *i, struct file *f)
{
    return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int my_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg)
#else
static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
#endif
{
    cmd_hdr dat;
    block_cmd_hdr b_dat;
    unsigned int val32=0;

    switch (cmd)
    {
        case CMD_READ_REG:
	    if (copy_from_user(&dat, (cmd_hdr *)arg, sizeof(cmd_hdr)))
            {
                return -EACCES;
            }
	    	wilc_sdio_read_reg(wilc,  dat.addr, &val32);
            dat.val = (int) val32;
            if (copy_to_user((cmd_hdr *)arg, &dat, sizeof(cmd_hdr)))
            {
                return -EACCES;
            }
            break;
	case CMD_WRITE_REG:
            if (copy_from_user(&dat, (cmd_hdr *)arg, sizeof(cmd_hdr)))
            {
                return -EACCES;
            }
	    	wilc_sdio_write_reg(wilc, dat.addr, dat.val);
            break;
        case CMD_WRITE_BLOCK_REG:
	    if (copy_from_user(&b_dat, (block_cmd_hdr *)arg, sizeof(block_cmd_hdr)))
           {
                return -EACCES;
           }
           wilc_sdio_write(wilc, b_dat.addr, b_dat.b_buffer, (b_dat.cmd>>16) & 0xFFFF);
           break;
	case CMD_READ_BLOCK_REG:
	    if (copy_from_user(&b_dat, (block_cmd_hdr *)arg, sizeof(block_cmd_hdr)))
            {
                return -EACCES;
            }
            wilc_sdio_read(wilc, b_dat.addr, b_dat.b_buffer, (b_dat.cmd>>16) & 0xFFFF);
            if (copy_to_user((block_cmd_hdr *)arg, &b_dat, sizeof(block_cmd_hdr)))
            {
                return -EACCES;
            }
            break;
        default:
            return -EINVAL;
    }
    return 0;
}
 

static struct file_operations query_fops =
{
    .owner = THIS_MODULE,
    .open = my_open,
    .release = my_close,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = my_ioctl
#else
    .unlocked_ioctl = my_ioctl
#endif
};
 
static int query_ioctl_init(void)
{
	int ret;
	struct device *dev_ret;
	
	if ((ret = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, "query_ioctl")) < 0)
	{
		printk("Failed alloc_chrdev %s %d \n",__func__,__LINE__);
		return ret;
	}

	cdev_init(&c_dev, &query_fops);
	c_dev.owner = THIS_MODULE;

	if ((ret = cdev_add(&c_dev, dev, MINOR_CNT)) < 0)
	{
		printk("Failed cdev_add %s %d \n",__func__,__LINE__);
		return ret;
	}

	if (IS_ERR(cl = class_create(THIS_MODULE,"wilcs_sdio")))
	{
		cdev_del(&c_dev);
		unregister_chrdev_region(dev, MINOR_CNT);
		printk("Failed class_create %s %d \n",__func__,__LINE__);
		return PTR_ERR(cl);
	}
	if (IS_ERR(dev_ret = device_create(cl, NULL, dev, NULL, "query")))
	{
		class_destroy(cl);
		cdev_del(&c_dev);
		unregister_chrdev_region(dev, MINOR_CNT);
		printk("Failed device_create %s %d",__func__,__LINE__);
		return PTR_ERR(dev_ret);
	}

	return 0;
}
 
static void query_ioctl_exit(void)
{
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&c_dev);
    unregister_chrdev_region(dev, MINOR_CNT);
}

static int wilc_sdio_probe(struct sdio_func *func,
			   const struct sdio_device_id *id)
{
    int ret, io_type;
    static bool init_power;
    struct wilc_sdio *sdio_priv;
    struct device_node *np;
    int irq_num;

	printk("Updated DRiver ###### \n");
    wilc = kzalloc(sizeof(*wilc), GFP_KERNEL);
	if (!wilc)
		return -ENOMEM;

    sdio_priv = kzalloc(sizeof(*sdio_priv), GFP_KERNEL);
    if (!sdio_priv) {
        ret = -ENOMEM;
		goto free;
	}

    sdio_priv->cmd53_buf = kzalloc(sizeof(u32), GFP_KERNEL);
    if (!sdio_priv->cmd53_buf) {
        ret = -ENOMEM;
        goto free;
    }

	if( query_ioctl_init() != 0 )
    {
        printk("ioctl init error\n");
    }
    

    sdio_set_drvdata(func, wilc);
    wilc->bus_data = sdio_priv;	
    wilc->dev = &func->dev;
    wilc->dt_dev = &func->card->dev;
    sdio_priv->wl = wilc;
	wilc_sdio_init(wilc,false);
	init_wilc_chip(wilc);

#if 0
   // wilc_of_parse_power_pins(wilc);
   //
   //
	/*
	 * Some WILC SDIO setups needs a SD power sequence driver to be able
	 * to power the WILC devices before reaching this function. For those
	 * devices the power sequence driver already provides reset-gpios
	 * and chip_en-gpios.
	 */
	np = of_parse_phandle(func->card->host->parent->of_node, "mmc-pwrseq",
			      0);
	if (np && of_device_is_available(np)) {
		init_power = 1;
		of_node_put(np);
	} else {
		ret = wilc_of_parse_power_pins(wilc);
		//if (ret)
		//	goto disable_rtc_clk;
	}
#endif
	
    if (!init_power) {
        wilc_wlan_power(wilc, false);
        init_power = 1;
        wilc_wlan_power(wilc, true);
    }	

    dev_info(&func->dev, "WILC SDIO Probe success \n");
    return 0;
free:
	if (wilc)
		kfree(wilc);
	if (sdio_priv)
		kfree(sdio_priv);
	
	return ret;
}

static void wilc_sdio_remove(struct sdio_func *func)
{
	printk("wilc_bus_remove SDIO card removed \n");
	query_ioctl_exit();
}

static int wilc_sdio_reset(struct wilc *wilc)
{
	struct sdio_cmd52 cmd;
	int ret;
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);

	dev_info(&func->dev, "De Init SDIO\n");

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = SDIO_CCCR_ABORT;
	cmd.data = WILC_SDIO_CCCR_ABORT_RESET;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Fail cmd 52, reset cmd ...\n");
		return ret;
	}
	return 0;
}

static int wilc_sdio_suspend(struct device *dev)
{
	return 0;
}

static int wilc_sdio_resume(struct device *dev)
{
	return 0;
}

static const struct of_device_id wilc_of_match[] = {
	{ .compatible = "microchip,wilc1000", },
	{ .compatible = "microchip,wilc3000", },
	{ /* sentinel */}
};
MODULE_DEVICE_TABLE(of, wilc_of_match);

static const struct dev_pm_ops wilc_sdio_pm_ops = {
	.suspend = wilc_sdio_suspend,
	.resume = wilc_sdio_resume,
};

static struct sdio_driver wilc_sdio_driver = {
	.name		= SDIO_MODALIAS,
	.id_table	= wilc_sdio_ids,
	.probe		= wilc_sdio_probe,
	.remove		= wilc_sdio_remove,
	.drv = {
		.pm = &wilc_sdio_pm_ops,
		.of_match_table = wilc_of_match,
	},
};
module_driver(wilc_sdio_driver,
	      sdio_register_driver,
	      sdio_unregister_driver);
MODULE_DESCRIPTION("Atmel WILC1000 SDIO wireless driver");
MODULE_LICENSE("GPL");
