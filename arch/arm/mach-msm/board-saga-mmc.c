/* linux/arch/arm/mach-msm/board-saga-mmc.c
 *
 * Copyright (C) 2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <mach/htc_fast_clk.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>

#include <asm/mach/mmc.h>

#include "devices.h"
#include "board-saga.h"
#include "proc_comm.h"

#define SAGA_SDMC_CD_N_SYS	PM8058_GPIO_PM_TO_SYS(SAGA_SDMC_CD_N)

#define DEBUG_SDSLOT_VDD 0

/*extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);
*/
/* ---- COMMON ---- */
static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for (n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

/* ---- SDCARD ---- */

static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(58, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* CLK */
	PCOM_GPIO_CFG(59, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(60, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(58, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(59, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(60, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static uint opt_disable_sdcard;

static uint32_t movinand_on_gpio_table[] = {
	PCOM_GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(69, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */
	PCOM_GPIO_CFG(115, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT4 */
	PCOM_GPIO_CFG(114, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT5 */
	PCOM_GPIO_CFG(113, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT6 */
	PCOM_GPIO_CFG(112, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT7 */
};

static int __init saga_disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_saga.disable_sdcard=", saga_disablesdcard_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

static struct mmc_vdd_xlat mmc_vdd_table[] = {
	{ MMC_VDD_165_195,	1800 },
	{ MMC_VDD_20_21,	2050 },
	{ MMC_VDD_21_22,	2150 },
	{ MMC_VDD_22_23,	2250 },
	{ MMC_VDD_23_24,	2350 },
	{ MMC_VDD_24_25,	2450 },
	{ MMC_VDD_25_26,	2550 },
	{ MMC_VDD_26_27,	2650 },
	{ MMC_VDD_27_28,	2750 },
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2950 },
};

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t saga_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i, rc;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
#if DEBUG_SDSLOT_VDD
		printk(KERN_DEBUG "%s: Disabling SD slot power\n", __func__);
#endif
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		rc = vreg_enable(vreg_sdslot);
		if (rc) {
			printk(KERN_ERR "%s: Error enabling vreg (%d)\n",
			       __func__, rc);
		}
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
#if DEBUG_SDSLOT_VDD
			printk(KERN_DEBUG "%s: Setting level to %u\n",
				__func__, mmc_vdd_table[i].level);
#endif
			rc = vreg_set_level(vreg_sdslot,
					    mmc_vdd_table[i].level);
			if (rc) {
				printk(KERN_ERR
				       "%s: Error setting vreg level (%d)\n",
				       __func__, rc);
			}
			return 0;
		}
	}

	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int saga_sdslot_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) gpio_get_value(SAGA_SDMC_CD_N_SYS);
	return !status;
}

#define SAGA_MMC_VDD		(MMC_VDD_28_29 | MMC_VDD_29_30)

static unsigned int saga_sdslot_type = MMC_TYPE_SD;

static struct mmc_platform_data saga_sdslot_data = {
	.ocr_mask	= SAGA_MMC_VDD,
	.status_irq	= MSM_GPIO_TO_INT(SAGA_SDMC_CD_N_SYS),
	.status		= saga_sdslot_status,
	.translate_vdd	= saga_sdslot_switchvdd,
	.slot_type	= &saga_sdslot_type,
	.dat0_gpio	= 69,
};

static unsigned int saga_emmcslot_type = MMC_TYPE_MMC;
static struct mmc_platform_data saga_movinand_data = {
	.ocr_mask	= SAGA_MMC_VDD,
	.slot_type	= &saga_emmcslot_type,
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
};

/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(110, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(116, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(110, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static struct vreg *vreg_wifi_osc;	/* WIFI 32khz oscilator */
static int saga_wifi_cd = 0;	/* WIFI virtual 'card detect' status */

static struct sdio_embedded_func wifi_func = {
	.f_class	= SDIO_CLASS_WLAN,
	.f_maxblksize	= 512,
};

static struct embedded_sdio_data saga_wifi_emb_data = {
	.cis	= {
		.vendor		= 0x104c,
		.device		= 0x9066,
		.blksize	= 512,
		.max_dtr	= 20000000,
	},
	.cccr	= {
		.multi_block	= 0,
		.low_speed	= 0,
		.wide_bus	= 1,
		.high_power	= 0,
		.high_speed	= 0,
	},
	.funcs	= &wifi_func,
	.num_funcs = 1,
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int saga_wifi_status_register(void (*callback)(int card_present,
							  void *dev_id),
					 void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int saga_wifi_status(struct device *dev)
{
	return saga_wifi_cd;
}

int saga_wifi_set_carddetect(int val)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, val);
	saga_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(saga_wifi_set_carddetect);
#endif

int saga_wifi_power_state=0;
int saga_bt_power_state=0;

static struct pm_gpio pmic_gpio_sleep_clk_output = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM8058_GPIO_VIN_S3,      /* S3 1.8 V */
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_2,
};

static DEFINE_SPINLOCK(saga_w_b_slock);
int saga_sleep_clk_state_wifi = CLK_OFF;
int saga_sleep_clk_state_bt = CLK_OFF;

int saga_wifi_bt_sleep_clk_ctl(int on, int id)
{
	int err = 0;
	unsigned long flags;

	printk(KERN_DEBUG "%s ON=%d, ID=%d\n", __func__, on, id);

	spin_lock_irqsave(&saga_w_b_slock, flags);
	if (on) {
		if ((CLK_OFF == saga_sleep_clk_state_wifi)
			&& (CLK_OFF == saga_sleep_clk_state_bt)) {
			printk(KERN_DEBUG "EN SLEEP CLK\n");
			pmic_gpio_sleep_clk_output.function = PM_GPIO_FUNC_2;
			err = pm8xxx_gpio_config(SAGA_WIFI_SLOW_CLK,
					&pmic_gpio_sleep_clk_output);
			if (err) {
				spin_unlock_irqrestore(&saga_w_b_slock,
							flags);
				printk(KERN_ERR "ERR EN SLEEP CLK, ERR=%d\n",
					err);
				return err;
			}
		}

		if (id == ID_BT)
			saga_sleep_clk_state_bt = CLK_ON;
		else
			saga_sleep_clk_state_wifi = CLK_ON;
	} else {
		if (((id == ID_BT) && (CLK_OFF == saga_sleep_clk_state_wifi))
			|| ((id == ID_WIFI)
			&& (CLK_OFF == saga_sleep_clk_state_bt))) {
			printk(KERN_DEBUG "DIS SLEEP CLK\n");
			pmic_gpio_sleep_clk_output.function =
						PM_GPIO_FUNC_NORMAL;
			err = pm8xxx_gpio_config(
					SAGA_WIFI_SLOW_CLK,
					&pmic_gpio_sleep_clk_output);
			if (err) {
				spin_unlock_irqrestore(&saga_w_b_slock,
							flags);
				printk(KERN_ERR "ERR DIS SLEEP CLK, ERR=%d\n",
					err);
				return err;
			}
		} else {
			printk(KERN_DEBUG "KEEP SLEEP CLK ALIVE\n");
		}

		if (id)
			saga_sleep_clk_state_bt = CLK_OFF;
		else
			saga_sleep_clk_state_wifi = CLK_OFF;
	}
	spin_unlock_irqrestore(&saga_w_b_slock, flags);

	return 0;
}
EXPORT_SYMBOL(saga_wifi_bt_sleep_clk_ctl);

int saga_wifi_power(int on)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
	}

	saga_wifi_bt_sleep_clk_ctl(on, ID_WIFI);
	gpio_set_value(SAGA_GPIO_WIFI_SHUTDOWN_N, on); /* WIFI_SHUTDOWN */

	mdelay(120);
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(saga_wifi_power);
#endif

/* Eenable VREG_MMC pin to turn on fastclock oscillator : colin */
int saga_bt_fastclock_power(int on)
{
	int rc;

	printk(KERN_DEBUG "saga_bt_fastclock_power on = %d\n", on);
	if (vreg_wifi_osc) {
		if (on) {
			rc = vreg_enable(vreg_wifi_osc);
			printk(KERN_DEBUG "BT vreg_enable vreg_mmc, rc=%d\n",
			       rc);
			if (rc) {
				printk("Error turn saga_bt_fastclock_power rc=%d\n", rc);
				return rc;
			}
		} else {
			if (!saga_wifi_power_state) {
				vreg_disable(vreg_wifi_osc);
				printk(KERN_DEBUG "BT disable vreg_wifi_osc.\n");
			} else
				printk(KERN_DEBUG "BT shouldn't disable vreg_wifi_osc. WiFi is using it!!\n");
		}
	}
	saga_bt_power_state = on;
	return 0;
}
EXPORT_SYMBOL(saga_bt_fastclock_power);

static int saga_wifi_reset_state;
void saga_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(saga_wifi_reset);
#endif

static struct mmc_platform_data saga_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= saga_wifi_status,
	.register_status_notify	= saga_wifi_status_register,
	.embedded_sdio		= &saga_wifi_emb_data,
};

int __init saga_init_mmc(unsigned int sys_rev)
{
	uint32_t id;
	wifi_status_cb = NULL;
	sdslot_vreg_enabled = 0;

	printk(KERN_INFO "saga: %s\n", __func__);
	/* SDC2: MoviNAND */
//	register_msm_irq_mask(INT_SDC2_0);
//	register_msm_irq_mask(INT_SDC2_1);
	config_gpio_table(movinand_on_gpio_table,
			  ARRAY_SIZE(movinand_on_gpio_table));
	msm_add_sdcc(2, &saga_movinand_data);

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(SAGA_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_OUTPUT,
		GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	gpio_set_value(SAGA_GPIO_WIFI_SHUTDOWN_N, 0);

	msm_add_sdcc(3, &saga_wifi_data);

//	register_msm_irq_mask(INT_SDC4_0);
//	register_msm_irq_mask(INT_SDC4_1);
	/* SDCC4: SD card */
	if (opt_disable_sdcard) {
		printk(KERN_INFO "saga: SD-Card interface disabled\n");
		return 0;
	}

	vreg_sdslot = vreg_get(0, "gp10");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	irq_set_irq_wake(MSM_GPIO_TO_INT(SAGA_SDMC_CD_N_SYS), 1);

	msm_add_sdcc(4, &saga_sdslot_data);
	return 0;
}

#if defined(CONFIG_DEBUG_FS)
static int sagammc_dbg_wifi_reset_set(void *data, u64 val)
{
	saga_wifi_reset((int) val);
	return 0;
}

static int sagammc_dbg_wifi_reset_get(void *data, u64 *val)
{
	*val = saga_wifi_reset_state;
	return 0;
}

static int sagammc_dbg_wifi_cd_set(void *data, u64 val)
{
	saga_wifi_set_carddetect((int) val);
	return 0;
}

static int sagammc_dbg_wifi_cd_get(void *data, u64 *val)
{
	*val = saga_wifi_cd;
	return 0;
}

static int sagammc_dbg_wifi_pwr_set(void *data, u64 val)
{
	saga_wifi_power((int) val);
	return 0;
}

static int sagammc_dbg_wifi_pwr_get(void *data, u64 *val)
{

	*val = saga_wifi_power_state;
	return 0;
}

static int sagammc_dbg_sd_pwr_set(void *data, u64 val)
{
	saga_sdslot_switchvdd(NULL, (unsigned int) val);
	return 0;
}

static int sagammc_dbg_sd_pwr_get(void *data, u64 *val)
{
	*val = sdslot_vdd;
	return 0;
}

static int sagammc_dbg_sd_cd_set(void *data, u64 val)
{
	return -ENOSYS;
}

static int sagammc_dbg_sd_cd_get(void *data, u64 *val)
{
	*val = saga_sdslot_status(NULL);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sagammc_dbg_wifi_reset_fops,
			sagammc_dbg_wifi_reset_get,
			sagammc_dbg_wifi_reset_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(sagammc_dbg_wifi_cd_fops,
			sagammc_dbg_wifi_cd_get,
			sagammc_dbg_wifi_cd_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(sagammc_dbg_wifi_pwr_fops,
			sagammc_dbg_wifi_pwr_get,
			sagammc_dbg_wifi_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(sagammc_dbg_sd_pwr_fops,
			sagammc_dbg_sd_pwr_get,
			sagammc_dbg_sd_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(sagammc_dbg_sd_cd_fops,
			sagammc_dbg_sd_cd_get,
			sagammc_dbg_sd_cd_set, "%llu\n");

static int __init sagammc_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("sagammc_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("wifi_reset", 0644, dent, NULL,
			    &sagammc_dbg_wifi_reset_fops);
	debugfs_create_file("wifi_cd", 0644, dent, NULL,
			    &sagammc_dbg_wifi_cd_fops);
	debugfs_create_file("wifi_pwr", 0644, dent, NULL,
			    &sagammc_dbg_wifi_pwr_fops);

	debugfs_create_file("sd_pwr", 0644, dent, NULL,
			    &sagammc_dbg_sd_pwr_fops);
	debugfs_create_file("sd_cd", 0644, dent, NULL,
			    &sagammc_dbg_sd_cd_fops);

	return 0;
}

device_initcall(sagammc_dbg_init);

#endif
