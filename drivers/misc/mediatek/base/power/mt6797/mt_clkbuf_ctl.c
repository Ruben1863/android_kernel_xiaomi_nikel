/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*
 * @file    mt_clk_buf_ctl.c
 * @brief   Driver for RF clock buffer control
 *
 */

#define __MT_CLK_BUF_CTL_C__

/*
 * Include files
 */

/* system includes */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>

#include <mt-plat/sync_write.h>
#include <mt_spm.h>
#include <mach/mt_clkmgr.h>
/* #include <mach/mt_gpio_core.h> */
#include <mt_clkbuf_ctl.h>
#include <mt-plat/upmu_common.h>

#define RF_BPI_GPIO_NUM 20
#define MD_JTAG_GPIO_NUM 2

#define TAG     "[Power/clkbuf]"

#define clk_buf_err(fmt, args...)		pr_err(TAG fmt, ##args)
#define clk_buf_warn(fmt, args...)		pr_warn(TAG fmt, ##args)
#define clk_buf_warn_limit(fmt, args...)	pr_warn_ratelimited(TAG fmt, ##args)
#define clk_buf_dbg(fmt, args...)			\
	do {						\
		if (clkbuf_debug)			\
			pr_warn(TAG fmt, ##args);	\
	} while (0)

/*
 * LOCK
 */
DEFINE_MUTEX(clk_buf_ctrl_lock);

#define DEFINE_ATTR_RO(_name)			\
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = #_name,			\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
}

#define DEFINE_ATTR_RW(_name)			\
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = #_name,			\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define __ATTR_OF(_name)	(&_name##_attr.attr)

#define clkbuf_readl(addr)		DRV_Reg32(addr)
#define clkbuf_writel(addr, val)	mt_reg_sync_writel(val, addr)

static void __iomem *pwrap_base;

#ifdef ENABLE_PMIC_CLK_BUFFER

#define PWRAP_REG(ofs)		(pwrap_base + ofs)

#define DCXO_ENABLE		PWRAP_REG(0x18C)
#define DCXO_CONN_ADR0		PWRAP_REG(0x190)
#define DCXO_CONN_WDATA0	PWRAP_REG(0x194)
#define DCXO_CONN_ADR1		PWRAP_REG(0x198)
#define DCXO_CONN_WDATA1	PWRAP_REG(0x19C)
#define DCXO_NFC_ADR0		PWRAP_REG(0x1A0)
#define DCXO_NFC_WDATA0		PWRAP_REG(0x1A4)
#define DCXO_NFC_ADR1		PWRAP_REG(0x1A8)
#define DCXO_NFC_WDATA1		PWRAP_REG(0x1AC)

#define	DCXO_CONN_ENABLE	(1U << 1)
#define	DCXO_NFC_ENABLE		(1U << 0)

#define	DCXO_EXTBUF_EN_M	0
#define	DCXO_EN_BB		1
#define	DCXO_CLK_SEL		2
#define	DCXO_EN_BB_OR_CLK_SEL	3

#define PMIC_REG_MASK				0xFFFF
#define PMIC_REG_SHIFT				0

#define PMIC_CW00_ADDR				0x7000
#define PMIC_CW00_INIT_VAL			0x4DFD
#define PMIC_CW00_XO_EXTBUF2_MODE_MASK		0x3
#define PMIC_CW00_XO_EXTBUF2_MODE_SHIFT		3
#define PMIC_CW00_XO_EXTBUF2_EN_M_MASK		0x1
#define PMIC_CW00_XO_EXTBUF2_EN_M_SHIFT		5
#define PMIC_CW00_XO_EXTBUF3_MODE_MASK		0x3
#define PMIC_CW00_XO_EXTBUF3_MODE_SHIFT		6
#define PMIC_CW00_XO_BB_LPM_EN_MASK		0x1
#define PMIC_CW00_XO_BB_LPM_EN_SHIFT		12

/*
 * 0x701A	CW13	Code Word 13
 * 15:14	RG_XO_EXTBUF4_ISET	XO Control Signal of Current on EXTBUF4
 * 13:12	RG_XO_EXTBUF4_HD	XO Control Signal of EXTBUF4 Output driving Strength
 * 11:10	RG_XO_EXTBUF3_ISET	XO Control Signal of Current on EXTBUF3
 * 9:8		RG_XO_EXTBUF3_HD	XO Control Signal of EXTBUF3 Output driving Strength
 * 7:6		RG_XO_EXTBUF2_ISET	XO Control Signal of Current on EXTBUF2
 * 5:4		RG_XO_EXTBUF2_HD	XO Control Signal of EXTBUF2 Output driving Strength
 * 3:2		RG_XO_EXTBUF1_ISET	XO Control Signal of Current on EXTBUF1
 * 1:0		RG_XO_EXTBUF1_HD	XO Control Signal of EXTBUF1 Output driving Strength
 */
#define PMIC_CW13_ADDR				0x701A
#define PMIC_CW13_DEFAULT_VAL			0xAAAA
#define PMIC_CW13_XO_EXTBUF_HD_VAL		((0x2 << 0) | (0x2 << 4) \
						 | (0x2 << 8) | (0x2 << 12))

#define PMIC_CW14_ADDR				0x701C
#define PMIC_CW14_XO_EXTBUF3_EN_M_MASK		0x1
#define PMIC_CW14_XO_EXTBUF3_EN_M_SHIFT		11

#define PMIC_CW15_ADDR				0x701E
#define PMIC_CW15_DCXO_STATIC_AUXOUT_EN_MASK	0x1
#define PMIC_CW15_DCXO_STATIC_AUXOUT_EN_SHIFT	0
#define PMIC_CW15_DCXO_STATIC_AUXOUT_SEL_MASK	0xF
#define PMIC_CW15_DCXO_STATIC_AUXOUT_SEL_SHIFT	1
#endif /* ENABLE_PMIC_CLK_BUFFER */

/* FIXME: only for bring up Co-TSX before DT is ready */
/* #define CLKBUF_COTSX_BRINGUP */

static bool is_clkbuf_initiated;
static bool g_is_flightmode_on;
static bool clkbuf_debug;

struct delayed_work clkbuf_delayed_work;

/* false: rf_clkbuf, true: pmic_clkbuf */
static bool is_pmic_clkbuf;

static unsigned int afcdac_val = 0x2A80008; /* afc_default=4096 */
static bool is_clkbuf_afcdac_updated;

#ifdef ENABLE_PMIC_CLK_BUFFER
static unsigned int g_pmic_cw13_rg_val = PMIC_CW13_DEFAULT_VAL;
#endif /* ENABLE_PMIC_CLK_BUFFER */
/* FIXME: Before MP, using suggested driving current to test. */
#define TEST_SUGGEST_RF_DRIVING_CURR_BEFORE_MP
#ifdef ENABLE_PMIC_CLK_BUFFER
#define TEST_SUGGEST_PMIC_DRIVING_CURR_BEFORE_MP
#endif /* ENABLE_PMIC_CLK_BUFFER */

#if !defined(CONFIG_MTK_LEGACY)  /*Everest early porting: DCT tool generate .h file? */
static unsigned int CLK_BUF1_STATUS, CLK_BUF2_STATUS,
		    CLK_BUF3_STATUS, CLK_BUF4_STATUS;
#ifdef ENABLE_PMIC_CLK_BUFFER
static unsigned int CLK_BUF5_STATUS_PMIC = CLOCK_BUFFER_HW_CONTROL,
		    CLK_BUF6_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL,
		    CLK_BUF7_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL,
		    CLK_BUF8_STATUS_PMIC = CLOCK_BUFFER_HW_CONTROL;
#endif /* ENABLE_PMIC_CLK_BUFFER */

static unsigned int RF_CLK_BUF1_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA,
		    RF_CLK_BUF2_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA,
		    RF_CLK_BUF3_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA,
		    RF_CLK_BUF4_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA;
#ifdef ENABLE_PMIC_CLK_BUFFER
static unsigned int PMIC_CLK_BUF5_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA,
		    PMIC_CLK_BUF6_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA,
		    PMIC_CLK_BUF7_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA,
		    PMIC_CLK_BUF8_DRIVING_CURR = CLK_BUF_DRIVING_CURR_1_4MA;
#endif /* ENABLE_PMIC_CLK_BUFFER */

#else /* CONFIG_MTK_LEGACY */

#ifdef ENABLE_PMIC_CLK_BUFFER
static unsigned int CLK_BUF5_STATUS_PMIC, CLK_BUF6_STATUS_PMIC,
			CLK_BUF7_STATUS_PMIC, CLK_BUF8_STATUS_PMIC;
#endif /* ENABLE_PMIC_CLK_BUFFER */
#if !defined(RF_CLK_BUF1_DRIVING_CURR)
#define RF_CLK_BUF1_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#define RF_CLK_BUF2_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#define RF_CLK_BUF3_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#define RF_CLK_BUF4_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#endif
#ifdef ENABLE_PMIC_CLK_BUFFER
#if !defined(PMIC_CLK_BUF5_DRIVING_CURR)
#define PMIC_CLK_BUF5_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#define PMIC_CLK_BUF6_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#define PMIC_CLK_BUF7_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#define PMIC_CLK_BUF8_DRIVING_CURR	CLK_BUF_DRIVING_CURR_1_4MA
#endif
#endif /* ENABLE_PMIC_CLK_BUFFER */

#ifdef CLKBUF_COTSX_BRINGUP
#undef CLK_BUF1_STATUS
#undef CLK_BUF2_STATUS
#undef CLK_BUF3_STATUS
#undef CLK_BUF4_STATUS
#undef CLK_BUF5_STATUS_PMIC
#undef CLK_BUF6_STATUS_PMIC
#undef CLK_BUF7_STATUS_PMIC
#undef CLK_BUF8_STATUS_PMIC

#define CLK_BUF1_STATUS		CLOCK_BUFFER_DISABLE
#define CLK_BUF2_STATUS		CLOCK_BUFFER_DISABLE
#define CLK_BUF3_STATUS		CLOCK_BUFFER_DISABLE
#define CLK_BUF4_STATUS		CLOCK_BUFFER_DISABLE
#define CLK_BUF5_STATUS_PMIC	CLOCK_BUFFER_HW_CONTROL
#define CLK_BUF6_STATUS_PMIC	CLOCK_BUFFER_SW_CONTROL
#define CLK_BUF7_STATUS_PMIC	CLOCK_BUFFER_SW_CONTROL
#define CLK_BUF8_STATUS_PMIC	CLOCK_BUFFER_HW_CONTROL
#endif
#endif /* CONFIG_MTK_LEGACY */

#if 1
static CLK_BUF_SWCTRL_STATUS_T  clk_buf_swctrl[CLKBUF_NUM] = {
#ifdef CLKBUF_COTSX_BRINGUP
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_DISABLE
#else
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_DISABLE
#endif
};

static CLK_BUF_SWCTRL_STATUS_T  clk_buf_swctrl_modem_on[CLKBUF_NUM] = {
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_DISABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE
};
#else /* For Bring-up */
static CLK_BUF_SWCTRL_STATUS_T  clk_buf_swctrl[CLKBUF_NUM] = {
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE
};
#endif

static CLK_BUF_SWCTRL_STATUS_T  pmic_clk_buf_swctrl[CLKBUF_NUM] = {
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE,
	CLK_BUF_SW_ENABLE
};

static void spm_clk_buf_ctrl(CLK_BUF_SWCTRL_STATUS_T *status)
{
	u32 spm_val;
	int i;

	spm_val = spm_read(SPM_MDBSI_CON) & ~0x7;

	for (i = 1; i < CLKBUF_NUM; i++)
		spm_val |= status[i] << (i - 1);

	spm_write(SPM_MDBSI_CON, spm_val);

	udelay(2);
}

static void pmic_clk_buf_ctrl(CLK_BUF_SWCTRL_STATUS_T *status)
{
#ifdef ENABLE_PMIC_CLK_BUFFER
	u32 conn_conf = 0, nfc_conf = 0;

	pmic_config_interface(PMIC_CW00_ADDR, (status[PMIC_CLK_BUF_CONN] % 2),
			      PMIC_CW00_XO_EXTBUF2_EN_M_MASK,
			      PMIC_CW00_XO_EXTBUF2_EN_M_SHIFT);
	pmic_config_interface(PMIC_CW14_ADDR, (status[PMIC_CLK_BUF_NFC] % 2),
			      PMIC_CW14_XO_EXTBUF3_EN_M_MASK,
			      PMIC_CW14_XO_EXTBUF3_EN_M_SHIFT);

	pmic_read_interface(PMIC_CW00_ADDR, &conn_conf,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_CW14_ADDR, &nfc_conf,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	clk_buf_dbg("%s: CW00=0x%x, CW14=0x%x, clkbuf2=%u, clkbuf3=%u\n",
		     __func__, conn_conf, nfc_conf,
		     status[PMIC_CLK_BUF_CONN],
		     status[PMIC_CLK_BUF_NFC]);
#endif /* ENABLE_PMIC_CLK_BUFFER */
}

#ifdef ENABLE_PMIC_CLK_BUFFER
/*
 * Baseband Low Power Mode (BBLPM) for PMIC clkbuf
 * Condition: flightmode on + conn mtcmos off + NFC clkbuf off + enter deepidle
 * Caller: deep idle
 */
void clk_buf_control_bblpm(bool on)
{
	u32 cw00 = 0;

	if (!is_pmic_clkbuf ||
	    (pmic_clk_buf_swctrl[PMIC_CLK_BUF_NFC] == CLK_BUF_SW_ENABLE))
		return;

	pmic_config_interface_nolock(PMIC_CW00_ADDR, (on ? 1 : 0),
			      PMIC_CW00_XO_BB_LPM_EN_MASK,
			      PMIC_CW00_XO_BB_LPM_EN_SHIFT);

	pmic_read_interface_nolock(PMIC_CW00_ADDR, &cw00,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	clk_buf_dbg("%s(%u): CW00=0x%x\n", __func__, (on ? 1 : 0), cw00);
}
#endif /* ENABLE_PMIC_CLK_BUFFER */

#if 0 /* Fix me: not ready yet */
/* config rf_bpi gpio as IES=0 when init */
void clk_buf_config_rfbpi_gpio(void)
{
	int idx = 0;
	int rf_bpi_gpio[RF_BPI_GPIO_NUM] = {
		192, 193, 198, 199, 200, 201, 202, 203, 204, 205,
		206, 207, 208, 209, 210, 211, 212, 213, 214, 215};

	unsigned long mt_gpio_index = 0;

	clk_buf_warn("PIN: [MODE] [PULL_SEL] [DIN] [DOUT] [PULL EN] [DIR] [IES] [SMT]\n");
	for (idx = 0 ; idx < RF_BPI_GPIO_NUM; idx++) {
		mt_gpio_index = rf_bpi_gpio[idx] + MT_CPIO_INDEX_OFS;

		clk_buf_warn("gpio = %3d: %d %d %d %d %d %d %d %d\n",
				rf_bpi_gpio[idx],
				mt_get_gpio_mode(mt_gpio_index), mt_get_gpio_pull_select(mt_gpio_index),
				mt_get_gpio_in(mt_gpio_index), mt_get_gpio_out(mt_gpio_index),
				mt_get_gpio_pull_enable(mt_gpio_index), mt_get_gpio_dir(mt_gpio_index),
				mt_get_gpio_ies(mt_gpio_index), mt_get_gpio_smt(mt_gpio_index));


		mt_set_gpio_ies(mt_gpio_index, GPIO_IES_DISABLE);

		clk_buf_warn("gpio = %3d: %d %d %d %d %d %d %d %d\n",
				rf_bpi_gpio[idx],
				mt_get_gpio_mode(mt_gpio_index), mt_get_gpio_pull_select(mt_gpio_index),
				mt_get_gpio_in(mt_gpio_index), mt_get_gpio_out(mt_gpio_index),
				mt_get_gpio_pull_enable(mt_gpio_index), mt_get_gpio_dir(mt_gpio_index),
				mt_get_gpio_ies(mt_gpio_index), mt_get_gpio_smt(mt_gpio_index));

	}
}
#endif

#ifndef CONFIG_MT_ENG_BUILD
void clk_buf_config_mdjtag_gpio_pull_down(void)
{
	/* if GPIO_KPROW1 and GPIO_KPROW2 is set as md jtag function, set GPI pull down for power saving */
	if (mt_get_gpio_mode(GPIO_KPROW1) == GPIO_MODE_07 && mt_get_gpio_mode(GPIO_KPROW2) == GPIO_MODE_07) {
		mt_set_gpio_mode(GPIO_KPROW1, GPIO_MODE_GPIO);
		mt_set_gpio_pull_enable(GPIO_KPROW1, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_KPROW1, GPIO_PULL_DOWN);
		mt_set_gpio_ies(GPIO_KPROW1, GPIO_IES_DISABLE);

		mt_set_gpio_mode(GPIO_KPROW2, GPIO_MODE_GPIO);
		mt_set_gpio_pull_enable(GPIO_KPROW2, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_KPROW2, GPIO_PULL_DOWN);
		mt_set_gpio_ies(GPIO_KPROW2, GPIO_IES_DISABLE);
		clk_buf_warn("config modem jtag gpio as GPI pull down\n");
	}
}
#endif /* CONFIG_MT_ENG_BUILD */

static void clkbuf_delayed_worker(struct work_struct *work)
{
	bool srcclkena_o1 = false;

	srcclkena_o1 = (spm_read(PCM_REG13_DATA) & R13_MD1_VRF18_REQ)
			&& R13_MD1_VRF18_REQ;
	clk_buf_warn("%s: g_is_flightmode_on=%d, srcclkena_o1=%u, pcm_reg13=0x%x\n",
		     __func__, g_is_flightmode_on, srcclkena_o1,
		     spm_read(PCM_REG13_DATA));

	BUG_ON(clkbuf_debug && g_is_flightmode_on && srcclkena_o1);

	mutex_lock(&clk_buf_ctrl_lock);

	if (g_is_flightmode_on)
		spm_clk_buf_ctrl(clk_buf_swctrl);

	mutex_unlock(&clk_buf_ctrl_lock);
}

/* for spm driver use */
bool is_clk_buf_under_flightmode(void)
{
	return g_is_flightmode_on;
}

/* for ccci driver to notify this */
void clk_buf_set_by_flightmode(bool is_flightmode_on)
{
	bool srcclkena_o1 = false;

	srcclkena_o1 = (spm_read(PCM_REG13_DATA) & R13_MD1_VRF18_REQ)
			&& R13_MD1_VRF18_REQ;
	clk_buf_warn("%s: g/is_flightmode_on=%d->%d, srcclkena_o1=%u, pcm_reg13=0x%x\n",
		     __func__, g_is_flightmode_on, is_flightmode_on,
		     srcclkena_o1, spm_read(PCM_REG13_DATA));

	mutex_lock(&clk_buf_ctrl_lock);

	if (g_is_flightmode_on == is_flightmode_on) {
		mutex_unlock(&clk_buf_ctrl_lock);
		return;
	}

	g_is_flightmode_on = is_flightmode_on;

	if (!is_pmic_clkbuf) {
		if (g_is_flightmode_on) {
			schedule_delayed_work(&clkbuf_delayed_work,
					      msecs_to_jiffies(2000));
			/* spm_clk_buf_ctrl(clk_buf_swctrl); */
		} else {
			cancel_delayed_work(&clkbuf_delayed_work);
			spm_clk_buf_ctrl(clk_buf_swctrl_modem_on);
		}
	}

	mutex_unlock(&clk_buf_ctrl_lock);
}

bool clk_buf_ctrl(enum clk_buf_id id, bool onoff)
{
#ifdef ENABLE_PMIC_CLK_BUFFER
	if (is_pmic_clkbuf) {
		mutex_lock(&clk_buf_ctrl_lock);

		/* record the status of NFC from caller for checking BBLPM */
		if (id == CLK_BUF_NFC)
			pmic_clk_buf_swctrl[id] = onoff;

		mutex_unlock(&clk_buf_ctrl_lock);

		clk_buf_dbg("%s: id=%d, onoff=%d, is_flightmode_on=%d\n",
			    __func__, id, onoff, g_is_flightmode_on);

		return false;
	}
#endif /*ENABLE_PMIC_CLK_BUFFER*/

	if (id >= CLK_BUF_INVALID) /* TODO: need check DCT tool for CLK BUF SW control */
		return false;

	if ((id == CLK_BUF_BB_MD) && (CLK_BUF1_STATUS == CLOCK_BUFFER_HW_CONTROL))
		return false;

	if ((id == CLK_BUF_CONN) && (CLK_BUF2_STATUS == CLOCK_BUFFER_HW_CONTROL))
		return false;

	if ((id == CLK_BUF_NFC) && (CLK_BUF3_STATUS == CLOCK_BUFFER_HW_CONTROL))
		return false;

	if ((id == CLK_BUF_AUDIO) && (CLK_BUF4_STATUS == CLOCK_BUFFER_HW_CONTROL))
		return false;

#if 0
	/* for bring-up */
	clk_buf_warn("clk_buf_ctrl is disabled for bring-up\n");
	return false;
#else
	clk_buf_warn("%s: id=%d, onoff=%d, is_flightmode_on=%d\n", __func__,
		     id, onoff, g_is_flightmode_on);

	mutex_lock(&clk_buf_ctrl_lock);

	clk_buf_swctrl[id] = onoff;

	if (g_is_flightmode_on)
		spm_clk_buf_ctrl(clk_buf_swctrl);

	mutex_unlock(&clk_buf_ctrl_lock);

	return true;
#endif
}

void clk_buf_get_swctrl_status(CLK_BUF_SWCTRL_STATUS_T *status)
{
	int i;

	clk_buf_warn("%s: is_flightmode_on=%d, swctrl:clkbuf3=%d/%d, clkbuf4=%d/%d\n",
		     __func__, g_is_flightmode_on,
		     clk_buf_swctrl[2], clk_buf_swctrl_modem_on[2],
		     clk_buf_swctrl[3], clk_buf_swctrl_modem_on[3]);

	for (i = 0; i < CLKBUF_NUM; i++) {
		if (g_is_flightmode_on)
			status[i] = clk_buf_swctrl[i];
		else
			status[i] = clk_buf_swctrl_modem_on[i];
	}
}

/*
 * Let caller get driving current setting of RF clock buffer
 * Caller: ccci & ccci will send it to modem
 */
void clk_buf_get_rf_drv_curr(void *rf_drv_curr)
{
#ifdef TEST_SUGGEST_RF_DRIVING_CURR_BEFORE_MP
	RF_CLK_BUF1_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA,
	RF_CLK_BUF2_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA,
	RF_CLK_BUF3_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA,
	RF_CLK_BUF4_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA;
#endif

	((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[0] = RF_CLK_BUF1_DRIVING_CURR;
	((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[1] = RF_CLK_BUF2_DRIVING_CURR;
	((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[2] = RF_CLK_BUF3_DRIVING_CURR;
	((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[3] = RF_CLK_BUF4_DRIVING_CURR;

	clk_buf_warn("%s: rf_drv_curr_vals=%d %d %d %d\n", __func__,
		     ((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[0],
		     ((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[1],
		     ((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[2],
		     ((MTK_CLK_BUF_DRIVING_CURR *)rf_drv_curr)[3]);
}

/* Called by ccci driver to keep afcdac value sent from modem */
void clk_buf_save_afc_val(unsigned int afcdac)
{
	if (is_pmic_clkbuf)
		return;

	afcdac_val = afcdac;

	if (!is_clkbuf_afcdac_updated) {
		spm_write(SPM_BSI_EN_SR, afcdac_val);
		is_clkbuf_afcdac_updated = true;
	}

	clk_buf_warn("%s: afcdac=0x%x, SPM_BSI_EN_SR=0x%x\n", __func__,
		     afcdac_val, spm_read(SPM_BSI_EN_SR));
}

/* Called by suspend driver to write afcdac into SPM register */
void clk_buf_write_afcdac(void)
{
	if (is_pmic_clkbuf)
		return;

	spm_write(SPM_BSI_EN_SR, afcdac_val);
	clk_buf_dbg("%s: afcdac=0x%x, SPM_BSI_EN_SR=0x%x, afcdac_updated=%d\n",
		     __func__, afcdac_val, spm_read(SPM_BSI_EN_SR),
		     is_clkbuf_afcdac_updated);
}

static ssize_t clk_buf_ctrl_store(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	/* design for BSI wrapper command or by GPIO wavefrom */
	u32 clk_buf_en[CLKBUF_NUM], i;
	char cmd[32];

	if (sscanf(buf, "%31s %x %x %x %x", cmd, &clk_buf_en[0], &clk_buf_en[1],
		   &clk_buf_en[2], &clk_buf_en[3]) != 5)
		return -EPERM;

	if (!strcmp(cmd, "bsi")) {
		if (is_pmic_clkbuf)
			return -EINVAL;

		mutex_lock(&clk_buf_ctrl_lock);

		for (i = 0; i < CLKBUF_NUM; i++)
			clk_buf_swctrl[i] = clk_buf_en[i];

		if (g_is_flightmode_on)
			spm_clk_buf_ctrl(clk_buf_swctrl);

		mutex_unlock(&clk_buf_ctrl_lock);

		return count;
	}

	if (!strcmp(cmd, "pmic")) {
		if (!is_pmic_clkbuf)
			return -EINVAL;

		for (i = 0; i < CLKBUF_NUM; i++)
			pmic_clk_buf_swctrl[i] = clk_buf_en[i];

		pmic_clk_buf_ctrl(pmic_clk_buf_swctrl);

		return count;
	}

	return -EINVAL;
}

static ssize_t clk_buf_ctrl_show(struct kobject *kobj, struct kobj_attribute *attr,
				 char *buf)
{
	int len = 0;
	char *p = buf;
	bool srcclkena_o1 = false;

	p += sprintf(p, "********** RF clock buffer state (%s) flightmode(FM)=%d **********\n",
		     (is_pmic_clkbuf ? "off" : "on"), g_is_flightmode_on);
	p += sprintf(p, "CKBUF1_BB   SW(1)/HW(2) CTL: %d, Dis(0)/En(1) of FM=1:%d, FM=0:%d\n",
		     CLK_BUF1_STATUS, clk_buf_swctrl[0],
		     clk_buf_swctrl_modem_on[0]);
	p += sprintf(p, "CKBUF2_NONE SW(1)/HW(2) CTL: %d, Dis(0)/En(1) of FM=1:%d, FM=0:%d\n",
		     CLK_BUF2_STATUS, clk_buf_swctrl[1],
		     clk_buf_swctrl_modem_on[1]);
	p += sprintf(p, "CKBUF3_NFC  SW(1)/HW(2) CTL: %d, Dis(0)/En(1) of FM=1:%d, FM=0:%d\n",
		     CLK_BUF3_STATUS, clk_buf_swctrl[2],
		     clk_buf_swctrl_modem_on[2]);
	p += sprintf(p, "CKBUF4_AUD  SW(1)/HW(2) CTL: %d, Dis(0)/En(1) of FM=1:%d, FM=0:%d\n",
		     CLK_BUF4_STATUS, clk_buf_swctrl[3],
		     clk_buf_swctrl_modem_on[3]);
#ifdef ENABLE_PMIC_CLK_BUFFER
	p += sprintf(p, "********** PMIC clock buffer state (%s) **********\n",
		     (is_pmic_clkbuf ? "on" : "off"));
	p += sprintf(p, "CKBUF1_BB   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
		     CLK_BUF5_STATUS_PMIC, pmic_clk_buf_swctrl[0]);
	p += sprintf(p, "CKBUF2_CONN SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
		     CLK_BUF6_STATUS_PMIC, pmic_clk_buf_swctrl[1]);
	p += sprintf(p, "CKBUF3_NFC  SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
		     CLK_BUF7_STATUS_PMIC, pmic_clk_buf_swctrl[2]);
	p += sprintf(p, "CKBUF4_RF   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d\n",
		     CLK_BUF8_STATUS_PMIC, pmic_clk_buf_swctrl[3]);
	p += sprintf(p, "\n********** clock buffer command help **********\n");
	p += sprintf(p, "BSI  switch on/off: echo bsi en1 en2 en3 en4 > /sys/power/clk_buf/clk_buf_ctrl\n");
	p += sprintf(p, "PMIC switch on/off: echo pmic en1 en2 en3 en4 > /sys/power/clk_buf/clk_buf_ctrl\n");
#endif /* ENABLE_PMIC_CLK_BUFFER */

#ifdef ENABLE_PMIC_CLK_BUFFER
	if (is_pmic_clkbuf)
		p += sprintf(p, "g_pmic_cw13_rg_val=0x%x\n", g_pmic_cw13_rg_val);
	else
#endif /* ENABLE_PMIC_CLK_BUFFER */
		p += sprintf(p, "afcdac=0x%x, is_afcdac_updated=%d\n",
			     afcdac_val, is_clkbuf_afcdac_updated);

	p += sprintf(p, "rf_drv_curr_vals=%d %d %d %d\n",
		     RF_CLK_BUF1_DRIVING_CURR,
		     RF_CLK_BUF2_DRIVING_CURR,
		     RF_CLK_BUF3_DRIVING_CURR,
		     RF_CLK_BUF4_DRIVING_CURR);
#ifdef ENABLE_PMIC_CLK_BUFFER
	p += sprintf(p, "pmic_drv_curr_vals=%d %d %d %d\n",
		     PMIC_CLK_BUF5_DRIVING_CURR,
		     PMIC_CLK_BUF6_DRIVING_CURR,
		     PMIC_CLK_BUF7_DRIVING_CURR,
		     PMIC_CLK_BUF8_DRIVING_CURR);
#endif /* ENABLE_PMIC_CLK_BUFFER */
	srcclkena_o1 = (spm_read(PCM_REG13_DATA) & R13_MD1_VRF18_REQ)
			&& R13_MD1_VRF18_REQ;
	p += sprintf(p, "srcclkena_o1=%u, pcm_reg13=0x%x, MD1_PWR_CON=0x%x, C2K_PWR_CON=0x%x\n",
		     srcclkena_o1, spm_read(PCM_REG13_DATA),
		     spm_read(MD1_PWR_CON), spm_read(C2K_PWR_CON));

	len = p - buf;

	return len;
}

static ssize_t clk_buf_debug_store(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int debug = 0;

	if (!kstrtoint(buf, 10, &debug)) {
		if (debug == 0)
			clkbuf_debug = false;
		else if (debug == 1)
			clkbuf_debug = true;
		else
			clk_buf_warn("bad argument!! should be 0 or 1 [0: disable, 1: enable]\n");
	} else
		return -EPERM;

	return count;
}

static ssize_t clk_buf_debug_show(struct kobject *kobj, struct kobj_attribute *attr,
				 char *buf)
{
	int len = 0;
	char *p = buf;

	p += sprintf(p, "clkbuf_debug=%d\n", clkbuf_debug);

	len = p - buf;

	return len;
}

DEFINE_ATTR_RW(clk_buf_ctrl);
DEFINE_ATTR_RW(clk_buf_debug);

static struct attribute *clk_buf_attrs[] = {
	/* for clock buffer control */
	__ATTR_OF(clk_buf_ctrl),
	__ATTR_OF(clk_buf_debug),

	/* must */
	NULL,
};

static struct attribute_group spm_attr_group = {
	.name	= "clk_buf",
	.attrs	= clk_buf_attrs,
};

bool is_clk_buf_from_pmic(void)
{
#ifdef ENABLE_PMIC_CLK_BUFFER
	unsigned int reg = 0;
	bool ret = false;

	if (is_clkbuf_initiated)
		return is_pmic_clkbuf;

	/* switch to debug mode */
	pmic_config_interface_nolock(PMIC_CW15_ADDR, 0x1,
			      PMIC_CW15_DCXO_STATIC_AUXOUT_EN_MASK,
			      PMIC_CW15_DCXO_STATIC_AUXOUT_EN_SHIFT);
	pmic_config_interface_nolock(PMIC_CW15_ADDR, 0x3,
			      PMIC_CW15_DCXO_STATIC_AUXOUT_SEL_MASK,
			      PMIC_CW15_DCXO_STATIC_AUXOUT_SEL_SHIFT);
	/* bit 6, 7, 8, 9 => 32K Less Mode, Buffer Mode, RTC Mode, Off Mode */
	pmic_read_interface_nolock(PMIC_CW00_ADDR, &reg,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	/* switch back from debug mode */
	pmic_config_interface_nolock(PMIC_CW15_ADDR, 0x0,
			      PMIC_CW15_DCXO_STATIC_AUXOUT_EN_MASK,
			      PMIC_CW15_DCXO_STATIC_AUXOUT_EN_SHIFT);
	if ((reg & 0x200) == 0x200) {
		clk_buf_warn_limit("clkbuf is from RF, CW00=0x%x\n", reg);
		ret = false;
	} else {
		clk_buf_warn_limit("clkbuf is from PMIC, CW00=0x%x\n", reg);
		ret = true;
	}

	return ret;
#else
	return false;
#endif /* ENABLE_PMIC_CLK_BUFFER */
};

#ifdef ENABLE_PMIC_CLK_BUFFER
static void gen_pmic_cw13_rg_val(void)
{
#ifdef TEST_SUGGEST_PMIC_DRIVING_CURR_BEFORE_MP
	PMIC_CLK_BUF5_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA,
	PMIC_CLK_BUF6_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA,
	PMIC_CLK_BUF7_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA,
	PMIC_CLK_BUF8_DRIVING_CURR = CLK_BUF_DRIVING_CURR_0_9MA;
#endif
	g_pmic_cw13_rg_val = PMIC_CW13_XO_EXTBUF_HD_VAL |
			     (PMIC_CLK_BUF5_DRIVING_CURR << 2) |
			     (PMIC_CLK_BUF6_DRIVING_CURR << 6) |
			     (PMIC_CLK_BUF7_DRIVING_CURR << 10) |
			     (PMIC_CLK_BUF8_DRIVING_CURR << 14);

	clk_buf_warn("%s: g_pmic_cw13_rg_val=0x%x, pmic_drv_curr_vals=%d %d %d %d\n",
		     __func__, g_pmic_cw13_rg_val,
		     PMIC_CLK_BUF5_DRIVING_CURR,
		     PMIC_CLK_BUF6_DRIVING_CURR,
		     PMIC_CLK_BUF7_DRIVING_CURR,
		     PMIC_CLK_BUF8_DRIVING_CURR);
}
#endif /* ENABLE_PMIC_CLK_BUFFER */

static void clk_buf_pmic_wrap_init(void)
{
#ifdef ENABLE_PMIC_CLK_BUFFER
	u32 conn_conf = 0, nfc_conf = 0, pmic_cw13 = 0;

	gen_pmic_cw13_rg_val();
#if 0
	/* Switch clkbuf2,3 to S/W mode control */
	pmic_config_interface(PMIC_CW00_ADDR, 0,
			      PMIC_CW00_XO_EXTBUF2_MODE_MASK,
			      PMIC_CW00_XO_EXTBUF2_MODE_SHIFT); /* XO_WCN */
	pmic_config_interface(PMIC_CW00_ADDR, 0,
			      PMIC_CW00_XO_EXTBUF3_MODE_MASK,
			      PMIC_CW00_XO_EXTBUF3_MODE_SHIFT); /* XO_NFC */
#else
	/* Setup initial PMIC clock buffer setting */
	pmic_read_interface(PMIC_CW00_ADDR, &conn_conf,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_CW14_ADDR, &nfc_conf,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_CW13_ADDR, &pmic_cw13,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	clk_buf_warn("%s PMIC_CW00_ADDR=0x%x, PMIC_CW14_ADDR=0x%x, PMIC_CW13_ADDR=0x%x\n",
		     __func__, conn_conf, nfc_conf, pmic_cw13);
	pmic_config_interface(PMIC_CW00_ADDR, PMIC_CW00_INIT_VAL,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_config_interface(PMIC_CW13_ADDR, g_pmic_cw13_rg_val,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
#endif

	/* Check if the setting is ok */
	pmic_read_interface(PMIC_CW00_ADDR, &conn_conf,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_CW14_ADDR, &nfc_conf,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	pmic_read_interface(PMIC_CW13_ADDR, &pmic_cw13,
			    PMIC_REG_MASK, PMIC_REG_SHIFT);
	clk_buf_warn("%s PMIC_CW00_ADDR=0x%x, PMIC_CW14_ADDR=0x%x, PMIC_CW13_ADDR=0x%x\n",
		     __func__, conn_conf, nfc_conf, pmic_cw13);

	clkbuf_writel(DCXO_CONN_ADR0, PMIC_CW00_ADDR);
	clkbuf_writel(DCXO_CONN_WDATA0, conn_conf & 0xFFDF);	/* bit5 = 0 */
	clkbuf_writel(DCXO_CONN_ADR1, PMIC_CW00_ADDR);
	clkbuf_writel(DCXO_CONN_WDATA1, conn_conf | 0x0020);	/* bit5 = 1 */
	clkbuf_writel(DCXO_NFC_ADR0, PMIC_CW14_ADDR);
	clkbuf_writel(DCXO_NFC_WDATA0, nfc_conf & 0xF7FF);	/* bit11 = 0 */
	clkbuf_writel(DCXO_NFC_ADR1, PMIC_CW14_ADDR);
	clkbuf_writel(DCXO_NFC_WDATA1, nfc_conf | 0x0800);	/* bit11 = 1 */

	clkbuf_writel(DCXO_ENABLE, DCXO_CONN_ENABLE | DCXO_NFC_ENABLE);
#endif /* ENABLE_PMIC_CLK_BUFFER */
}

static int clk_buf_fs_init(void)
{
	int r = 0;

#if defined(CONFIG_PM)
	/* create /sys/power/clk_buf/xxx */
	r = sysfs_create_group(power_kobj, &spm_attr_group);
	if (r)
		clk_buf_err("FAILED TO CREATE /sys/power/clk_buf (%d)\n", r);
#endif

	return r;
}

static void clk_buf_clear_rf_setting(void)
{
#ifdef ENABLE_PMIC_CLK_BUFFER
	memset(clk_buf_swctrl, 0, sizeof(clk_buf_swctrl));
	memset(clk_buf_swctrl_modem_on, 0, sizeof(clk_buf_swctrl_modem_on));

#if !defined(CONFIG_MTK_LEGACY)
	CLK_BUF1_STATUS = CLOCK_BUFFER_DISABLE;
	CLK_BUF2_STATUS = CLOCK_BUFFER_DISABLE;
	CLK_BUF3_STATUS = CLOCK_BUFFER_DISABLE;
	CLK_BUF4_STATUS = CLOCK_BUFFER_DISABLE;
	CLK_BUF5_STATUS_PMIC = CLOCK_BUFFER_HW_CONTROL;
	CLK_BUF6_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL;
	CLK_BUF7_STATUS_PMIC = CLOCK_BUFFER_SW_CONTROL;
	CLK_BUF8_STATUS_PMIC = CLOCK_BUFFER_HW_CONTROL;

	clk_buf_warn("%s: RF_CLK_BUF?_STATUS=%d %d %d %d\n", __func__,
		     CLK_BUF1_STATUS, CLK_BUF2_STATUS,
		     CLK_BUF3_STATUS, CLK_BUF4_STATUS);
	clk_buf_warn("%s: PMIC_CLK_BUF?_STATUS=%d %d %d %d\n", __func__,
		     CLK_BUF5_STATUS_PMIC, CLK_BUF6_STATUS_PMIC,
		     CLK_BUF7_STATUS_PMIC, CLK_BUF8_STATUS_PMIC);
#endif
#endif /* ENABLE_PMIC_CLK_BUFFER */
}

bool clk_buf_init(void)
{
	struct device_node *node;
#if !defined(CONFIG_MTK_LEGACY)
	u32 vals[CLKBUF_NUM] = {0, 0, 0, 0};
	int ret = -1;

#if 1 /* for kernel 3.18 */
	node = of_find_compatible_node(NULL, NULL, "mediatek,rf_clock_buffer");
	if (node) {
		of_property_read_u32_array(node, "mediatek,clkbuf-config",
					   vals, CLKBUF_NUM);
		CLK_BUF1_STATUS = vals[0];
		CLK_BUF2_STATUS = vals[1];
		CLK_BUF3_STATUS = vals[2];
		CLK_BUF4_STATUS = vals[3];
		ret = of_property_read_u32_array(node, "mediatek,clkbuf-driving-current",
						 vals, CLKBUF_NUM);
		if (!ret) {
			RF_CLK_BUF1_DRIVING_CURR = vals[0];
			RF_CLK_BUF2_DRIVING_CURR = vals[1];
			RF_CLK_BUF3_DRIVING_CURR = vals[2];
			RF_CLK_BUF4_DRIVING_CURR = vals[3];
			clk_buf_warn("%s: device tree rf_drv_curr_vals=%d %d %d %d\n", __func__,
				     RF_CLK_BUF1_DRIVING_CURR,
				     RF_CLK_BUF2_DRIVING_CURR,
				     RF_CLK_BUF3_DRIVING_CURR,
				     RF_CLK_BUF4_DRIVING_CURR);
		}
	} else {
		clk_buf_err("%s can't find compatible node for rf_clock_buffer\n", __func__);
		BUG();
	}
#ifdef ENABLE_PMIC_CLK_BUFFER
	node = of_find_compatible_node(NULL, NULL, "mediatek,pmic_clock_buffer");
	if (node) {
		ret = of_property_read_u32_array(node, "mediatek,clkbuf-config",
						 vals, CLKBUF_NUM);
		if (!ret) {
			CLK_BUF5_STATUS_PMIC = vals[0];
			CLK_BUF6_STATUS_PMIC = vals[1];
			CLK_BUF7_STATUS_PMIC = vals[2];
			CLK_BUF8_STATUS_PMIC = vals[3];
		}
		ret = of_property_read_u32_array(node, "mediatek,clkbuf-driving-current",
						 vals, CLKBUF_NUM);
		if (!ret) {
			PMIC_CLK_BUF5_DRIVING_CURR = vals[0];
			PMIC_CLK_BUF6_DRIVING_CURR = vals[1];
			PMIC_CLK_BUF7_DRIVING_CURR = vals[2];
			PMIC_CLK_BUF8_DRIVING_CURR = vals[3];
		}
	} else {
		clk_buf_err("%s can't find compatible node for pmic_clock_buffer\n", __func__);
		BUG();
	}
#endif /* ENABLE_PMIC_CLK_BUFFER */
#else /* for kernel 3.10 */
	node = of_find_compatible_node(NULL, NULL, "mediatek, rf_clock_buffer");
	if (node) {
		of_property_read_u32(node, "buffer1", (u32 *)&CLK_BUF1_STATUS);
		of_property_read_u32(node, "buffer2", (u32 *)&CLK_BUF2_STATUS);
		of_property_read_u32(node, "buffer3", (u32 *)&CLK_BUF3_STATUS);
		of_property_read_u32(node, "buffer4", (u32 *)&CLK_BUF4_STATUS);
		ret = of_property_read_u32_array(node, "mediatek,clkbuf-driving-current",
						 vals, CLKBUF_NUM);
		if (!ret) {
			RF_CLK_BUF1_DRIVING_CURR = vals[0];
			RF_CLK_BUF2_DRIVING_CURR = vals[1];
			RF_CLK_BUF3_DRIVING_CURR = vals[2];
			RF_CLK_BUF4_DRIVING_CURR = vals[3];
		}
	} else {
		clk_buf_err("%s can't find compatible node for rf_clock_buffer\n", __func__);
		BUG_ON(1);
	}

#ifdef ENABLE_PMIC_CLK_BUFFER
	node = of_find_compatible_node(NULL, NULL, "mediatek, pmic_clock_buffer");
	if (node) {
		of_property_read_u32(node, "buffer1", (u32 *)&CLK_BUF5_STATUS_PMIC);
		of_property_read_u32(node, "buffer2", (u32 *)&CLK_BUF6_STATUS_PMIC);
		of_property_read_u32(node, "buffer3", (u32 *)&CLK_BUF7_STATUS_PMIC);
		of_property_read_u32(node, "buffer4", (u32 *)&CLK_BUF8_STATUS_PMIC);
		ret = of_property_read_u32_array(node, "mediatek,clkbuf-driving-current", vals, CLKBUF_NUM);
		if (!ret) {
			PMIC_CLK_BUF5_DRIVING_CURR = vals[0];
			PMIC_CLK_BUF6_DRIVING_CURR = vals[1];
			PMIC_CLK_BUF7_DRIVING_CURR = vals[2];
			PMIC_CLK_BUF8_DRIVING_CURR = vals[3];
		}
	} else {
		clk_buf_err("%s can't find compatible node for pmic_clock_buffer\n", __func__);
		BUG_ON(1);
	}
#endif /* ENABLE_PMIC_CLK_BUFFER */

#endif /* for kernel 3.18 */
#endif /* CONFIG_MTK_LEGACY */
	if (is_clkbuf_initiated)
		return false;

	if (clk_buf_fs_init())
		return false;

	/* Co-TSX @PMIC */
	if (is_clk_buf_from_pmic()) {
		is_pmic_clkbuf = true;

		node = of_find_compatible_node(NULL, NULL, "mediatek,pwrap");

		if (node)
			pwrap_base = of_iomap(node, 0);
		else {
			clk_buf_err("%s can't find compatible node for pwrap\n",
			       __func__);
			BUG();
		}

		clk_buf_pmic_wrap_init();
		clk_buf_clear_rf_setting();
	} else { /* VCTCXO @RF */
		spm_write(SPM_BSI_EN_SR, afcdac_val);
		clk_buf_warn("%s: afcdac=0x%x, SPM_BSI_EN_SR=0x%x\n", __func__,
			     afcdac_val, spm_read(SPM_BSI_EN_SR));
	}

	/* Config RF_bpi related GPIO as IES=0*/
	/*clk_buf_config_rfbpi_gpio(); */

#ifndef CONFIG_MT_ENG_BUILD
	clk_buf_config_mdjtag_gpio_pull_down();
#endif

	INIT_DELAYED_WORK(&clkbuf_delayed_work, clkbuf_delayed_worker);

	is_clkbuf_initiated = true;

	return true;
}

