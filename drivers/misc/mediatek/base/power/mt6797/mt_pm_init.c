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

#include <linux/pm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/types.h>
/* #include <linux/xlog.h> */

#include <asm/io.h>
#include <asm/uaccess.h>

/* #include "mach/irqs.h" */
#include <mt-plat/sync_write.h>
/* #include "mach/mt_reg_base.h" */
/* #include "mach/mt_typedefs.h" */
#include "mt_spm.h"
#include "mt_sleep.h"
#include "mt_dcm.h"
#include "mt_clkmgr.h"
#include "mt_cpufreq.h"
#include "mt_gpufreq.h"
/* #include "mach/mt_sleep.h" */
/* #include "mach/mt_dcm.h" */
#include <mach/mt_clkmgr.h>
/* #include "mach/mt_cpufreq.h" */
/* #include "mach/mt_gpufreq.h" */
#include "mt_cpuidle.h"
#include "mt_clkbuf_ctl.h"
/* #include "mach/mt_chip.h" */
#include <mach/mt_freqhopping.h>
#include "mt-plat/mtk_rtc.h"
#include "mt_freqhopping_drv.h"


#ifdef CONFIG_ARM64
#define IOMEM(a)	((void __force __iomem *)((a)))
#endif

#define pminit_write(addr, val)         mt_reg_sync_writel((val), ((void *)(addr)))
#define pminit_read(addr)               __raw_readl(IOMEM(addr))
#ifndef DRV_WriteReg32
#define DRV_WriteReg32(addr, val)   \
	mt_reg_sync_writel(val, addr)
#endif

#define TOPCK_LDVT
#ifdef TOPCK_LDVT
/***************************
*For TOPCKGen Meter LDVT Test
****************************/
unsigned int ckgen_meter(int ID)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk26cali_0 = pminit_read(CLK26CALI_0);

	clk_dbg_cfg = pminit_read(CLK_DBG_CFG);

	/*sel ckgen_cksw[0] and enable freq meter sel ckgen[13:8], 01:hd_faxi_ck*/
	pminit_write(CLK_DBG_CFG, (ID << 8) | 0x01);

	clk_misc_cfg_0 = pminit_read(CLK_MISC_CFG_0);
	pminit_write(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));	/*select divider?dvt set zero*/

	clk26cali_1 = pminit_read(CLK26CALI_1);
	/*pminit_write(CLK26CALI_1, 0x00ff0000); */

	/*temp = pminit_read(CLK26CALI_0);*/
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (pminit_read(CLK26CALI_0) & 0x10) {
		udelay(10);
		i++;
		if (i > 10000)
			break;
	}

	temp = pminit_read(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024;	/* Khz*/

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	/*pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);*/
	/*pminit_write(CLK26CALI_0, clk26cali_0);*/
	/*pminit_write(CLK26CALI_1, clk26cali_1);*/

	pminit_write(CLK26CALI_0, 0x1010);
	udelay(50);
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x0000);
	if (i > 10)
		return 0;
	else
		return output;

}

unsigned int abist_meter(int ID)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;
	unsigned int temp2 = 0, temp1 = 0;

	temp1 = mt6797_0x1001AXXX_reg_read(ARMPLLDIV_ARM_K1);
	temp2 = mt6797_0x1001AXXX_reg_read(ARMPLLDIV_MON_EN);
	mt6797_0x1001AXXX_reg_write(ARMPLLDIV_ARM_K1, 0);
	mt6797_0x1001AXXX_reg_write(ARMPLLDIV_MON_EN, 0xFFFFFFFF);

	clk26cali_0 = pminit_read(CLK26CALI_0);

	clk_dbg_cfg = pminit_read(CLK_DBG_CFG);

	/*sel abist_cksw and enable freq meter sel abist*/
	pminit_write(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFC0FFFC) | (ID << 16));

	clk_misc_cfg_0 = pminit_read(CLK_MISC_CFG_0);
	pminit_write(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF)|0x01000000);	/* select divider, WAIT CONFIRM*/

	clk26cali_1 = pminit_read(CLK26CALI_1);
	/*pminit_write(CLK26CALI_1, 0x00ff0000); // cycle count default 1024,[25:16]=3FF*/

	/*temp = pminit_read(CLK26CALI_0);*/
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (pminit_read(CLK26CALI_0) & 0x10) {
		udelay(10);
		i++;
		if (i > 10000)
			break;
	}


	temp = pminit_read(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024 * 2;	/* Khz*/

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	/*pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);*/
	/*pminit_write(CLK26CALI_0, clk26cali_0);*/
	/*pminit_write(CLK26CALI_1, clk26cali_1);*/


	pminit_write(CLK26CALI_0, 0x1010);
	udelay(50);
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x0000);

	mt6797_0x1001AXXX_reg_write(ARMPLLDIV_MON_EN, temp2);
	mt6797_0x1001AXXX_reg_write(ARMPLLDIV_ARM_K1, temp1);

	if (i > 10)
		return 0;
	else
		return output;
}

const char *ckgen_array[] = {
	"hd_faxi_ck", "hf_fddrphycfg_ck", "f_fpwm_ck", "hf_fvdec_ck", "hf_fmm_ck",
	"hf_fcamtg_ck", "f_fuart_ck", "hf_fspi_ck", "hf_fmsdc50_0_h_ck", "hf_fmsdc50_0_ck",
	"hf_fmsdc30_1_ck", "hf_fmsdc30_2_ck", "f52m_mfg_ck", "hf_faudio_ck", "hf_faud_intbus_ck",
	"hf_fpmicspi_ck", "hf_fscp_ck", "hf_fatb_ck", "hf_fmjc_ck", "hf_fdpi0_ck",
	"hf_faud_1_ck", "hf_faud_2_ck", "hf_fanc_md32_ck", "hf_fmfg_ck", "f_fusb20_ck",
	"hf_fenc_ck", "f_fssusb_top_sys_ck", "hg_fspm_ck", "fmem_ck", "hf_fbsi_spi_ck",
	"hf_faudio_h_ck"
};

const char *ckgen_abist_array[] = {
	"b0", "AD_OSC_CK_abist_mon ", "AD_OSC_CK_D3 ", "AD_MAIN_H546M_CK ", "AD_MAIN_H364M_CK ",
	"AD_MAIN_H218P4M_CK ", "AD_MAIN_H156M_CK ", "AD_UNIV_178P3M_CK ", "AD_UNIV_48M_CK ",
	    "AD_UNIV_624M_CK ",
	"AD_UNIV_416M_CK ", "AD_UNIV_249P6M_CK ", "AD_APLL1_180P6336M_CK ", "AD_APLL2_196P608M_CK ",
	    "AD_MDPLL1_26M_CK ",
	"rtc32k_ck_i ", "AD_MFGPLL_500M_CK ", "AD_IMGPLL_450M_CK ", "AD_TVDPLL_594M_CK ",
	    "AD_CODECPLL_494M_CK ",
	"AD_MSDCPLL_400M_CK ", "AD_USB20_48M_CK ", "AD_MEMPLL_MONCLK ", "MIPI0_DSI_TST_CK ",
	    "AD_PLLGP_TST_CK ",
	"AD_MCUPLLGP_TSTDIV2_CK ", "fmem_ck ", "clkm_ck_out_0 ", "clkm_ck_out_1 ", "clkm_ck_out_2 ",
	"clkm_ck_out_3 ", "clkm_ck_out_4 ", "clkm_ck_out_5 ", "armpll_arm_clk_out1 ",
	    "armpll_arm_clk_out2 ",
	"armpll_arm_clk_out3 ", "armpll_arm_clk_out0 ", "1b0", "1b0", "AD_VDECPLL_500M_CK ",
	"AD_MPLL_104M_CK ", "AD_ARMCAXPLL1_CK_MON ", "AD_ARMCAXPLL2_CK_MON ",
	    "AD_ARMCAXPLL3_CK_MON ", "AD_ARMCAXPLL4_CK_MON ",
	"AD_ARMCAXPLL0_CK_MON ", "AD_SSUSB_48M_CK ", "AD_CA_RPHYPLL_DIV4_CK ",
	    "AD_CA_RCLKRPLL_DIV4_CK ", "AD_CB_RPHYPLL_DIV4_CK ",
	"AD_CB_RCLKRPLL_DIV4_CK ", "AD_AB_RPHYPLL_DIV4_CK ", "AD_AB_RCLKRPLL_DIV4_CK ",
	    "AD_CSI0A_DELAYCAL_CK_mux ", "AD_CSI0B_DELAYCAL_CK_mux ",
	"AD_CSI1A_DELAYCAL_CK_mux ", "AD_CSI1B_DELAYCAL_CK_mux ", "AD_CSI2_DELAYCAL_CK_mux ",
	    "MIPI1_DSI_TST_CK ", "AD_MDPLLGP_TSTDIV2_CK ",
	"AD_CA_RPHYPLL_TSTDIV2_CK ", "AD_CB_RPHYPLL_TSTDIV2_CK ", "AD_AB_RPHYPLL_TSTDIV2_CK ",
};


static int ckgen_meter_read(struct seq_file *m, void *v)
{
	int i;

	for (i = 1; i < 32; i++)
		seq_printf(m, "[%d] %s: %d\n", i, ckgen_array[i - 1], ckgen_meter(i));

	return 0;
}

static ssize_t ckgen_meter_write(struct file *file, const char __user *buffer,
				 size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;
	int val;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (kstrtoint(desc, 10, &val) >= 0)
		pr_debug("ckgen_meter %d is %d\n", val, ckgen_meter(val));

	return count;
}


static int abist_meter_read(struct seq_file *m, void *v)
{
	int i;

	for (i = 1; i < 64; i++) {
		/*skip unknown case */
		if (i == 1 || i == 38 || i == 39)
			continue;

		seq_printf(m, "[%d] %s: %d\n", i, ckgen_abist_array[i - 1], abist_meter(i));
	}
	return 0;
}

static ssize_t abist_meter_write(struct file *file, const char __user *buffer,
				 size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;
	int val;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (kstrtoint(desc, 10, &val) >= 0)
		pr_debug("abist_meter %d is %d\n", val, abist_meter(val));

	return count;
}

static int proc_abist_meter_open(struct inode *inode, struct file *file)
{
	return single_open(file, abist_meter_read, NULL);
}

static const struct file_operations abist_meter_fops = {
	.owner = THIS_MODULE,
	.open = proc_abist_meter_open,
	.read = seq_read,
	.write = abist_meter_write,
};

static int proc_ckgen_meter_open(struct inode *inode, struct file *file)
{
	return single_open(file, ckgen_meter_read, NULL);
}

static const struct file_operations ckgen_meter_fops = {
	.owner = THIS_MODULE,
	.open = proc_ckgen_meter_open,
	.read = seq_read,
	.write = ckgen_meter_write,
};

#endif

/*********************************************************************
 * FUNCTION DEFINATIONS
 ********************************************************************/

static unsigned int mt_get_emi_freq(void)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk26cali_0 = pminit_read(CLK26CALI_0);
	/*sel abist_cksw and enable abist meter sel abist*/
	clk_dbg_cfg = pminit_read(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFC) | (23 << 16));

	clk_misc_cfg_0 = pminit_read(CLK_MISC_CFG_0);
	pminit_write(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));	/* select divider*/

	clk26cali_1 = pminit_read(CLK26CALI_1);
	/*pminit_write(CLK26CALI_1, 0x00ff0000); */

	/*temp = pminit_read(CLK26CALI_0);*/
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (pminit_read(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = pminit_read(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024;	/* Khz*/

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	if (i > 10)
		return 0;
	else
		return output;

}
EXPORT_SYMBOL(mt_get_emi_freq);

unsigned int mt_get_bus_freq(void)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk26cali_0 = pminit_read(CLK26CALI_0);

	clk_dbg_cfg = pminit_read(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, (1 << 8) | 0x01);	/*sel abist_cksw and enable freq meter sel abist*/

	clk_misc_cfg_0 = pminit_read(CLK_MISC_CFG_0);
	pminit_write(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));	/* select divider*/

	clk26cali_1 = pminit_read(CLK26CALI_1);
	/*pminit_write(CLK26CALI_1, 0x00ff0000); */

	/*temp = pminit_read(CLK26CALI_0);*/
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (pminit_read(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = pminit_read(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024;	/* Khz*/

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	if (i > 10)
		return 0;
	else
		return output;


}
EXPORT_SYMBOL(mt_get_bus_freq);

#if 0
static unsigned int mt_get_cpu_freq(void)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = pminit_read(CLK_DBG_CFG);

	/*sel abist_cksw and enable freq meter sel abist*/
	pminit_write(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFC) | (42 << 16));
	clk_misc_cfg_0 = pminit_read(CLK_MISC_CFG_0);
	pminit_write(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));	/* select divider, WAIT CONFIRM*/

	clk26cali_1 = pminit_read(CLK26CALI_1);
	/*pminit_write(CLK26CALI_1, 0x00ff0000); // cycle count default 1024,[25:16]=3FF*/

	/*temp = pminit_read(CLK26CALI_0);*/
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (pminit_read(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = pminit_read(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024;	/* Khz*/

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	if (i > 10)
		return 0;
	else
		return output;

}
EXPORT_SYMBOL(mt_get_cpu_freq);
#endif

#if 0
enum {
	ARMPLL_BIG = 0,
	ARMPLL_LL,
	ARMPLL_L,
	ARMPLL_CCI,
};
static int cpu_div_info(int cpu)
{
	unsigned int temp = 0, div = 0, i = 0;
	int ret = 0;

	temp = pminit_read(ARMPLLDIV_CKDIV);
	switch (cpu) {
	case ARMPLL_BIG:
		div = (temp & _ARMPLL_B_DIV_MASK_) >> _ARMPLL_B_DIV_BIT_;
		break;
	case ARMPLL_LL:
		div = (temp & _ARMPLL_LL_DIV_MASK_) >> _ARMPLL_LL_DIV_BIT_;
		break;
	case ARMPLL_L:
		div = (temp & _ARMPLL_L_DIV_MASK_) >> _ARMPLL_L_DIV_BIT_;
		break;
	case ARMPLL_CCI:
		div = (temp & _ARMPLL_CCI_DIV_MASK_) >> _ARMPLL_CCI_DIV_BIT_;
		break;
	}

	switch (div) {
	case _ARMPLL_DIV_4_:
		ret = 4;
		break;
	case _ARMPLL_DIV_2_:
		ret = 2;
		break;
	case _ARMPLL_DIV_1_:
		ret = 1;
		break;
	case 0:
		ret = 1;
		break;
	}
	return ret;
}
#endif

static int cpu_speed_dump_read(struct seq_file *m, void *v)
{

#if 0
	seq_printf(m, "%s(LL):  %d Khz, CKDIV: %d\n", ckgen_abist_array[41], abist_meter(42),
		   cpu_div_info(ARMPLL_LL));
	seq_printf(m, "%s(L):  %d Khz, CKDIV: %d\n", ckgen_abist_array[42], abist_meter(43),
		   cpu_div_info(ARMPLL_L));
	seq_printf(m, "%s{CCI}:  %d Khz, CKDIV: %d\n", ckgen_abist_array[43], abist_meter(44),
		   cpu_div_info(ARMPLL_CCI));
	seq_printf(m, "%s(BiG) :  %d Khz, CKDIV: %d\n", ckgen_abist_array[45], abist_meter(46),
		   cpu_div_info(ARMPLL_BIG));
#else
	seq_printf(m, "%s(LL): %d Khz\n", ckgen_abist_array[41], abist_meter(34));
	seq_printf(m, "%s(L): %d Khz\n", ckgen_abist_array[42], abist_meter(35));
	seq_printf(m, "%s(CCI): %d Khz\n", ckgen_abist_array[43], abist_meter(36));
	seq_printf(m, "%s(BIG): %d Khz\n", ckgen_abist_array[45], abist_meter(37));
#endif
	return 0;
}

static int mfg_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s :  %d Khz\n", ckgen_abist_array[16], abist_meter(17));
	return 0;
}

static int emi_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_emi_freq());
	return 0;
}

static int bus_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_bus_freq());
	return 0;
}

#if 0
static int mmclk_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_mmclk_freq());
	return 0;
}

static int mfgclk_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_mfgclk_freq());
	return 0;
}
#endif

static int proc_cpu_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_speed_dump_read, NULL);
}

static const struct file_operations cpu_fops = {
	.owner = THIS_MODULE,
	.open = proc_cpu_open,
	.read = seq_read,
};

static int proc_mfg_open(struct inode *inode, struct file *file)
{
	return single_open(file, mfg_speed_dump_read, NULL);
}

static const struct file_operations mfg_fops = {
	.owner = THIS_MODULE,
	.open = proc_mfg_open,
	.read = seq_read,
};

static int proc_emi_open(struct inode *inode, struct file *file)
{
	return single_open(file, emi_speed_dump_read, NULL);
}

static const struct file_operations emi_fops = {
	.owner = THIS_MODULE,
	.open = proc_emi_open,
	.read = seq_read,
};

static int proc_bus_open(struct inode *inode, struct file *file)
{
	return single_open(file, bus_speed_dump_read, NULL);
}

static const struct file_operations bus_fops = {
	.owner = THIS_MODULE,
	.open = proc_bus_open,
	.read = seq_read,
};

#if 0
static int proc_mmclk_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmclk_speed_dump_read, NULL);
}

static const struct file_operations mmclk_fops = {
	.owner = THIS_MODULE,
	.open = proc_mmclk_open,
	.read = seq_read,
};

static int proc_mfgclk_open(struct inode *inode, struct file *file)
{
	return single_open(file, mfgclk_speed_dump_read, NULL);
}

static const struct file_operations mfgclk_fops = {
	.owner = THIS_MODULE,
	.open = proc_mfgclk_open,
	.read = seq_read,
};
#endif


static int __init mt_power_management_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *pm_init_dir = NULL;
	/* unsigned int code = mt_get_chip_hw_code(); */

	pm_power_off = mt_power_off;

#if !defined(CONFIG_MTK_FPGA)
	/* cpu dormant driver init */
	mt_cpu_dormant_init();

#if 0
	/* SPM driver init*/
	spm_module_init();

	/* Sleep driver init (for suspend)*/
	if (0x321 == code)
		slp_module_init();
	else if (0x335 == code)
		slp_module_init();
	else if (0x337 == code)
		slp_module_init();
	/* other unknown chip ID, error !!*/
#endif

	spm_module_init();
	slp_module_init();
	mt_clkmgr_init();
	mt_freqhopping_init();

	/* mt_pm_log_init(); // power management log init */

	/* mt_dcm_init(); // dynamic clock management init */


	pm_init_dir = proc_mkdir("pm_init", NULL);
	/* pm_init_dir = proc_mkdir("pm_init", NULL); */
	if (!pm_init_dir) {
		pr_debug("[%s]: mkdir /proc/pm_init failed\n", __func__);
	} else {
		entry = proc_create("cpu_speed_dump", S_IRUGO, pm_init_dir, &cpu_fops);
		entry = proc_create("mfg_speed_dump", S_IRUGO | S_IWGRP, pm_init_dir, &mfg_fops);

#ifdef TOPCK_LDVT
		entry =
		    proc_create("abist_meter_test", S_IRUGO | S_IWUSR, pm_init_dir,
				&abist_meter_fops);
		entry =
		    proc_create("ckgen_meter_test", S_IRUGO | S_IWUSR, pm_init_dir,
				&ckgen_meter_fops);
#endif
	}

#endif


	return 0;
}

arch_initcall(mt_power_management_init);


#if !defined(MT_DORMANT_UT)
static int __init mt_pm_late_init(void)
{
#ifndef CONFIG_MTK_FPGA
/*	mt_idle_init(); */
	clk_buf_init();
#endif
	return 0;
}

late_initcall(mt_pm_late_init);
#endif				/* #if !defined (MT_DORMANT_UT) */


MODULE_DESCRIPTION("MTK Power Management Init Driver");
MODULE_LICENSE("GPL");
