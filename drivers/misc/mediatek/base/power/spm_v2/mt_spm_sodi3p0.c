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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <mach/irqs.h>
#include <mach/mt_gpt.h>
#ifdef CONFIG_MTK_WD_KICKER
#include <mach/wd_api.h>
#endif

#include <mt-plat/mt_boot.h>
#if defined(CONFIG_MTK_SYS_CIRQ)
#include <mt-plat/mt_cirq.h>
#endif
#include <mt-plat/upmu_common.h>
#include <mt-plat/mt_io.h>

#include <mt_clkbuf_ctl.h>
#include <mt_spm_sodi3.h>


/**************************************
 * only for internal debug
 **************************************/

#define SODI3_TAG     "[SODI3] "
#define sodi3_err(fmt, args...)		pr_err(SODI3_TAG fmt, ##args)
#define sodi3_warn(fmt, args...)	pr_warn(SODI3_TAG fmt, ##args)
#define sodi3_debug(fmt, args...)	pr_debug(SODI3_TAG fmt, ##args)

#define SPM_BYPASS_SYSPWREQ         0

#define LOG_BUF_SIZE					(256)
#define SODI3_LOGOUT_TIMEOUT_CRITERIA	(20)
#define SODI3_LOGOUT_MAXTIME_CRITERIA	(2000)
#define SODI3_LOGOUT_INTERVAL_CRITERIA	(5000U) /* unit:ms */


static struct pwr_ctrl sodi3_ctrl = {
	.wake_src = WAKE_SRC_FOR_SODI3,

	.wake_src_md32 = WAKE_SRC_FOR_MD32,
	.r0_ctrl_en = 1,
	.r7_ctrl_en = 1,
	.infra_dcm_lock = 1, /* set to be 1 if SODI 3.0 */
	.wfi_op = WFI_OP_AND,

	/* SPM_AP_STANDBY_CON */
	.mp0top_idle_mask = 0,
	.mp1top_idle_mask = 0,
	.mcusys_idle_mask = 0,
	.md_ddr_dbc_en = 0,
	.md1_req_mask_b = 1,
	.md2_req_mask_b = 0, /* bit 20 */
#if defined(CONFIG_ARCH_MT6755)
	.scp_req_mask_b = 0, /* bit 21 */
#elif defined(CONFIG_ARCH_MT6797)
	.scp_req_mask_b = 1, /* bit 21 */
#endif
	.lte_mask_b = 0,
	.md_apsrc1_sel = 0, /* bit 24 */
	.md_apsrc0_sel = 0, /* bit 25 */
	.conn_mask_b = 1,
	.conn_apsrc_sel = 0, /* bit 27 */

	/* SPM_SRC_REQ */
	.spm_apsrc_req = 0,
	.spm_f26m_req = 0,
	.spm_lte_req = 0,
	.spm_infra_req = 0,
	.spm_vrf18_req = 0,
	.spm_dvfs_req = 0,
	.spm_dvfs_force_down = 0,
	.spm_ddren_req = 0,
	.cpu_md_dvfs_sop_force_on = 0,

	/* SPM_SRC_MASK */
	.ccif0_to_md_mask_b = 1,
	.ccif0_to_ap_mask_b = 1,
	.ccif1_to_md_mask_b = 1,
	.ccif1_to_ap_mask_b = 1,
	.ccifmd_md1_event_mask_b = 1,
	.ccifmd_md2_event_mask_b = 1,
	.vsync_mask_b = 0,	/* 5-bit */
	.md_srcclkena_0_infra_mask_b = 0, /* bit 12 */
	.md_srcclkena_1_infra_mask_b = 0, /* bit 13 */
	.conn_srcclkena_infra_mask_b = 0, /* bit 14 */
	.md32_srcclkena_infra_mask_b = 0, /* bit 15 */
	.srcclkeni_infra_mask_b = 0, /* bit 16 */
	.md_apsrcreq_0_infra_mask_b = 1,
	.md_apsrcreq_1_infra_mask_b = 0,
	.conn_apsrcreq_infra_mask_b = 1,
	.md32_apsrcreq_infra_mask_b = 0,
	.md_ddr_en_0_mask_b = 1,
	.md_ddr_en_1_mask_b = 0, /* bit 22 */
	.md_vrf18_req_0_mask_b = 1,
	.md_vrf18_req_1_mask_b = 0, /* bit 24 */
	.emi_bw_dvfs_req_mask = 1,
	.md_srcclkena_0_dvfs_req_mask_b = 0,
	.md_srcclkena_1_dvfs_req_mask_b = 0,
	.conn_srcclkena_dvfs_req_mask_b = 0,

	/* SPM_SRC2_MASK */
	.dvfs_halt_mask_b = 0x1f, /* 5-bit */
	.vdec_req_mask_b = 0, /* bit 6 */
	.gce_req_mask_b = 1, /* bit 7, set to be 1 for SODI */
	.cpu_md_dvfs_erq_merge_mask_b = 0,
	.md1_ddr_en_dvfs_halt_mask_b = 0,
	.md2_ddr_en_dvfs_halt_mask_b = 0,
	.vsync_dvfs_halt_mask_b = 0, /* 5-bit, bit 11 ~ 15 */
	.conn_ddr_en_mask_b = 1,
	.disp_req_mask_b = 1, /* bit 17, set to be 1 for SODI */
	.disp1_req_mask_b = 1, /* bit 18, set to be 1 for SODI */
#if defined(CONFIG_ARCH_MT6755)
	.mfg_req_mask_b = 0, /* bit 19 */
#elif defined(CONFIG_ARCH_MT6797)
	.mfg_req_mask_b = 1, /* bit 19, set to be 1 for SODI */
#endif
	.c2k_ps_rccif_wake_mask_b = 1,
	.c2k_l1_rccif_wake_mask_b = 1,
	.ps_c2k_rccif_wake_mask_b = 1,
	.l1_c2k_rccif_wake_mask_b = 1,
	.sdio_on_dvfs_req_mask_b = 0,
	.emi_boost_dvfs_req_mask_b = 0,
	.cpu_md_emi_dvfs_req_prot_dis = 0,
#if defined(CONFIG_ARCH_MT6797)
	.disp_od_req_mask_b = 1, /* bit 27, set to be 1 for SODI */
#endif
	/* SPM_CLK_CON */
	.srclkenai_mask = 1,

	.mp1_cpu0_wfi_en	= 1,
	.mp1_cpu1_wfi_en	= 1,
	.mp1_cpu2_wfi_en	= 1,
	.mp1_cpu3_wfi_en	= 1,
	.mp0_cpu0_wfi_en	= 1,
	.mp0_cpu1_wfi_en	= 1,
	.mp0_cpu2_wfi_en	= 1,
	.mp0_cpu3_wfi_en	= 1,

#if SPM_BYPASS_SYSPWREQ
	.syspwreq_mask = 1,
#endif
};

struct spm_lp_scen __spm_sodi3 = {
	.pwrctrl = &sodi3_ctrl,
};

static bool gSpm_sodi3_en = true;

static unsigned long int sodi3_logout_prev_time;
static int pre_emi_refresh_cnt;
static int memPllCG_prev_status = 1;	/* 1:CG, 0:pwrdn */
static unsigned int logout_sodi3_cnt;
static unsigned int logout_selfrefresh_cnt;
#if defined(CONFIG_ARCH_MT6755)
static int by_ccif1_count;
#elif defined(CONFIG_ARCH_MT6797)
static unsigned long int logout_prev_dvfs_time;
static unsigned int last_r12;

#define NOT_FREQUENT_EVENT(evt, curr)	((evt != last_r12) || \
					 (((curr) - sodi3_logout_prev_time) > SODI3_LOGOUT_TIMEOUT_CRITERIA))
#define NOT_IGNORE_EVENT(evt)           (((evt)&WAKE_SRC_R12_APXGPT1_EVENT_B) == 0)
#define logout_wakeup_event(evt, curr)	(NOT_IGNORE_EVENT(evt) && NOT_FREQUENT_EVENT(evt, curr))
#endif

static void spm_sodi3_pre_process(void)
{
	u32 val;

	__spm_pmic_pg_force_on();

	spm_disable_mmu_smi_async();

	spm_pmic_power_mode(PMIC_PWR_SODI3, 0, 0);

	spm_bypass_boost_gpio_set();

#if defined(CONFIG_ARCH_MT6755)
	pmic_read_interface_nolock(MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON_ADDR,
					&val,
					MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON_MASK,
					MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON_SHIFT);
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE, IDX_DI_VSRAM_NORMAL, val);
#elif defined(CONFIG_ARCH_MT6797)
	pmic_read_interface_nolock(MT6351_PMIC_RG_VCORE_VDIFF_ENLOWIQ_ADDR, &val, 0xFFFF, 0);
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
			IDX_DI_VCORE_LQ_EN,
			val | (1 << MT6351_PMIC_RG_VCORE_VDIFF_ENLOWIQ_SHIFT));
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
			IDX_DI_VCORE_LQ_DIS,
			val & ~(1 << MT6351_PMIC_RG_VCORE_VDIFF_ENLOWIQ_SHIFT));

	__spm_pmic_low_iq_mode(1);
#endif

	pmic_read_interface_nolock(MT6351_TOP_CON, &val, 0x037F, 0);
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
					IDX_DI_SRCCLKEN_IN2_NORMAL,
					val | (1 << MT6351_PMIC_RG_SRCLKEN_IN2_EN_SHIFT));
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
					IDX_DI_SRCCLKEN_IN2_SLEEP,
					val & ~(1 << MT6351_PMIC_RG_SRCLKEN_IN2_EN_SHIFT));

	/* set PMIC WRAP table for deepidle power control */
	mt_spm_pmic_wrap_set_phase(PMIC_WRAP_PHASE_DEEPIDLE);

	/* for afcdac setting */
	clk_buf_write_afcdac();

	/* Do more low power setting when MD1/C2K/CONN off */
	if (is_md_c2k_conn_power_off()) {
		__spm_bsi_top_init_setting();
#if defined(CONFIG_ARCH_MT6755)
		__spm_backup_pmic_ck_pdn();
#endif
	}
}

static void spm_sodi3_post_process(void)
{
#if defined(CONFIG_ARCH_MT6755)
	/* Do more low power setting when MD1/C2K/CONN off */
	if (is_md_c2k_conn_power_off())
		__spm_restore_pmic_ck_pdn();
#elif defined(CONFIG_ARCH_MT6797)
	__spm_pmic_low_iq_mode(0);
#endif

	/* set PMIC WRAP table for normal power control */
	mt_spm_pmic_wrap_set_phase(PMIC_WRAP_PHASE_NORMAL);

	spm_enable_mmu_smi_async();
	__spm_pmic_pg_force_off();
}

static wake_reason_t
spm_sodi3_output_log(struct wake_status *wakesta, struct pcm_desc *pcmdesc, int vcore_status, u32 sodi3_flags)
{
	wake_reason_t wr = WR_NONE;
	unsigned long int sodi3_logout_curr_time = 0;
	int need_log_out = 0;

	if (sodi3_flags&SODI_FLAG_NO_LOG) {
		if ((wakesta->assert_pc != 0) || (wakesta->r12 == 0)) {
			sodi3_err("PCM ASSERT AT SPM_PC = 0x%0x (%s), R12 = 0x%x, R13 = 0x%x, DEBUG_FLAG = 0x%x\n",
				wakesta->assert_pc, pcmdesc->version, wakesta->r12, wakesta->r13, wakesta->debug_flag);
			wr = WR_PCM_ASSERT;
		}
	} else if (!(sodi3_flags&SODI_FLAG_REDUCE_LOG) || (sodi3_flags & SODI_FLAG_RESIDENCY)) {
		sodi3_warn("vcore_status = %d, self_refresh = 0x%x, sw_flag = 0x%x, 0x%x, %s\n",
				vcore_status, spm_read(SPM_PASR_DPD_0), spm_read(SPM_SW_FLAG),
				spm_read(DUMMY1_PWR_CON), pcmdesc->version);

		wr = __spm_output_wake_reason(wakesta, pcmdesc, false);
	} else {
		/*
		 * Log reduction mechanism, print debug information criteria :
		 * 1. SPM assert or No wakeup event
		 * 2. Not wakeup by GPT
		 * 3. Residency is less than 20 ticks of 32KHz
		 * 4. Emi self-refresh state is changed
		 * 5. Time from the last output log is larger than 5 sec
		 * 6. CG/PD state is changed
		*/
		sodi3_logout_curr_time = spm_get_current_time_ms();

		if ((wakesta->assert_pc != 0) || (wakesta->r12 == 0)) {
#if defined(CONFIG_ARCH_MT6755)
			need_log_out = 1;
#elif defined(CONFIG_ARCH_MT6797)
			if (wakesta->r12_ext != WAKE_SRC_R12_EXT_VCORE_DVFS_B) {
				need_log_out = 1;
			} else if (sodi3_logout_curr_time - logout_prev_dvfs_time > 20U) {
				wakesta->r12 = WAKE_SRC_R12_PCM_TIMER;
				need_log_out = 1;
				logout_prev_dvfs_time = sodi3_logout_curr_time;
			}
#endif
#if defined(CONFIG_ARCH_MT6755)
		} else if ((wakesta->r12 & (0x1 << 4)) == 0) {
			if (wakesta->r12 & (0x1 << 18)) {
				/* wake up by R12_CCIF1_EVENT_B */
				if ((by_ccif1_count >= 5) ||
				    ((sodi3_logout_curr_time - sodi3_logout_prev_time) > 20U)) {
					need_log_out = 1;
					by_ccif1_count = 0;
				} else if (by_ccif1_count == 0) {
					need_log_out = 1;
				}
				by_ccif1_count++;
			}
#elif defined(CONFIG_ARCH_MT6797)
		} else if (logout_wakeup_event(wakesta->r12, sodi3_logout_curr_time)) {
			need_log_out = 1;
#endif
		} else if ((wakesta->timer_out <= SODI3_LOGOUT_TIMEOUT_CRITERIA) ||
			   (wakesta->timer_out >= SODI3_LOGOUT_MAXTIME_CRITERIA)) {
			need_log_out = 1;
		} else if ((spm_read(SPM_PASR_DPD_0) == 0 && pre_emi_refresh_cnt > 0) ||
				(spm_read(SPM_PASR_DPD_0) > 0 && pre_emi_refresh_cnt == 0)) {
			need_log_out = 1;
		} else if ((sodi3_logout_curr_time - sodi3_logout_prev_time) > SODI3_LOGOUT_INTERVAL_CRITERIA) {
			need_log_out = 1;
		} else {
			int mem_status = 0;

			if (((spm_read(SPM_SW_FLAG) & SPM_FLAG_SODI_CG_MODE) != 0) ||
				((spm_read(DUMMY1_PWR_CON) & DUMMY1_PWR_ISO_LSB) != 0))
				mem_status = 1;

			if (memPllCG_prev_status != mem_status) {
				memPllCG_prev_status = mem_status;
				need_log_out = 1;
			}
		}

		logout_sodi3_cnt++;
		logout_selfrefresh_cnt += spm_read(SPM_PASR_DPD_0);
		pre_emi_refresh_cnt = spm_read(SPM_PASR_DPD_0);
#if defined(CONFIG_ARCH_MT6797)
		last_r12 = wakesta->r12;
#endif
		if (need_log_out == 1) {
			sodi3_logout_prev_time = sodi3_logout_curr_time;

			if ((wakesta->assert_pc != 0) || (wakesta->r12 == 0)) {
				if (wakesta->assert_pc != 0) {
					sodi3_err("Warning: wakeup reason is WR_PCM_ASSERT!\n");
					wr = WR_PCM_ASSERT;
				} else if (wakesta->r12 == 0) {
					sodi3_err("Warning: wakeup reason is WR_UNKNOWN!\n");
					wr = WR_UNKNOWN;
				}

				sodi3_err("VCORE_STATUS = %d, SELF_REFRESH = 0x%x, SW_FLAG = 0x%x, 0x%x, %s\n",
						vcore_status, spm_read(SPM_PASR_DPD_0), spm_read(SPM_SW_FLAG),
						spm_read(DUMMY1_PWR_CON), pcmdesc->version);

				sodi3_err("SODI3_CNT = %d, SELF_REFRESH_CNT = 0x%x, SPM_PC = 0x%0x, R13 = 0x%x, DEBUG_FLAG = 0x%x\n",
						logout_sodi3_cnt, logout_selfrefresh_cnt,
						wakesta->assert_pc, wakesta->r13, wakesta->debug_flag);

				sodi3_err("R12 = 0x%x, R12_E = 0x%x, RAW_STA = 0x%x, IDLE_STA = 0x%x, EVENT_REG = 0x%x, ISR = 0x%x\n",
						wakesta->r12, wakesta->r12_ext, wakesta->raw_sta, wakesta->idle_sta,
						wakesta->event_reg, wakesta->isr);
				wr = WR_PCM_ASSERT;
			} else {
				char buf[LOG_BUF_SIZE] = { 0 };
				int i;

				if (wakesta->r12 & WAKE_SRC_R12_PCM_TIMER) {
					if (wakesta->wake_misc & WAKE_MISC_PCM_TIMER)
						strncat(buf, " PCM_TIMER", sizeof(buf) - strlen(buf) - 1);

					if (wakesta->wake_misc & WAKE_MISC_TWAM)
						strncat(buf, " TWAM", sizeof(buf) - strlen(buf) - 1);

					if (wakesta->wake_misc & WAKE_MISC_CPU_WAKE)
						strncat(buf, " CPU", sizeof(buf) - strlen(buf) - 1);

#if defined(CONFIG_ARCH_MT6797)
					if (wakesta->r12_ext == WAKE_SRC_R12_EXT_VCORE_DVFS_B) {
						strncat(buf, " vcore dvfs", sizeof(buf) - strlen(buf) - 1);
					}
#endif
				}
				for (i = 1; i < 32; i++) {
					if (wakesta->r12 & (1U << i)) {
						strncat(buf, wakesrc_str[i], sizeof(buf) - strlen(buf) - 1);
						wr = WR_WAKE_SRC;
					}
				}
				BUG_ON(strlen(buf) >= LOG_BUF_SIZE);

				sodi3_warn("wake up by %s, vcore_status = %d, self_refresh = 0x%x, sw_flag = 0x%x, 0x%x, %s, %d, 0x%x, timer_out = %u, r13 = 0x%x, debug_flag = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
						buf, vcore_status, spm_read(SPM_PASR_DPD_0), spm_read(SPM_SW_FLAG),
						spm_read(DUMMY1_PWR_CON), pcmdesc->version,
						logout_sodi3_cnt, logout_selfrefresh_cnt,
						wakesta->timer_out, wakesta->r13, wakesta->debug_flag,
						wakesta->r12, wakesta->r12_ext, wakesta->raw_sta, wakesta->idle_sta,
						wakesta->event_reg, wakesta->isr);
			}

			logout_sodi3_cnt = 0;
			logout_selfrefresh_cnt = 0;
		}

	}

	return wr;
}

wake_reason_t spm_go_to_sodi3(u32 spm_flags, u32 spm_data, u32 sodi3_flags)
{
	u32 sec = 2;
#ifdef CONFIG_MTK_WD_KICKER
	int wd_ret;
	struct wd_api *wd_api;
#endif
	u32 con1;
	struct wake_status wakesta;
	unsigned long flags;
	struct mtk_irq_mask *mask;
	wake_reason_t wr = WR_NONE;
	struct pcm_desc *pcmdesc;
	struct pwr_ctrl *pwrctrl = __spm_sodi3.pwrctrl;
	int vcore_status = vcorefs_get_curr_ddr();
	u32 cpu = spm_data;
	u32 sodi_idx;

	sodi_idx = DYNA_LOAD_PCM_SODI + cpu / 4;

	if (!dyna_load_pcm[sodi_idx].ready) {
		sodi3_err("ERROR: LOAD FIRMWARE FAIL\n");
		BUG();
	}
	pcmdesc = &(dyna_load_pcm[sodi_idx].desc);

	spm_sodi3_footprint(SPM_SODI3_ENTER);

	if (spm_get_sodi_mempll() == 1)
		spm_flags |= SPM_FLAG_SODI_CG_MODE;	/* CG mode */
	else
		spm_flags &= ~SPM_FLAG_SODI_CG_MODE;	/* PDN mode */

	update_pwrctrl_pcm_flags(&spm_flags);
	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

	pwrctrl->timer_val = sec * 32768;

#ifdef CONFIG_MTK_WD_KICKER
	wd_ret = get_wd_api(&wd_api);
	if (!wd_ret) {
		wd_api->wd_spmwdt_mode_config(WD_REQ_EN, WD_REQ_RST_MODE);
		wd_api->wd_suspend_notify();
	}
#endif

	/* enable APxGPT timer */
	soidle3_before_wfi(cpu);

	lockdep_off();
	spin_lock_irqsave(&__spm_lock, flags);

	mask = kmalloc(sizeof(struct mtk_irq_mask), GFP_ATOMIC);
	if (!mask) {
		wr = -ENOMEM;
		goto UNLOCK_SPM;
	}
	mt_irq_mask_all(mask);
	mt_irq_unmask_for_sleep(SPM_IRQ0_ID);
#if defined(CONFIG_ARCH_MT6797)	/* unmask for edge trigger interrupt */
	mt_irq_unmask_for_sleep(169);	/* for WDT */
	mt_irq_unmask_for_sleep(319);	/* for C2K_WDT */
	mt_irq_unmask_for_sleep(211);	/* for KP_IRQ */
	mt_irq_unmask_for_sleep(298);	/* for MD_WDT */
	mt_irq_unmask_for_sleep(317);	/* for CONN_WDT */
#endif
#if defined(CONFIG_MTK_SYS_CIRQ)
	mt_cirq_clone_gic();
	mt_cirq_enable();
#endif

	spm_sodi3_footprint(SPM_SODI3_ENTER_UART_SLEEP);

	if (request_uart_to_sleep()) {
		wr = WR_UART_BUSY;
		goto RESTORE_IRQ;
	}

	spm_sodi3_footprint(SPM_SODI3_ENTER_SPM_FLOW);

	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

#if defined(CONFIG_ARCH_MT6755)
	__spm_check_md_pdn_power_control(pwrctrl);
#endif

	__spm_sync_vcore_dvfs_power_control(pwrctrl, __spm_vcore_dvfs.pwrctrl);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	/* PCM WDT WAKE MODE for lastPC */
	con1 = spm_read(PCM_CON1) & ~(PCM_WDT_WAKE_MODE_LSB | PCM_WDT_EN_LSB);
	spm_write(PCM_CON1, SPM_REGWR_CFG_KEY | con1);
	if (spm_read(PCM_TIMER_VAL) > PCM_TIMER_MAX)
		spm_write(PCM_TIMER_VAL, PCM_TIMER_MAX);
	spm_write(PCM_WDT_VAL, spm_read(PCM_TIMER_VAL) + PCM_WDT_TIMEOUT);
	if (pwrctrl->timer_val_cust == 0)
		spm_write(PCM_CON1, con1 | SPM_REGWR_CFG_KEY | PCM_WDT_EN_LSB | PCM_TIMER_EN_LSB);
	else
		spm_write(PCM_CON1, con1 | SPM_REGWR_CFG_KEY | PCM_WDT_EN_LSB);

	spm_sodi3_pre_process();

	__spm_kick_pcm_to_run(pwrctrl);

	spm_sodi3_footprint_val((1 << SPM_SODI3_ENTER_WFI) |
				(1 << SPM_SODI3_B3) | (1 << SPM_SODI3_B4) |
				(1 << SPM_SODI3_B5) | (1 << SPM_SODI3_B6));

#ifdef SPM_SODI3_PROFILE_TIME
	gpt_get_cnt(SPM_SODI3_PROFILE_APXGPT, &soidle3_profile[1]);
#endif

	spm_trigger_wfi_for_sodi(pwrctrl);

#ifdef SPM_SODI3_PROFILE_TIME
	gpt_get_cnt(SPM_SODI3_PROFILE_APXGPT, &soidle3_profile[2]);
#endif

	spm_sodi3_footprint(SPM_SODI3_LEAVE_WFI);

	spm_sodi3_post_process();

	__spm_get_wakeup_status(&wakesta);

	/* disable PCM WDT to stop count if needed */
	spm_write(PCM_CON1, SPM_REGWR_CFG_KEY | (spm_read(PCM_CON1) & ~PCM_WDT_EN_LSB));

	__spm_clean_after_wakeup();

	spm_sodi3_footprint(SPM_SODI3_ENTER_UART_AWAKE);

	request_uart_to_wakeup();

	wr = spm_sodi3_output_log(&wakesta, pcmdesc, vcore_status, sodi3_flags);

	spm_sodi3_footprint(SPM_SODI3_LEAVE_SPM_FLOW);

RESTORE_IRQ:
#if defined(CONFIG_MTK_SYS_CIRQ)
	mt_cirq_flush();
	mt_cirq_disable();
#endif
	mt_irq_mask_restore(mask);
	kfree(mask);

UNLOCK_SPM:
	spin_unlock_irqrestore(&__spm_lock, flags);
	lockdep_on();

	/* stop APxGPT timer and enable caore0 local timer */
	soidle3_after_wfi(cpu);
#ifdef CONFIG_MTK_WD_KICKER
	if (!wd_ret) {
		wd_api->wd_resume_notify();
		wd_api->wd_spmwdt_mode_config(WD_REQ_DIS, WD_REQ_RST_MODE);
	}
#endif

#if defined(CONFIG_ARCH_MT6797)
	if (wr != WR_UART_BUSY) {
		spm_sodi3_footprint(SPM_SODI3_REKICK_VCORE);
		spm_write(PCM_CON1, SPM_REGWR_CFG_KEY | (spm_read(PCM_CON1) & ~PCM_TIMER_EN_LSB));
		__spm_backup_vcore_dvfs_dram_shuffle();
		vcorefs_go_to_vcore_dvfs();
	}
#endif

	spm_sodi3_reset_footprint();
	return wr;
}

void spm_enable_sodi3(bool en)
{
	gSpm_sodi3_en = en;
}

bool spm_get_sodi3_en(void)
{
	return gSpm_sodi3_en;
}

void spm_sodi3_init(void)
{
	sodi3_debug("spm_sodi3_init\n");
#ifdef SPM_SODI3_PROFILE_TIME
	request_gpt(SPM_SODI3_PROFILE_APXGPT, GPT_FREE_RUN, GPT_CLK_SRC_RTC, GPT_CLK_DIV_1,
			  0, NULL, GPT_NOIRQEN);
#endif
	spm_sodi3_aee_init();
}

MODULE_DESCRIPTION("SPM-SODI3 Driver v0.1");
