/*****************************************************************************
*
* Filename:
* ---------
*   bq24296m.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq24296m header file
*
* Author:
* -------
*  yueli@tcl.com
*
****************************************************************************/
#ifndef _bq24296m_SW_H_
#define _bq24296m_SW_H_

/* Input Source Control */
#define BQ24296M_REG_ISC						0x00 
#define BQ24296M_REG_ISC_EN_HIZ_MASK			0x1
#define BQ24296M_REG_ISC_EN_HIZ_SHIFT			7
#define BQ24296M_REG_ISC_VINDPM_MASK			0xF
#define BQ24296M_REG_ISC_VINDPM_SHIFT			3
#define BQ24296M_REG_ISC_IINLIM_MASK			0x7
#define BQ24296M_REG_ISC_IINLIM_SHIFT			0

/* Power-On Configuration */
#define BQ24296M_REG_POC						0x01 
#define BQ24296M_REG_POC_RESET_MASK				0x1
#define BQ24296M_REG_POC_RESET_SHIFT			7
#define BQ24296M_REG_POC_WDT_RESET_MASK			0x1
#define BQ24296M_REG_POC_WDT_RESET_SHIFT		6
#define BQ24296M_REG_POC_OTG_CONFIG_MASK		0x1
#define BQ24296M_REG_POC_OTG_CONFIG_SHIFT		5
#define BQ24296M_REG_POC_CHG_CONFIG_MASK		0x1
#define BQ24296M_REG_POC_CHG_CONFIG_SHIFT		4
#define BQ24296M_REG_POC_SYS_MIN_MASK			0x7
#define BQ24296M_REG_POC_SYS_MIN_SHIFT			1
#define BQ24296M_REG_POC_BOOST_LIM_MASK			0x1
#define BQ24296M_REG_POC_BOOST_LIM_SHIFT		0

/* Charge Current Control */
#define BQ24296M_REG_CCC						0x02 
#define BQ24296M_REG_CCC_ICHG_MASK				0x3F
#define BQ24296M_REG_CCC_ICHG_SHIFT				2
#define BQ24296M_REG_CCC_BCLOD_MASK				0x1
#define BQ24296M_REG_CCC_BCLOD_SHIFT			1
#define BQ24296M_REG_CCC_FORCE_20PCT_MASK		0x1
#define BQ24296M_REG_CCC_FORCE_20PCT_SHIFT		0

/* Pre-charge/Termination Current Cntl */
#define BQ24296M_REG_PCTCC						0x03 
#define BQ24296M_REG_PCTCC_IPRECHG_MASK			0xF
#define BQ24296M_REG_PCTCC_IPRECHG_SHIFT		4
#define BQ24296M_REG_PCTCC_ITERM_MASK			0x7
#define BQ24296M_REG_PCTCC_ITERM_SHIFT			0

/* Charge Voltage Control */
#define BQ24296M_REG_CVC						0x04 
#define BQ24296M_REG_CVC_VREG_MASK				0x3F
#define BQ24296M_REG_CVC_VREG_SHIFT				2
#define BQ24296M_REG_CVC_BATLOWV_MASK			0x1
#define BQ24296M_REG_CVC_BATLOWV_SHIFT			1
#define BQ24296M_REG_CVC_VRECHG_MASK			0x1
#define BQ24296M_REG_CVC_VRECHG_SHIFT			0

/* Charge Term/Timer Control */
#define BQ24296M_REG_CTTC						0x05 
#define BQ24296M_REG_CTTC_EN_TERM_MASK			0x1
#define BQ24296M_REG_CTTC_EN_TERM_SHIFT			7
#define BQ24296M_REG_CTTC_WATCHDOG_MASK			0x3
#define BQ24296M_REG_CTTC_WATCHDOG_SHIFT		4
#define BQ24296M_REG_CTTC_EN_TIMER_MASK			0x1
#define BQ24296M_REG_CTTC_EN_TIMER_SHIFT		3
#define BQ24296M_REG_CTTC_CHG_TIMER_MASK		0x3
#define BQ24296M_REG_CTTC_CHG_TIMER_SHIFT		1

/* Boost Voltage/Thermal Regulation Control */
#define BQ24296M_REG_BVTRC						0x06 
#define BQ24296M_REG_BVTRC_BOOSTV_MASK			0xF
#define BQ24296M_REG_BVTRC_BOOSTV_SHIFT			4
#define BQ24296M_REG_BVTRC_BHOT_MASK			0x3
#define BQ24296M_REG_BVTRC_BHOT_SHIFT			2
#define BQ24296M_REG_BVTRC_TREG_MASK			0x3
#define BQ24296M_REG_BVTRC_TREG_SHIFT			0

/* Misc. Operation Control */
#define BQ24296M_REG_MOC						0x07 
#define BQ24296M_REG_MOC_DPDM_EN_MASK			0x1
#define BQ24296M_REG_MOC_DPDM_EN_SHIFT			7
#define BQ24296M_REG_MOC_TMR2X_EN_MASK			0x1
#define BQ24296M_REG_MOC_TMR2X_EN_SHIFT			6
#define BQ24296M_REG_MOC_BATFET_DISABLE_MASK	0x1
#define BQ24296M_REG_MOC_BATFET_DISABLE_SHIFT	5
#define BQ24296M_REG_MOC_INT_MASK_MASK			0x3
#define BQ24296M_REG_MOC_INT_MASK_SHIFT			0

/* System Status */
#define BQ24296M_REG_SS							0x08 
#define BQ24296M_REG_SS_VBUS_STAT_MASK			0x3
#define BQ24296M_REG_SS_VBUS_STAT_SHIFT			6
#define BQ24296M_REG_SS_CHRG_STAT_MASK			0x3
#define BQ24296M_REG_SS_CHRG_STAT_SHIFT			4
#define BQ24296M_REG_SS_DPM_STAT_MASK			0x1
#define BQ24296M_REG_SS_DPM_STAT_SHIFT			3
#define BQ24296M_REG_SS_PG_STAT_MASK			0x1
#define BQ24296M_REG_SS_PG_STAT_SHIFT			2
#define BQ24296M_REG_SS_THERM_STAT_MASK			0x1
#define BQ24296M_REG_SS_THERM_STAT_SHIFT		1
#define BQ24296M_REG_SS_VSYS_STAT_MASK			0x1
#define BQ24296M_REG_SS_VSYS_STAT_SHIFT			0

/* New Fault */
#define BQ24296M_REG_NF							0x09 
#define BQ24296M_REG_NF_WATCHDOG_FAULT_MASK		0x1
#define BQ24296M_REG_NF_WATCHDOG_FAULT_SHIFT	7
#define BQ24296M_REG_NF_OTG_FAULT_MASK			0x1
#define BQ24296M_REG_NF_OTG_FAULT_SHIFT			6
#define BQ24296M_REG_NF_CHRG_FAULT_MASK			0x3
#define BQ24296M_REG_NF_CHRG_FAULT_SHIFT		4
#define BQ24296M_REG_NF_BAT_FAULT_MASK			0x1
#define BQ24296M_REG_NF_BAT_FAULT_SHIFT			3
#define BQ24296M_REG_NF_NTC_FAULT_MASK			0x3
#define BQ24296M_REG_NF_NTC_FAULT_SHIFT			0

/* Vendor/Part/Revision Status */
#define BQ24296M_REG_VPRS						0x0A 
#define BQ24296M_REG_VPRS_PN_MASK				0x7
#define BQ24296M_REG_VPRS_PN_SHIFT				5
#define BQ24296M_REG_VPRS_REV_MASK				0x7
#define BQ24296M_REG_VPRS_REV_SHIFT				0

/**********************************************************
  *
  *   [Extern Function]
  *
  *********************************************************/
/* Input Source Control */
extern void bq24296m_set_en_hiz(unsigned int val);
extern void bq24296m_set_vindpm(unsigned int val);
extern void bq24296m_set_iinlim(unsigned int val);

/* Power-On Configuration */
extern void bq24296m_register_reset(unsigned int val);
extern void bq24296m_wdt_reset(unsigned val);
extern void bq24296m_otg_cfg(unsigned int val);
extern void bq24296m_chg_cfg(unsigned int val);
extern void bq24296m_set_sys_min(unsigned int val);
extern void bq24296m_set_boost_lim(unsigned int val);

/* Charge Current Control */
extern void bq24296m_set_ichg(unsigned int val);
extern unsigned int bq24296m_get_ichg(void);
extern void bq24296m_set_bcold(unsigned int val);
extern void bq24296m_set_force_20pct(unsigned int val);

/* Pre-Charge/Termination Current Control */
extern void bq24296m_set_iprechg(unsigned int val);
extern void bq24296m_set_iterm(unsigned int val);

/* Charge Voltage Control*/
extern void bq24296m_set_vreg(unsigned int val);
extern void bq24296m_set_batlowv(unsigned int val);
extern void bq24296m_set_vrechg(unsigned int val);

/* Charge Termination/Timer Control */
extern void bq24296m_chg_term_en(unsigned int val);
extern void bq24296m_set_wd_timer(unsigned int val);
extern void bq24296m_safety_timer_en(unsigned int val);
extern void bq24296m_set_fast_chg_timer(unsigned int val);

/* Boost Voltage/Thermal Regulation Control */
extern void bq24296m_set_boostv(unsigned int val);
extern void bq24296m_set_bhot(unsigned int val);
extern void bq24296m_set_thermal_reg(unsigned int val);

/* Misc Operation Control */
extern void bq24296m_dpdm_en(unsigned int val);
extern void bq24296m_tmr2x_en(unsigned int val);
extern void bq24296m_batfet_disable(unsigned int val);
extern void bq24296m_set_int(unsigned int val);

/* System Status */
extern unsigned int bq24296m_get_vbus_state(void);
extern unsigned int bq24296m_get_chrg_state(void);
extern unsigned int bq24296m_get_dpm_state(void);
extern unsigned int bq24296m_get_pg_state(void);
extern unsigned int bq24296m_get_therm_state(void);
extern unsigned int bq24296m_get_vsys_state(void);

/* New Fault */
extern unsigned int bq24296m_get_wdt_fault_state(void);
extern unsigned int bq24296m_get_otg_fault_state(void);
extern unsigned int bq24296m_get_chrg_fault_state(void);
extern unsigned int bq24296m_get_bat_fault_state(void);
extern unsigned int bq24296m_get_ntc_fault_state(void);

/* Vender / Part / Revision Status */
extern unsigned int bq24296m_get_ven(void);
extern unsigned int bq24296m_get_rev(void);


extern void bq24296m_dump_register(void);
extern unsigned int bq24296m_reg_config_interface(unsigned char RegNum, unsigned char val);

extern unsigned int bq24296m_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
					 unsigned char SHIFT);
extern unsigned int bq24296m_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
					   unsigned char SHIFT);

/* aggregated APIs */
extern void bq24296m_hw_init(void);

#endif
