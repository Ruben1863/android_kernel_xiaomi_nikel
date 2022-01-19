#include <linux/types.h>
#include <linux/init.h>	
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include "bq24296m.h"
/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/

#ifdef CONFIG_OF
#else

#define bq24296m_SLAVE_ADDR_WRITE   0x6B
#define bq24296m_SLAVE_ADDR_Read    0x6C

#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define bq24296m_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define bq24296m_BUSNUM 1
#endif

#endif

#define bq24296m_REG_NUM 11
static struct i2c_client *new_client;
static const struct i2c_device_id bq24296m_i2c_id[] = { {"bq24296m", 0}, {} };

kal_bool chargin_hw_init_done = false;
static int bq24296m_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq24296m_reg[bq24296m_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq24296m_i2c_access);

int g_bq24296m_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24296m]
  *
  *********************************************************/
#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int bq24296m_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&bq24296m_i2c_access);

	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&bq24296m_i2c_access);

		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	new_client->ext_flag = 0;
	mutex_unlock(&bq24296m_i2c_access);

	return 1;
}

unsigned int bq24296m_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&bq24296m_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&bq24296m_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&bq24296m_i2c_access);
	return 1;
}
#else
unsigned int bq24296m_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&bq24296m_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = new_client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, &msgs[xfers], xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&bq24296m_i2c_access);

	return ret == xfers ? 0 : -1;
}

unsigned int bq24296m_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&bq24296m_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, &msgs[xfers], xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&bq24296m_i2c_access);

	return ret == xfers ? 0 : -1;
}
#endif
/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq24296m_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq24296m_reg = 0;
	unsigned int ret = 0;

	ret = bq24296m_read_byte(RegNum, &bq24296m_reg);

	battery_log(BAT_LOG_FULL, "[bq24296m_read_interface] Reg[%x]=0x%x\n", RegNum, bq24296m_reg);

	bq24296m_reg &= (MASK << SHIFT);
	*val = (bq24296m_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[bq24296m_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq24296m_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq24296m_reg = 0;
	unsigned int ret = 0;

	ret = bq24296m_read_byte(RegNum, &bq24296m_reg);
	battery_log(BAT_LOG_FULL, "[bq24296m_config_interface] Reg[%x]=0x%x\n", RegNum, bq24296m_reg);

	bq24296m_reg &= ~(MASK << SHIFT);
	bq24296m_reg |= (val << SHIFT);

	ret = bq24296m_write_byte(RegNum, bq24296m_reg);
	battery_log(BAT_LOG_FULL, "[bq24296m_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    bq24296m_reg);

	return ret;
}

/* write one register directly */
unsigned int bq24296m_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = bq24296m_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* Input Source Control */
void bq24296m_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_ISC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_ISC_EN_HIZ_MASK),
				       (unsigned char) (BQ24296M_REG_ISC_EN_HIZ_SHIFT)
	    );
}

void bq24296m_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_ISC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_ISC_VINDPM_MASK),
				       (unsigned char) (BQ24296M_REG_ISC_VINDPM_SHIFT)
	    );
}

void bq24296m_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_ISC),
				       (val),
				       (unsigned char) (BQ24296M_REG_ISC_IINLIM_MASK),
				       (unsigned char) (BQ24296M_REG_ISC_IINLIM_SHIFT)
	    );
}

unsigned int bq24296m_get_iinlim(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_ISC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_ISC_IINLIM_MASK), 
				     (unsigned char) (BQ24296M_REG_ISC_IINLIM_SHIFT)
	    );
	return val;
}

/* Power-On Configuration */
void bq24296m_register_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_POC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_POC_RESET_MASK),
				       (unsigned char) (BQ24296M_REG_POC_RESET_SHIFT)
	    );
}

void bq24296m_wdt_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_POC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_POC_WDT_RESET_MASK),
				       (unsigned char) (BQ24296M_REG_POC_WDT_RESET_SHIFT)
	    );
}

void bq24296m_otg_cfg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_POC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_POC_OTG_CONFIG_MASK),
				       (unsigned char) (BQ24296M_REG_POC_OTG_CONFIG_SHIFT)
	    );
}
void bq24296m_chg_cfg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_POC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_POC_CHG_CONFIG_MASK),
				       (unsigned char) (BQ24296M_REG_POC_CHG_CONFIG_SHIFT)
	    );
}
unsigned int bq24296m_get_chg_cfg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_POC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_POC_CHG_CONFIG_MASK),
				     (unsigned char) (BQ24296M_REG_POC_CHG_CONFIG_SHIFT)
	    );
	return val;
}

void bq24296m_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_POC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_POC_SYS_MIN_MASK),
				       (unsigned char) (BQ24296M_REG_POC_SYS_MIN_SHIFT)
	    );
}
unsigned int bq24296m_get_sys_min(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_POC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_POC_SYS_MIN_MASK),
				     (unsigned char) (BQ24296M_REG_POC_SYS_MIN_SHIFT)
	    );
	return val;
}

void bq24296m_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_POC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_POC_BOOST_LIM_MASK),
				       (unsigned char) (BQ24296M_REG_POC_BOOST_LIM_SHIFT)
	    );
}

/* Charge Current Control */
void bq24296m_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CCC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_CCC_ICHG_MASK),
				       (unsigned char) (BQ24296M_REG_CCC_ICHG_SHIFT)
	    );
}
unsigned int bq24296m_get_ichg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_CCC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_CCC_ICHG_MASK),
				     (unsigned char) (BQ24296M_REG_CCC_ICHG_SHIFT)
	    );
	return val;
}

void bq24296m_set_bcold(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CCC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_CCC_BCLOD_MASK),
				       (unsigned char) (BQ24296M_REG_CCC_BCLOD_SHIFT)
	    );
}
void bq24296m_set_force_20pct(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CCC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_CCC_FORCE_20PCT_MASK),
				       (unsigned char) (BQ24296M_REG_CCC_FORCE_20PCT_SHIFT)
	    );
}

/* Pre-Charge/Termination Current Control */
void bq24296m_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_PCTCC),
				       (val),
				       (unsigned char) (BQ24296M_REG_PCTCC_IPRECHG_MASK), 
				       (unsigned char) (BQ24296M_REG_PCTCC_IPRECHG_SHIFT)
	    );
}
unsigned int bq24296m_get_iprechg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_PCTCC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_PCTCC_IPRECHG_MASK),
				     (unsigned char) (BQ24296M_REG_PCTCC_IPRECHG_SHIFT)
	    );
	return val;
}

void bq24296m_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_PCTCC),
				       (val),
				       (unsigned char) (BQ24296M_REG_PCTCC_ITERM_MASK),
				       (unsigned char) (BQ24296M_REG_PCTCC_ITERM_SHIFT)
	    );
}
unsigned int bq24296m_get_iterm(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_PCTCC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_PCTCC_ITERM_MASK),
				     (unsigned char) (BQ24296M_REG_PCTCC_ITERM_SHIFT)
	    );
	return val;
}

/* Charge Voltage Control*/
void bq24296m_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CVC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_CVC_VREG_MASK),
				       (unsigned char) (BQ24296M_REG_CVC_VREG_SHIFT)
	    );
}
unsigned int bq24296m_get_vreg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_CVC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_CVC_VREG_MASK),
				     (unsigned char) (BQ24296M_REG_CVC_VREG_SHIFT)
	    );
	return val;
}

void bq24296m_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CVC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_CVC_BATLOWV_MASK), 
				       (unsigned char) (BQ24296M_REG_CVC_BATLOWV_SHIFT)
	    );
}
unsigned int bq24296m_get_batlowv(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_CVC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_CVC_BATLOWV_MASK),
				     (unsigned char) (BQ24296M_REG_CVC_BATLOWV_SHIFT)
	    );
	return val;
}

void bq24296m_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CVC),
				     (unsigned char) (val),
				     (unsigned char) (BQ24296M_REG_CVC_VRECHG_MASK), 
				     (unsigned char) (BQ24296M_REG_CVC_VRECHG_SHIFT)
	    );
}
unsigned int bq24296m_get_vrechg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_CVC),
				     (&val),
				     (unsigned char) (BQ24296M_REG_CVC_VRECHG_MASK),
				     (unsigned char) (BQ24296M_REG_CVC_VRECHG_SHIFT)
	    );
	return val;
}

/* Charge Termination/Timer Control */
void bq24296m_chg_term_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CTTC),
				       (val),
				       (unsigned char) (BQ24296M_REG_CTTC_EN_TERM_MASK),
				       (unsigned char) (BQ24296M_REG_CTTC_EN_TERM_SHIFT)
	    );
}

void bq24296m_set_wd_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CTTC),
				       (val),
				       (unsigned char) (BQ24296M_REG_CTTC_WATCHDOG_MASK), 
				       (unsigned char) (BQ24296M_REG_CTTC_WATCHDOG_SHIFT)
	    );
}
void bq24196m_safety_timer_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CTTC),
				       (val),
				       (unsigned char) (BQ24296M_REG_CTTC_EN_TIMER_MASK),
				       (unsigned char) (BQ24296M_REG_CTTC_EN_TIMER_SHIFT)
	    );
}

void bq24296m_set_fast_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_CTTC),
				       (val),
				       (unsigned char) (BQ24296M_REG_CTTC_CHG_TIMER_MASK), 
				       (unsigned char) (BQ24296M_REG_CTTC_CHG_TIMER_SHIFT)
	    );
}

/* Boost Voltage/Thermal Regulation Control */
void bq24296m_set_boostv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_BVTRC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_BVTRC_BOOSTV_MASK),
				       (unsigned char) (BQ24296M_REG_BVTRC_BOOSTV_SHIFT)
	    );
}

void bq24296m_set_bhot(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_BVTRC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_BVTRC_BHOT_MASK),
				       (unsigned char) (BQ24296M_REG_BVTRC_BHOT_SHIFT)
	    );
}

void bq24296m_set_thermal_reg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_BVTRC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_BVTRC_TREG_MASK),
				       (unsigned char) (BQ24296M_REG_BVTRC_TREG_SHIFT)
	    );
}

/* Misc Operation Control */
void bq24296m_dpdm_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_MOC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_MOC_DPDM_EN_MASK),
				       (unsigned char) (BQ24296M_REG_MOC_DPDM_EN_SHIFT)
	    );
}

void bq24296m_safety_timer_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_MOC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_MOC_TMR2X_EN_MASK),
				       (unsigned char) (BQ24296M_REG_MOC_TMR2X_EN_SHIFT)
	    );
}

void bq24296m_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_MOC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_MOC_BATFET_DISABLE_MASK),
				       (unsigned char) (BQ24296M_REG_MOC_BATFET_DISABLE_SHIFT)
	    );
}

void bq24296m_set_int(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296m_config_interface((unsigned char) (BQ24296M_REG_MOC),
				       (unsigned char) (val),
				       (unsigned char) (BQ24296M_REG_MOC_INT_MASK_MASK),
				       (unsigned char) (BQ24296M_REG_MOC_INT_MASK_SHIFT)
	    );
}

/* System Status */
unsigned int bq24296m_get_vbus_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_SS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_SS_VBUS_STAT_MASK),
				     (unsigned char) (BQ24296M_REG_SS_VBUS_STAT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_chrg_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_SS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_SS_CHRG_STAT_MASK),
				     (unsigned char) (BQ24296M_REG_SS_CHRG_STAT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_dpm_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_SS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_SS_DPM_STAT_MASK),
				     (unsigned char) (BQ24296M_REG_SS_DPM_STAT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_pg_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_SS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_SS_PG_STAT_MASK),
				     (unsigned char) (BQ24296M_REG_SS_PG_STAT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_therm_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_SS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_SS_THERM_STAT_MASK),
				     (unsigned char) (BQ24296M_REG_SS_THERM_STAT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_vsys_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_SS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_SS_VSYS_STAT_MASK),
				     (unsigned char) (BQ24296M_REG_SS_VSYS_STAT_SHIFT)
	    );
	return val;
}

/* New Fault */
unsigned int bq24296m_get_wdt_fault_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_NF),
				     (&val),
				     (unsigned char) (BQ24296M_REG_NF_WATCHDOG_FAULT_MASK),
				     (unsigned char) (BQ24296M_REG_NF_WATCHDOG_FAULT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_otg_fault_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_NF),
				     (&val),
				     (unsigned char) (BQ24296M_REG_NF_OTG_FAULT_MASK),
				     (unsigned char) (BQ24296M_REG_NF_OTG_FAULT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_chrg_fault_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_NF),
				     (&val),
				     (unsigned char) (BQ24296M_REG_NF_CHRG_FAULT_MASK),
				     (unsigned char) (BQ24296M_REG_NF_CHRG_FAULT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_bat_fault_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_NF),
				     (&val),
				     (unsigned char) (BQ24296M_REG_NF_BAT_FAULT_MASK),
				     (unsigned char) (BQ24296M_REG_NF_BAT_FAULT_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_ntc_fault_state(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_NF),
				     (&val),
				     (unsigned char) (BQ24296M_REG_NF_NTC_FAULT_MASK),
				     (unsigned char) (BQ24296M_REG_NF_NTC_FAULT_SHIFT)
	    );
	return val;
}

/* Vender / Part / Revision Status */
unsigned int bq24296m_get_ven(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_VPRS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_VPRS_PN_MASK),
				     (unsigned char) (BQ24296M_REG_VPRS_PN_SHIFT)
	    );
	return val;
}
unsigned int bq24296m_get_rev(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_read_interface((unsigned char) (BQ24296M_REG_VPRS),
				     (&val),
				     (unsigned char) (BQ24296M_REG_VPRS_REV_MASK),
				     (unsigned char) (BQ24296M_REG_VPRS_REV_SHIFT)
	    );
	return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void bq24296m_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296m_get_ven();

	if (val == 0)
		g_bq24296m_hw_exist = 0;
	else
		g_bq24296m_hw_exist = 1;

	battery_log(BAT_LOG_CRTI,"[bq24296m_hw_component_detect] exist=%d, Reg[0x03]=0x%x\n",
		 g_bq24296m_hw_exist, val);
}

int is_bq24296m_exist(void)
{
	pr_debug("[is_bq24296m_exist] g_bq24296m_hw_exist=%d\n", g_bq24296m_hw_exist);

	return g_bq24296m_hw_exist;
}

void bq24296m_dump_register(void)
{
	unsigned int i = 0;
	unsigned int ichg = 0;
	unsigned int vsys_min = 0;
	unsigned int vchg = 0;
	unsigned int iprechg = 0;
	unsigned int iterm = 0;
	unsigned int vrechg = 0;
	unsigned int chrg_state = 0;
	unsigned int chr_en = 0;
	unsigned int vbatlow = 0;
	unsigned int fault = 0;

	for (i = 0; i < bq24296m_REG_NUM; i++) {
		bq24296m_read_byte(i, &bq24296m_reg[i]);
		battery_log(BAT_LOG_CRTI, "[bq24296m reg@][0x%x]=0x%x ", i, bq24296m_reg[i]);
	}

	chrg_state = bq24296m_get_chrg_state();
	chr_en = bq24296m_get_chg_cfg();
	ichg = bq24296m_get_ichg();
	vchg = bq24296m_get_vreg();
	vsys_min = bq24296m_get_sys_min();
	iprechg = bq24296m_get_iprechg();
	iterm = bq24296m_get_iterm();
	vrechg = bq24296m_get_vrechg();
	vbatlow = bq24296m_get_batlowv();
	fault = bq24296m_get_chrg_fault_state();
	
	battery_log(BAT_LOG_CRTI, "BQ24296 Ichg = %dmA, Vchg = %dmV, Vsys_min =%dmV, Iprechg = %dmA, iterm = %dmA, Vrechg = %d, Vbatlow = %d, chrg_state = %d, err = %d\n",
		    ichg * 64 + 512, vchg * 16 + 3504, vsys_min *100 + 3000, (iprechg+1)*128, (iterm+1)*128, vrechg, vbatlow, chrg_state, fault);

}

void bq24296m_hw_init(void)
{
	bq24296m_dump_register();
}

static int bq24296m_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	battery_log(BAT_LOG_CRTI, "[bq24296m_driver_probe]\n");
	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;

	/* --------------------- */
	bq24296m_hw_component_detect();
	bq24296m_dump_register();
	/* bq24296m_hw_init(); //move to charging_hw_xxx.c */
	chargin_hw_init_done = true;

	return 0;

exit:
	return err;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq24296m = 0;
static ssize_t show_bq24296m_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq24296m_access] 0x%x\n", g_reg_value_bq24296m);
	return sprintf(buf, "%u\n", g_reg_value_bq24296m);
}

static ssize_t store_bq24296m_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	/*char *pvalue = NULL;*/
	unsigned int reg_value = 0;
	unsigned long int reg_address = 0;
	int rv;

	battery_log(BAT_LOG_CRTI, "[store_bq24296m_access]\n");

	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[store_bq24296m_access] buf is %s and size is %zu\n", buf,
			    size);
		/*reg_address = simple_strtoul(buf, &pvalue, 16);*/
		rv = kstrtoul(buf, 0, &reg_address);
			if (rv != 0)
				return -EINVAL;
		/*ret = kstrtoul(buf, 16, reg_address); *//* This must be a null terminated string */
		if (size > 3) {
			/*NEED to check kstr*/
			/*reg_value = simple_strtoul((pvalue + 1), NULL, 16);*/
			/*ret = kstrtoul(buf + 3, 16, reg_value); */
			battery_log(BAT_LOG_CRTI,
				    "[store_bq24296m_access] write bq24296m reg 0x%x with value 0x%x !\n",
				    (unsigned int) reg_address, reg_value);
			ret = bq24296m_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq24296m_read_interface(reg_address, &g_reg_value_bq24296m, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
				    "[store_bq24296m_access] read bq24296m reg 0x%x with value 0x%x !\n",
				    (unsigned int) reg_address, g_reg_value_bq24296m);
			battery_log(BAT_LOG_CRTI,
				    "[store_bq24296m_access] Please use \"cat bq24296m_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(bq24296m_access, 0664, show_bq24296m_access, store_bq24296m_access);	/* 664 */

static int bq24296m_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** bq24296m_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24296m_access);

	return 0;
}

struct platform_device bq24296m_user_space_device = {
	.name = "bq24296m-user",
	.id = -1,
};

static struct platform_driver bq24296m_user_space_driver = {
	.probe = bq24296m_user_space_probe,
	.driver = {
		   .name = "bq24296m-user",
		   },
};

#ifdef CONFIG_OF
static const struct of_device_id bq24296m_of_match[] = {
	{.compatible = "mediatek,SWITHING_CHARGER"},
	{},
};
#else
static struct i2c_board_info i2c_bq24296m __initdata = {
	I2C_BOARD_INFO("bq24296m", (bq24296m_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver bq24296m_driver = {
	.driver = {
		   .name = "bq24296m",
#ifdef CONFIG_OF
		   .of_match_table = bq24296m_of_match,
#endif
		   },
	.probe = bq24296m_driver_probe,
	.id_table = bq24296m_i2c_id,
};

static int __init bq24296m_init(void)
{
	int ret = 0;

	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	battery_log(BAT_LOG_CRTI, "[bq24296m_init] init start with i2c DTS");
#else
	battery_log(BAT_LOG_CRTI, "[bq24296m_init] init start. ch=%d\n", bq24296m_BUSNUM);
	i2c_register_board_info(bq24296m_BUSNUM, &i2c_bq24296m, 1);
#endif
	if (i2c_add_driver(&bq24296m_driver) != 0) {
		battery_log(BAT_LOG_CRTI,
			    "[bq24296m_init] failed to register bq24296m i2c driver.\n");
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[bq24296m_init] Success to register bq24296m i2c driver.\n");
	}

	/* bq24296m user space access interface */
	ret = platform_device_register(&bq24296m_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq24296m_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&bq24296m_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq24296m_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

static void __exit bq24296m_exit(void)
{
	i2c_del_driver(&bq24296m_driver);
}
module_init(bq24296m_init);
module_exit(bq24296m_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24296m Driver");
MODULE_AUTHOR(" Li Yue <yueli@tcl.com>");
