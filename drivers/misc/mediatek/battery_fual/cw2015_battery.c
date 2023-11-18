/*
 * CW 2015
 * Reversed by Ruben1863 for:
 * Xiaomi Redmi Note 4X (Nikel)
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>


#include <mt-plat/upmu_common.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include <mt-plat/battery_meter_hal.h>
#include <mt-plat/dev_info.h>

#include <mach/cw2015_battery.h>

#define SIZE_BATINFO    64

#define BATTERY_ID_VOLTAGE		        659999 	// Add for rn4x
#define BATTERY_ID_VOLTAGE_2	        899999 	// Add for rn4x
#define BATTERY_ID_CHANNEL_NUM_PMIC     2 	    // Add for rn4x

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_BATINFO             0x10
#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        //ATHD = 0%

#define CW_I2C_SPEED            100000          // default i2c speed set 100khz
#define BATTERY_UP_MAX_CHANGE   420             // the max time allow battery change quantity
#define BATTERY_DOWN_CHANGE   60                // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE_RUN 30          // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800      // the min time allow battery change quantity when run 30min

#define BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE 1800
#define DEVICE_RUN_TIME_FIX_VALUE 40

#define NO_STANDARD_AC_BIG_CHARGE_MODE 1
// #define SYSTEM_SHUTDOWN_VOLTAGE  3400000        //set system shutdown voltage related in battery info.
#define BAT_LOW_INTERRUPT    1

#define USB_CHARGER_MODE        1
#define AC_CHARGER_MODE         2

#define USE_MTK_INIT_VOL

static struct i2c_client *cw2015_i2c_client; /* global i2c_client to support ioctl */

#define FG_CW2015_DEBUG 1

#define FG_CW2015_TAG                  "[FG_CW2015]"

#ifdef FG_CW2015_DEBUG
#define FG_CW2015_FUN(f)               printk(KERN_ERR FG_CW2015_TAG"%s\n", __FUNCTION__)
#define FG_CW2015_ERR(fmt, args...)    printk(KERN_ERR FG_CW2015_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define FG_CW2015_LOG(fmt, args...)    printk(KERN_ERR FG_CW2015_TAG fmt, ##args)
#endif

#define CW2015_DEV_NAME     "CW2015"

static const struct i2c_device_id FG_CW2015_i2c_id[] = {
	{CW2015_DEV_NAME, 0}, {}
};

MODULE_DEVICE_TABLE(i2c, FG_CW2015_i2c_id);

static const struct of_device_id FG_CW2015_dt_match[] = {
	{.compatible = "mediatek,cw2015"},
	{},
};
MODULE_DEVICE_TABLE(of, FG_CW2015_dt_match);

static struct i2c_board_info __initdata i2c_FG_CW2015 = {
	I2C_BOARD_INFO(CW2015_DEV_NAME, 0x62)
};

int g_cw2015_capacity = 0;
int g_cw2015_vol = 0;
int g_mtk_init_vol = -10;
extern int FG_charging_type;
extern int FG_charging_status;
int CW2015_test_init=0;

extern int Charger_enable_Flag;

#define queue_delayed_work_time  8000

static int file_sys_state = 1;

static u8 config_info_1[SIZE_BATINFO] = {
0x17,0xFD,0x62,0x6B,0x6D,0x6D,0x6A,0x68,0x63,0x63,0x5F,0x5C,0x64,0x52,0x46,0x41,
0x33,0x2E,0x28,0x24,0x24,0x34,0x46,0x4C,0x17,0x65,0x0A,0x3D,0x2B,0x4B,0x4B,0x5B,
0x6D,0x69,0x66,0x67,0x3F,0x1B,0x60,0x66,0x0D,0x2E,0x2C,0x70,0x85,0x98,0x9A,0x29,
0x85,0x8E,0x91,0xAE,0x67,0x89,0xB7,0x70,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x46,0xAE
};

static u8 config_info_2[SIZE_BATINFO] = {
0x17,0xE3,0x81,0x74,0x74,0x6A,0x69,0x65,0x5E,0x6C,0x51,0x58,0x54,0x55,0x48,0x41,
0x31,0x27,0x1E,0x17,0x1D,0x2E,0x46,0x53,0x1D,0x79,0x0A,0x3D,0x04,0x08,0x44,0x57,
0x69,0x65,0x65,0x68,0x3B,0x1A,0x66,0x3F,0x24,0x33,0x23,0x58,0x7C,0x98,0x9B,0x1B,
0x69,0x87,0x95,0xD0,0x4C,0x6E,0x95,0x70,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x46,0xAE
};

static u8 config_info_3[SIZE_BATINFO] = {
0x18,0x00,0x67,0x6D,0x6B,0x6B,0x69,0x64,0x5A,0x77,0x5E,0x4F,0x65,0x56,0x46,0x3D,
0x38,0x2E,0x27,0x24,0x24,0x35,0x43,0x4D,0x1E,0x63,0x0A,0x3D,0x2C,0x4C,0x57,0x69,
0x81,0x7A,0x76,0x7B,0x3E,0x1B,0x69,0x66,0x1A,0x29,0x39,0x56,0x83,0x96,0x97,0x35,
0x69,0x8C,0x91,0xAE,0x6E,0x85,0x9B,0x70,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x2C,0xB1
};

//extern unsigned int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);
int hmi_battery_version = 0;

void hmi_get_battery_version(void) // Edited for rn4x
{
	int id_volt = 0;
	id_volt = PMIC_IMM_GetOneChannelValue(BATTERY_ID_CHANNEL_NUM_PMIC, 5, 0);
	printk("[fgauge_get_profile_id] id_vol id_volt = %d\n", id_volt);
	//battery_id_voltage = id_volt / 1000;
	
	if(id_volt != 0)
	{
		if(id_volt <= BATTERY_ID_VOLTAGE)
			hmi_battery_version = 1;
		else if(id_volt > BATTERY_ID_VOLTAGE_2) 
			hmi_battery_version = 2;
		else
			hmi_battery_version = 3;
	}else{
		hmi_battery_version = 0;
	}
	printk("hmi_battery_version=%d\n", hmi_battery_version);
}

struct cw_bat_platform_data {
        int is_dc_charge;
        int dc_det_pin;
        int dc_det_level;

        int is_usb_charge;
        int chg_mode_sel_pin;
        int chg_mode_sel_level;

        int bat_low_pin;
        int bat_low_level;
        int chg_ok_pin;
        int chg_ok_level;
        u8* cw_bat_config_info;
};

static struct cw_bat_platform_data cw_bat_platdata = {
	.dc_det_pin = 0,
	.dc_det_level = 0,

	.bat_low_pin = 0,
	.bat_low_level = 0,   
	.chg_ok_pin = 0,
	.chg_ok_level = 0,

	.is_usb_charge = 0,
	.chg_mode_sel_pin = 0,
	.chg_mode_sel_level = 0,

	.cw_bat_config_info = config_info_1, // Edited for rn4x, not sure
};

struct cw_battery {
	struct i2c_client *client;
	struct workqueue_struct *battery_workqueue;
	struct delayed_work battery_delay_work;
	struct delayed_work dc_wakeup_work;
	struct delayed_work bat_low_wakeup_work;
	struct cw_bat_platform_data *plat_data;

	struct power_supply rk_bat;
	struct power_supply rk_ac;
	struct power_supply rk_usb;

	long sleep_time_capacity_change;      // the sleep time from capacity change to present, it will set 0 when capacity change 
	long run_time_capacity_change;

	long sleep_time_charge_start;      // the sleep time from insert ac to present, it will set 0 when insert ac
	long run_time_charge_start;

	int dc_online;
	int usb_online;
	int charger_mode;
	int charger_init_mode;
	int capacity;
	int voltage;
	int status;
	int time_to_empty;
	int alt;

	int bat_change;
};

struct cw_battery *CW2015_obj = NULL;

static int cw_read(struct i2c_client *client, u8 reg, u8 buf[])
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client,reg);
	printk("cw_read buf2 = %d",ret);
	if (ret < 0)
	{
	return ret;
	}
	else
	{
		buf[0] = ret;
		ret = 0;
	}

	return ret;
}

static int cw_write(struct i2c_client *client, u8 reg, u8 const buf[])
{
	int ret = 0;

	ret =  i2c_smbus_write_byte_data(client,reg,buf[0]);

	return ret;
}

static int cw_read_word(struct i2c_client *client, u8 reg, u8 buf[])
{
	int ret = 0;
	unsigned int data = 0;

	data = i2c_smbus_read_word_data(client, reg);
	buf[0] = data & 0x00FF;
	buf[1] = (data & 0xFF00)>>8;

	return ret;
}

static int cw_get_vol(struct cw_battery *cw_bat)
{
	int ret;
	u8 reg_val[2];
	u16 value16, value16_1, value16_2, value16_3;
	int voltage;
	FG_CW2015_LOG("cw_get_vol \n");
	
	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
	{
		FG_CW2015_LOG("cw_get_vol 1111\n");
		return ret;
	}
	value16 = (reg_val[0] << 8) + reg_val[1];
	
	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
	{
		FG_CW2015_LOG("cw_get_vol 2222\n");
		return ret;
	}
	value16_1 = (reg_val[0] << 8) + reg_val[1];

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
	{
		FG_CW2015_LOG("cw_get_vol 3333\n");
		return ret;
	}
	value16_2 = (reg_val[0] << 8) + reg_val[1];

	if(value16 > value16_1)
	{	 
		value16_3 = value16;
		value16 = value16_1;
		value16_1 = value16_3;
	}
		
	if(value16_1 > value16_2)
	{
		value16_3 = value16_1;
		value16_1 = value16_2;
		value16_2 = value16_3;
	}
		
	if(value16 > value16_1)
	{	 
		value16_3 = value16;
		value16 = value16_1;
		value16_1 =value16_3;
	}		

	voltage = value16_1 * 312 / 1024;
	voltage = voltage;
	FG_CW2015_LOG("cw_get_vol 4444 voltage = %d\n", voltage);
	return voltage;
}

static int cw_update_config_info(struct cw_battery *cw_bat) // Edited for rn4x
{
	int ret;
	u8 reg_val;
	int i;
	u8 reset_val;

#ifdef FG_CW2015_DEBUG
	FG_CW2015_LOG("func: %s-------\n", __func__);
#endif

	if(hmi_battery_version == 1 || hmi_battery_version == 2)
		FG_CW2015_LOG("test cw_bat_config_info = 0x%x", config_info_2[0]);
	else
		FG_CW2015_LOG("test cw_bat_config_info = 0x%x", config_info_3[0]);

	/* make sure no in sleep mode */
	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
	FG_CW2015_LOG("cw_update_config_info reg_val = 0x%x",reg_val);
	if (ret < 0)
		return ret;

	reset_val = reg_val;
	if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
#ifdef FG_CW2015_DEBUG
		FG_CW2015_ERR("Error, device in sleep mode, cannot update battery info\n");
#endif
		return -1;
	}

	/* update new battery info */
	for (i = 0; i < SIZE_BATINFO; i++) {
		if(hmi_battery_version == 1)
			ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_2[i]);
		else if (hmi_battery_version == 2)
			ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_1[i]);
		else
			ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_3[i]);
		
		if (ret < 0)
			return ret;
	}
	
	/* readback & check */
	for (i = 0; i < SIZE_BATINFO; i++)
	{
		ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
		
		if(hmi_battery_version == 1) {
			if (reg_val != config_info_2[i])
				return -1;
		} else if (hmi_battery_version == 2) {
			if (reg_val != config_info_1[i])
				return -1;
		} else {
			if (reg_val != config_info_3[i])
				return -1;
		}
	}

	/* set cw2015/cw2013 to use new battery info */
	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
	reg_val &= 0x07;                /* clear ATHD */
	reg_val |= ATHD;                /* set ATHD */
	ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	/* check 2015/cw2013 for ATHD & update_flag */ 
	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;
	
	if (!(reg_val & CONFIG_UPDATE_FLG)) {
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("update flag for new battery info have not set..\n");
#endif
	}

	if ((reg_val & 0xf8) != ATHD) {
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("the new ATHD have not set..\n");
#endif
	}
	/* reset */
	reset_val &= ~(MODE_RESTART);
	reg_val = reset_val | MODE_RESTART;
	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0)
		return ret;

	msleep(10);
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	
	msleep(10);
	return 0;
}

static int cw_init(struct cw_battery *cw_bat) // Edited for rn4x
{
	int ret;
	int i;
	struct devinfo_struct *s_DEVINFO_cw2015 = (struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	u8 reg_val = MODE_SLEEP;
	
	hmi_get_battery_version();
	
	s_DEVINFO_cw2015->device_type = "Battery";
	s_DEVINFO_cw2015->device_module = DEVINFO_NULL;
	s_DEVINFO_cw2015->device_ic = DEVINFO_NULL;
	s_DEVINFO_cw2015->device_info = DEVINFO_NULL;
	s_DEVINFO_cw2015->device_version = DEVINFO_NULL;
	s_DEVINFO_cw2015->device_used = "true";
	
	switch (hmi_battery_version) {
		case 1:
		  s_DEVINFO_cw2015->device_vendor = "Sun";
		  break;
		  
		case 2:
		  s_DEVINFO_cw2015->device_vendor = "Des";
		  break;
		  
		case 3:
		  s_DEVINFO_cw2015->device_vendor = "Sun_atl";
		  break;
		  
		default:
		  s_DEVINFO_cw2015->device_vendor = "ERROR";
		  break;
	}
	
	devinfo_check_add_device(s_DEVINFO_cw2015);

	if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) 
	{
		reg_val = MODE_NORMAL;

		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
		if (ret < 0)
			return ret;
				 
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	FG_CW2015_LOG("the new ATHD have not set reg_val = 0x%x\n",reg_val);
	if ((reg_val & 0xf8) != ATHD) 
	{
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("the new ATHD have not set\n");
#endif
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
		FG_CW2015_LOG("cw_init 1111\n");
		if (ret < 0)
			return ret;			 
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		 return ret;
			 
	FG_CW2015_LOG("cw_init REG_CONFIG = %d\n",reg_val);
	
	if (!(reg_val & CONFIG_UPDATE_FLG))
	{
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("update flag for new battery info have not set\n");
#endif

		ret = cw_update_config_info(cw_bat);
		if (ret < 0)
			return ret;
	} else {
		for(i = 0; i < SIZE_BATINFO; i++) 
		{ 
			ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
			if (ret < 0)
				return ret;
				 
			if(hmi_battery_version == 1) {
				if (config_info_2[i] != reg_val)
					break;
				
			} else if(hmi_battery_version == 2) {
				if (config_info_1[i] != reg_val)
					break;
				
			} else {
				if (config_info_3[i] != reg_val)
					break;
				
			}
		}

		if (i != SIZE_BATINFO) {
#ifdef FG_CW2015_DEBUG
			FG_CW2015_LOG("update flag for new battery info have not set\n"); 
#endif
			ret = cw_update_config_info(cw_bat);
			if (ret < 0)
				return ret; 
		}
	}

	for (i = 0; i < 30; i++) 
	{
		ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
		if (ret < 0)
			return ret;
		
		else if (reg_val <= 0x64) 
			break;
		
		msleep(100);
		if (i > 25)
		{
#ifdef FG_CW2015_DEBUG
			FG_CW2015_ERR("cw2015/cw2013 input unvalid power error\n");
#endif
		}
	}
	
	if (i >= 30)
	{
		reg_val = MODE_SLEEP;
		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
#ifdef FG_CW2015_DEBUG
		FG_CW2015_ERR("cw2015/cw2013 input unvalid power error_2\n");
#endif
		return -1;
	}
	
	CW2015_test_init=1;
	
	return 0;
}

static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity;
	int ret;
	u8 reg_val[2];

	struct timespec ts;
	long new_run_time;
	long new_sleep_time;
	long capacity_or_aconline_time;
	int allow_change;
	int allow_capacity;
	static int if_quickstart = 0;
	static int jump_flag = 0;
	static int reset_loop = 0;
	int charge_time;
	u8 reset_val;
	static int count_real_capacity = 0;
	u8 count_real_sum = 0;

	ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
	if (ret < 0)
		return ret;

	FG_CW2015_LOG("cw_get_capacity cw_capacity_0 = %d,cw_capacity_1 = %d\n", reg_val[0], reg_val[1]);
	cw_capacity = reg_val[0];
	if ((cw_capacity < 0) || (cw_capacity > 100)) {
#ifdef FG_CW2015_DEBUG
		FG_CW2015_ERR("get cw_capacity error; cw_capacity = %d\n", cw_capacity);
#endif
		reset_loop++;
		
		if (reset_loop > 5) { 		
			reset_val = MODE_SLEEP;               
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;
			reset_val = MODE_NORMAL;
			msleep(10);
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;
						
			ret = cw_init(cw_bat);
			if (ret) 
				return ret;
			reset_loop = 0;               
		}
				 
		return cw_capacity;
	} else {
		reset_loop = 0;
	}

	if (cw_capacity == 0) 
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("the cw201x capacity is 0 !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
#endif
	else 
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("the cw201x capacity is %d, funciton: %s\n", cw_capacity, __func__);
#endif

	ktime_get_ts(&ts);
	new_run_time = ts.tv_sec;

	get_monotonic_boottime(&ts);
	new_sleep_time = ts.tv_sec - new_run_time;
	FG_CW2015_LOG("cw_get_capacity cw_bat->charger_mode = %d\n",cw_bat->charger_mode);
	//count_time == 20s  

	if(count_real_capacity <= count_real_sum) {
        count_real_capacity++;
	    FG_CW2015_LOG("count_real_capacity = %d\n",cw_bat->charger_mode);
    }

	if (((cw_bat->charger_mode == 0) && (cw_capacity > cw_bat->capacity)&&(cw_capacity < (cw_bat->capacity+20))&&(count_real_capacity>= count_real_sum ) )) {             // modify battery level swing
		if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) 
		{		
			cw_capacity = cw_bat->capacity;
		}
	}

	if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {     // avoid no charge full
		capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
		capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
		allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_UP_MAX_CHANGE;
		if (allow_change > 0) {
			allow_capacity = cw_bat->capacity + allow_change; 
			cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
			jump_flag =1;
		} else if (cw_capacity <= cw_bat->capacity) {
			cw_capacity = cw_bat->capacity; 
		}

	}		 
	else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {     // avoid battery level jump to CW_BAT
		capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
		capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
		allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_DOWN_CHANGE;
		if (allow_change > 0) {
			allow_capacity = cw_bat->capacity - allow_change; 
			if (cw_capacity >= allow_capacity) {
				jump_flag =0;
			}
			else{
				cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
			}
		} else if (cw_capacity <= cw_bat->capacity) {
			cw_capacity = cw_bat->capacity;
		}
	}
	
	if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {              // avoid battery level jump to 0% at a moment from more than 2%
		allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
		allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

		allow_capacity = cw_bat->capacity - allow_change;
		cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("report GGIC POR happened\n");
#endif
		reset_val = MODE_SLEEP;               
		ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
		if (ret < 0)
			return ret;
		reset_val = MODE_NORMAL;
		msleep(10);
		ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
		if (ret < 0)
			return ret;
						
		ret = cw_init(cw_bat);
		if (ret) 
			return ret;	 
	}
	
	if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
	{		  
		charge_time = new_sleep_time + new_run_time - cw_bat->sleep_time_charge_start - cw_bat->run_time_charge_start;
		if ((charge_time > BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE) && (if_quickstart == 0)) {
			reset_val = MODE_SLEEP;               
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;
			reset_val = MODE_NORMAL;
			msleep(10);
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;
						
			ret = cw_init(cw_bat);
			if (ret) 
				return ret;
#ifdef FG_CW2015_DEBUG
			FG_CW2015_LOG("report battery capacity still 0 if in changing\n");
#endif
			if_quickstart = 1;
		}
	} else if ((if_quickstart == 1)&&(cw_bat->charger_mode == 0)) {
		if_quickstart = 0;
	}

#ifdef SYSTEM_SHUTDOWN_VOLTAGE
	if ((cw_bat->charger_mode == 0) && (cw_capacity <= 20) && (cw_bat->voltage <= SYSTEM_SHUTDOWN_VOLTAGE)) {      	     
		if (if_quickstart == 10) {  
		
			allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
			allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

			allow_capacity = cw_bat->capacity - allow_change;
			cw_capacity = (allow_capacity >= 0) ? allow_capacity: 0;
		
			if (cw_capacity < 1){	     	      	
				cw_quickstart(cw_bat);
				if_quickstart = 12;
				cw_capacity = 0;
			}
		} else if (if_quickstart <= 10)
			if_quickstart =if_quickstart + 2;
#ifdef FG_CW2015_DEBUG
		FG_CW2015_LOG("the cw201x voltage is less than SYSTEM_SHUTDOWN_VOLTAGE !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
#endif
	} else if ((cw_bat->charger_mode > 0)&& (if_quickstart <= 12)) {
		if_quickstart = 0;
	}
#endif
	return cw_capacity;
}

static void cw_update_time_member_charge_start(struct cw_battery *cw_bat)
{
	struct timespec ts;
	int new_run_time;
	int new_sleep_time;

	ktime_get_ts(&ts);
	new_run_time = ts.tv_sec;

	get_monotonic_boottime(&ts);
	new_sleep_time = ts.tv_sec - new_run_time;

	cw_bat->run_time_charge_start = new_run_time;
	cw_bat->sleep_time_charge_start = new_sleep_time; 
}

static void cw_update_time_member_capacity_change(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;
    
    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;
    
    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;
    
    cw_bat->run_time_capacity_change = new_run_time;
    cw_bat->sleep_time_capacity_change = new_sleep_time;
}

static int get_usb_charge_state(struct cw_battery *cw_bat)
{
	int usb_status = 0;

	FG_CW2015_LOG("get_usb_charge_state FG_charging_type = %d\n", FG_charging_type);
	if (FG_charging_status == 0)
	{
		usb_status = 0;
		cw_bat->charger_mode = 0;
	}
	else
	{
		if (FG_charging_type == STANDARD_HOST)
		{
			usb_status = 1;
			cw_bat->charger_mode = USB_CHARGER_MODE;
		}
		else
		{
			usb_status = 2;
			cw_bat->charger_mode = AC_CHARGER_MODE;
		}
	}
	FG_CW2015_LOG("get_usb_charge_state usb_status = %d, FG_charging_status = %d\n", usb_status, FG_charging_status);

	return usb_status;
}

static int rk_usb_update_online(struct cw_battery *cw_bat)
{
	int ret = 0;
	int usb_status = 0;

	FG_CW2015_LOG("rk_usb_update_online FG_charging_status = %d\n", FG_charging_status);
	FG_CW2015_LOG("get_usb_update_online FG_charging_type = %d\n", FG_charging_type);
	
	usb_status = get_usb_charge_state(cw_bat);        
	if (usb_status == 2) {
		if (cw_bat->charger_mode != AC_CHARGER_MODE) {
			cw_bat->charger_mode = AC_CHARGER_MODE;
			ret = 1;
		}
		if (cw_bat->usb_online != 1) {
			cw_bat->usb_online = 1;
			cw_update_time_member_charge_start(cw_bat);
		}
	} else if (usb_status == 1) {
		if (cw_bat->charger_mode != USB_CHARGER_MODE) {
			cw_bat->charger_mode = USB_CHARGER_MODE;
			ret = 1;
		}
		if (cw_bat->usb_online != 1) {
			cw_bat->usb_online = 1;
			cw_update_time_member_charge_start(cw_bat);
		}
	} else if (usb_status == 0 && cw_bat->usb_online != 0) {
		cw_bat->charger_mode = 0;
		cw_update_time_member_charge_start(cw_bat);
		cw_bat->usb_online = 0;
		ret = 1;
	}

	return ret;
}

static void rk_bat_update_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity;
    cw_capacity = cw_get_capacity(cw_bat);
    if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
        cw_bat->capacity = cw_capacity;
        cw_bat->bat_change = 1;
        cw_update_time_member_capacity_change(cw_bat);
        
        if (cw_bat->capacity == 0)
        #ifdef FG_CW2015_DEBUG
            FG_CW2015_LOG("report battery capacity 0 and will shutdown if no changing\n");
        #endif
        
    }
    FG_CW2015_LOG("rk_bat_update_capacity cw_capacity = %d\n",cw_bat->capacity);
}

static void rk_bat_update_vol(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_vol(cw_bat);

    if ((ret >= 0) && (cw_bat->voltage != ret)) {
        cw_bat->voltage = ret;
        cw_bat->bat_change = 1;
    }
}

extern int g_platform_boot_mode;
static void cw_bat_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;
	int ret;
	static int count_real_capacity = 0;
	
	FG_CW2015_FUN(); 
	printk("cw_bat_work\n");

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);
	ret = rk_usb_update_online(cw_bat);

	if (cw_bat->usb_online == 1) 
		ret = rk_usb_update_online(cw_bat);

	rk_bat_update_capacity(cw_bat);
	rk_bat_update_vol(cw_bat);
	g_cw2015_capacity = cw_bat->capacity;
	g_cw2015_vol = cw_bat->voltage;
	
	printk("cw_bat_work 777 vol = %d,cap = %d\n",cw_bat->voltage,cw_bat->capacity);
	if (cw_bat->bat_change) {
		cw_bat->bat_change = 0;
	}
	
	if(count_real_capacity < 30 && g_platform_boot_mode == 8) {
		queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(1000));
		count_real_capacity++;
	} else {
		queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
	}
}

/*----------------------------------------------------------------------------*/

static int cw2015_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	FG_CW2015_FUN(); 
	
	strcpy(info->type, CW2015_DEV_NAME);
	return 0;
}

static ssize_t file_state_show(struct device *d, struct device_attribute *a, char *buf)
{
	return sprintf(buf, "%d", file_sys_state);
}

static DEVICE_ATTR(file_state, S_IRUGO, file_state_show, NULL);

static int cw2015_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cw_battery *cw_bat;
	int ret;
	
	FG_CW2015_FUN(); 

	file_sys_state = 1;

	cw_bat = kzalloc(sizeof(struct cw_battery), GFP_KERNEL);
	if (!cw_bat) {
#ifdef FG_CW2015_DEBUG
		FG_CW2015_ERR("fail to allocate memory\n");
#endif
		return -ENOMEM;
	}
		
	i2c_set_clientdata(client, cw_bat);
	cw_bat->plat_data = client->dev.platform_data;
	cw_bat->client = client;
	cw_bat->plat_data = &cw_bat_platdata;
	ret = cw_init(cw_bat);

	if (ret) 
		return ret;
	cw_bat->dc_online = 0;
	cw_bat->usb_online = 0;
	cw_bat->charger_mode = 0;
	cw_bat->capacity = -1;
	cw_bat->voltage = 0;
	cw_bat->status = 0;
	cw_bat->time_to_empty = 0;
	cw_bat->bat_change = 0;

	cw_update_time_member_capacity_change(cw_bat);
	cw_update_time_member_charge_start(cw_bat);

	device_create_file(&client->dev, &dev_attr_file_state);

	cw_bat->battery_workqueue = create_singlethread_workqueue("rk_battery");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10));

#ifdef FG_CW2015_DEBUG
	FG_CW2015_LOG("cw2015/cw2013 driver v1.2 probe sucess\n");
#endif
	return 0;
}

static int cw2015_i2c_remove(struct i2c_client *client)
{
	struct cw_battery *data = i2c_get_clientdata(client);

	FG_CW2015_FUN(); 
	cancel_delayed_work(&data->battery_delay_work);
	cw2015_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(data);

	return 0;
}

static int cw2015_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);
	
	FG_CW2015_FUN(); 
	cancel_delayed_work(&cw_bat->battery_delay_work);

	return 0;
}

static int cw2015_i2c_resume(struct i2c_client *client)
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);
	
	FG_CW2015_FUN(); 
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(100));

	return 0;
}

static struct i2c_driver cw2015_i2c_driver = {	
	.probe      = cw2015_i2c_probe,
	.remove     = cw2015_i2c_remove,
	.detect     = cw2015_i2c_detect,
	.suspend    = cw2015_i2c_suspend,
	.resume     = cw2015_i2c_resume,
	.id_table   = FG_CW2015_i2c_id,
	.driver = {
		.name           = CW2015_DEV_NAME,
        .of_match_table = of_match_ptr(FG_CW2015_dt_match),
	},
};

static int __init cw_bat_init(void)
{
	FG_CW2015_LOG("%s: \n", __func__); 
	printk("cw_bat_init\n");
	i2c_register_board_info(1, &i2c_FG_CW2015, 1);

	if(i2c_add_driver(&cw2015_i2c_driver))
	{
		FG_CW2015_ERR("add driver error\n");
		return -1;
	}
	return 0;
}

static void __exit cw_bat_exit(void)
{
	FG_CW2015_LOG("%s: \n", __func__); 
	printk("cw_bat_exit\n");
	i2c_del_driver(&cw2015_i2c_driver);
}

module_init(cw_bat_init);
module_exit(cw_bat_exit);

MODULE_AUTHOR("ruben1863<rubenes2003@gmail.com>");
MODULE_DESCRIPTION("CW2015 battery driver for rn4x");
MODULE_LICENSE("GPL v2");
