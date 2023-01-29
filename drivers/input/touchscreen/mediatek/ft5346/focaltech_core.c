/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
//#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>

#include "include/tpd_ft5x0x_common.h"
/*tuwenzan@wind-mobi.com 20160722 start ***/
#include <linux/fs.h>
/*tuwenzan@wind-mobi.com 20160722 end ***/
#include "focaltech_core.h"
/* #include "ft5x06_ex_fun.h" */

#include "tpd.h"
#include "base.h"
/* #define TIMER_DEBUG */


#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif
//tuwenzan@wind-mobi.com modify at 20160608 begin
#if CTP_ESD_PROTECT
//define and implement in focaltech_esd_protection.c
extern int  fts_esd_protection_init(void);
extern int  fts_esd_protection_exit(void);
extern int  fts_esd_protection_notice(void);
extern int  fts_esd_protection_suspend(void);
extern int  fts_esd_protection_resume(void);

int apk_debug_flag = 0;
//int  power_switch_gesture = 0;
//#define TPD_ESD_CHECK_CIRCLE        		200
//static struct delayed_work ctp_esd_check_work;
//static struct workqueue_struct *ctp_esd_check_workqueue = NULL;
void ctp_esd_check_func(void);
static int count_irq = 0;
#endif
//tuwenzan@wind-mobi.com modify at 20160608 end


#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
enum DOZE_T {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static DOZE_T doze_status = DOZE_DISABLED;
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client);

enum TOUCH_IPI_CMD_T {
	/* SCP->AP */
	IPI_COMMAND_SA_GESTURE_TYPE,
	/* AP->SCP */
	IPI_COMMAND_AS_CUST_PARAMETER,
	IPI_COMMAND_AS_ENTER_DOZEMODE,
	IPI_COMMAND_AS_ENABLE_GESTURE,
	IPI_COMMAND_AS_GESTURE_SWITCH,
};

struct Touch_Cust_Setting {
	u32 i2c_num;
	u32 int_num;
	u32 io_int;
	u32 io_rst;
};

struct Touch_IPI_Packet {
	u32 cmd;
	union {
		u32 data;
		Touch_Cust_Setting tcs;
	} param;
};

/* static bool tpd_scp_doze_en = FALSE; */
static bool tpd_scp_doze_en = TRUE;
DEFINE_MUTEX(i2c_access);
#endif

#define TPD_SUPPORT_POINTS	5


struct i2c_client *i2c_client = NULL;
struct task_struct *thread_tpd = NULL;
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client 				= NULL;
struct input_dev *fts_input_dev				=NULL;
#ifdef TPD_AUTO_UPGRADE
static bool is_update = false;
#endif
#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;
#endif


//static struct kobject *touchscreen_dir=NULL;
//static struct kobject *virtual_dir=NULL;
//static struct kobject *touchscreen_dev_dir=NULL;
//static char *vendor_name=NULL;
//static u8 ctp_fw_version;
//static int tpd_keys[TPD_VIRTUAL_KEY_MAX] = { 0 };
//static int tpd_keys_dim[TPD_VIRTUAL_KEY_MAX][4]={0};

#define WRITE_BUF_SIZE  1016
#define PROC_UPGRADE							0
#define PROC_READ_REGISTER						1
#define PROC_WRITE_REGISTER					    2
#define PROC_AUTOCLB							4
#define PROC_UPGRADE_INFO						5
#define PROC_WRITE_DATA						    6
#define PROC_READ_DATA							7
#define PROC_SET_TEST_FLAG						8
//static unsigned char proc_operate_mode 			= PROC_UPGRADE;



static DECLARE_WAIT_QUEUE_HEAD(waiter);

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);


static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static void fts_release_all_finger(void);
static int tpd_flag;
/*static int point_num = 0;
static int p_point_num = 0;*/

unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
unsigned int touch_irq = 0;
#define TPD_OK 0


/* Register define */
#define DEVICE_MODE	0x00
#define GEST_ID		0x01
#define TD_STATUS	0x02

#define TOUCH1_XH	0x03
#define TOUCH1_XL	0x04
#define TOUCH1_YH	0x05
#define TOUCH1_YL	0x06

#define TOUCH2_XH	0x09
#define TOUCH2_XL	0x0A
#define TOUCH2_YH	0x0B
#define TOUCH2_YL	0x0C

#define TOUCH3_XH	0x0F
#define TOUCH3_XL	0x10
#define TOUCH3_YH	0x11
#define TOUCH3_YL	0x12

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3

#ifdef TIMER_DEBUG

static struct timer_list test_timer;

static void timer_func(unsigned long data)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	mod_timer(&test_timer, jiffies + 100*(1000/HZ));
}

static int init_test_timer(void)
{
	memset((void *)&test_timer, 0, sizeof(test_timer));
	test_timer.expires  = jiffies + 100*(1000/HZ);
	test_timer.function = timer_func;
	test_timer.data     = 0;
	init_timer(&test_timer);
	add_timer(&test_timer);
	return 0;
}
#endif


#if defined(CONFIG_TPD_ROTATE_90) || defined(CONFIG_TPD_ROTATE_270) || defined(CONFIG_TPD_ROTATE_180)
/*
static void tpd_swap_xy(int *x, int *y)
{
	int temp = 0;

	temp = *x;
	*x = *y;
	*y = temp;
}
*/
/*
static void tpd_rotate_90(int *x, int *y)
{
//	int temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
static void tpd_rotate_180(int *x, int *y)
{
	*y = TPD_RES_Y + 1 - *y;
	*x = TPD_RES_X + 1 - *x;
}
/*
static void tpd_rotate_270(int *x, int *y)
{
//	int temp;

	*y = TPD_RES_Y + 1 - *y;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
#endif
struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__
#ifdef __MSG_DMA_MODE__
	u8 *g_dma_buff_va = NULL;
	dma_addr_t g_dma_buff_pa = 0;
#endif

#ifdef __MSG_DMA_MODE__

	static void msg_dma_alloct(void)
	{
	    if (NULL == g_dma_buff_va)
    		{
       		 tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
       		 g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
    		}

	    	if(!g_dma_buff_va)
		{
	        	TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	    	}
	}

#if 0
	static void msg_dma_release(void){
		if(g_dma_buff_va)
		{
	     		dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
	        	g_dma_buff_va = NULL;
	        	g_dma_buff_pa = 0;
			TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	    	}
	}
#endif
#endif

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
/* static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX; */
/* static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX; */
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id ft5x0x_tpd_id[] = {{"ft5x0x", 0}, {} };
static const struct of_device_id ft5x0x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft5x0x_dt_match),
		.name = "ft5x0x",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5x0x_tpd_id,
	.detect = tpd_i2c_detect,
};

static int of_get_ft5x0x_platform_data(struct device *dev)
{
	/*int ret, num;*/

	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ft5x0x_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}
//	tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "rst-gpio", 0);
//	tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "int-gpio", 0);
	/*ret = of_property_read_u32(dev->of_node, "rst-gpio", &num);
	if (!ret)
		tpd_rst_gpio_number = num;
	ret = of_property_read_u32(dev->of_node, "int-gpio", &num);
	if (!ret)
		tpd_int_gpio_number = num;
  */
	TPD_DMESG("g_vproc_en_gpio_number %d\n", tpd_rst_gpio_number);
	TPD_DMESG("g_vproc_vsel_gpio_number %d\n", tpd_int_gpio_number);
	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static ssize_t show_scp_ctrl(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t store_scp_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u32 cmd;
	Touch_IPI_Packet ipi_pkt;

	if (kstrtoul(buf, 10, &cmd)) {
		TPD_DEBUG("[SCP_CTRL]: Invalid values\n");
		return -EINVAL;
	}

	TPD_DEBUG("SCP_CTRL: Command=%d", cmd);
	switch (cmd) {
	case 1:
	    /* make touch in doze mode */
	    tpd_scp_wakeup_enable(TRUE);
	    tpd_suspend(NULL);
	    break;
	case 2:
	    tpd_resume(NULL);
	    break;
		/*case 3:
	    // emulate in-pocket on
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 1;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;
	case 4:
	    // emulate in-pocket off
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 0;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;*/
	case 5:
		{
				Touch_IPI_Packet ipi_pkt;

				ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
			    ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
			ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
				ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
			ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;
			if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0) < 0)
				TPD_DEBUG("[TOUCH] IPI cmd failed (%d)\n", ipi_pkt.cmd);

			break;
		}
	default:
	    TPD_DEBUG("[SCP_CTRL] Unknown command");
	    break;
	}

	return size;
}
static DEVICE_ATTR(tpd_scp_ctrl, 0664, show_scp_ctrl, store_scp_ctrl);
#endif

static struct device_attribute *ft5x0x_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif
};

// rn4x
/*****************************************************************************
*  Name: fts_release_all_finger
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_release_all_finger(void)
{
	unsigned int finger_count = 0;

	for (finger_count = 0; finger_count < CFG_MAX_TOUCH_POINTS; finger_count++) {
		input_mt_slot(tpd->dev, finger_count);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
	}
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_sync(tpd->dev);
}

/************************************************************************
* Name: fts_read_touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
static int fts_read_touchdata(struct ts_event *data)
{
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_ID;

	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		printk("[B]Read touchdata failed, ret: %d", ret);
		return ret;
	}
	ret = data->touchs;
	memset(data, 0, sizeof(struct ts_event));
	data->touchs = ret;
	data->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;
	data->touch_point = 0;

	for (i = 0; i < tpd_dts_data.touch_max_num; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			data->touch_point++;
		data->au16_x[i] =
		    (s16) (buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F)
		    << 8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
		data->au16_y[i] =
		    (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F)
		    << 8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
		data->au8_touch_event[i] =
		    buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
		data->au8_finger_id[i] =
		    (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;

		data->pressure[i] = (buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);	/* cannot constant value */
		data->area[i] = (buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;
		if ((data->au8_touch_event[i] == 0
		     || data->au8_touch_event[i] == 2)
		    && (data->touch_point_num == 0))
			break;

	}
	return 0;
}

/************************************************************************
* Name: fts_report_key
* Brief: report key event
* Input: event info
* Output: no
* Return: 0: is key event, -1: isn't key event
***********************************************************************/
static int fts_report_key(struct ts_event *data)
{
	int i = 0;

	if (1 != data->touch_point)
		return -1;

	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		if (data->au16_y[i] <= TPD_RES_Y) {
			return -1;
		}
	}

	if(i >= CFG_MAX_TOUCH_POINTS)
		return -1;
	
	if (data->au8_touch_event[i] == 0
		|| data->au8_touch_event[i] == 2) {
		if (data->au8_touch_event[i] == 0) {
			printk("[B]Key(%d, %d) DOWN!", data->au16_x[0],
				 data->au16_y[0]);
		}
		tpd_button(data->au16_x[0], data->au16_y[0], 1);
		printk("[B]Key(%d, %d) DOWN!", data->au16_x[0],
			  data->au16_y[0]);
	} else {
		tpd_button(data->au16_x[0], data->au16_y[0], 0);
		printk("[B]Key(%d, %d) UP!", data->au16_x[0],
			  data->au16_y[0]);
		printk("[B]Key(%d, %d) UP!", data->au16_x[0],
			  data->au16_y[0]);
	}

	input_sync(tpd->dev);

	return 0;
}

/************************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
#define FTS_FORCE_TOUCH_EN                      0
#define FTS_REPORT_PRESSURE_EN                  1

static int fts_report_value(struct ts_event *data)
{
	int i = 0;
	int up_point = 0;
	int touchs = 0;
	int pressure = 0;
	bool reported = false;

	for (i = 0; i < data->touch_point; i++) {
		input_mt_slot(tpd->dev, data->au8_finger_id[i]);

		reported = true;
		if (data->au8_touch_event[i] == 0
		    || data->au8_touch_event[i] == 2) {
			if (data->au8_touch_event[i] == 0) {
				printk("[B]P%d(%4d,%4d)[tm:%d] DOWN!",
				data->au8_finger_id[i],
				data->au16_x[i], data->au16_y[i],
				data->area[i]);
			}
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,
						   true);
			input_report_key(tpd->dev, BTN_TOUCH, 1);
			if (FTS_REPORT_PRESSURE_EN) {
				if (FTS_FORCE_TOUCH_EN) {
					if (data->pressure[i] > 0) {
						pressure = data->pressure[i];
					} else {
						printk("[B]Illegal pressure: %d", data->pressure[i]);
						pressure = 1;
					}
				} else {
					pressure = 0x3f;
				}
				input_report_abs(tpd->dev, ABS_MT_PRESSURE,
						 pressure);
			}
			if (data->area[i] > 0) {
				input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, data->area[i]);	/* 0x05 */
			} else {
				printk("[B]Illegal touch-major: %d", data->area[i]);
				input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);	/* 0x05 */
			}
			input_report_abs(tpd->dev, ABS_MT_POSITION_X,
					 data->au16_x[i]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y,
					 data->au16_y[i]);
			touchs |= BIT(data->au8_finger_id[i]);
			data->touchs |= BIT(data->au8_finger_id[i]);

			if (FTS_REPORT_PRESSURE_EN) {
				printk("[B]P%d(%4d,%4d)[p:%d,tm:%d] DOWN!",
					  data->au8_finger_id[i],
					  data->au16_x[i], data->au16_y[i],
					  pressure, data->area[i]);
			} else {
				printk("[B]P%d(%4d,%4d)[tm:%d] DOWN!",
					  data->au8_finger_id[i],
					  data->au16_x[i], data->au16_y[i],
					  data->area[i]);
			}
		} else {
			up_point++;
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,
						   false);
			if (FTS_REPORT_PRESSURE_EN) {
				input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
			}
			data->touchs &= ~BIT(data->au8_finger_id[i]);
			printk("[B]P%d UP!", data->au8_finger_id[i]);
			printk("[B]P%d UP!", data->au8_finger_id[i]);
		}

	}
	for (i = 0; i < tpd_dts_data.touch_max_num; i++) {
		if (BIT(i) & (data->touchs ^ touchs)) {
			printk("[B]P%d UP!", i);
			reported = true;
			printk("[B]P%d UP!", i);
			data->touchs &= ~BIT(i);
			input_mt_slot(tpd->dev, i);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,
						   false);
			if (FTS_REPORT_PRESSURE_EN) {
				input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
			}
		}
	}
	data->touchs = touchs;

	/* if (data->touch_point_num == 0) {
		for (i = 0; i < FTS_MAX_POINTS; i++) {
			input_mt_slot(tpd->dev, i);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
		}
		data->touchs = 0;
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_sync(tpd->dev);
		return 0;
	} */

	if (reported) {
		if ((data->touch_point == up_point) || !data->touch_point_num) {
			printk("[B]Points All UP!");
			input_report_key(tpd->dev, BTN_TOUCH, 0);
		} else {
			input_report_key(tpd->dev, BTN_TOUCH, 1);
		}

		input_sync(tpd->dev);
	}
	return 0;
}

int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
    int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			pr_err("%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("%s:i2c read error.\n", __func__);
	}
	return ret;
}
// end rn4x

/************************************************************************
* Name: fts_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error.\n", __func__);

	return ret;
}

/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(client, buf, sizeof(buf));
}

/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{

	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);

}

// rn4x
int FG_charging_status = 0;
int charging_flag = 0;

static int touch_event_handler(void *unused)
{
	int i = 0;
	int ret = 0;
	u8 state = 0;
	struct ts_event pevent;

	struct touch_info finfo;
	struct sched_param param = { .sched_priority = 4 };
	
	if (tpd_dts_data.use_tpd_button) {
		memset(&finfo, 0, sizeof(struct touch_info));
		for (i = 0; i < TPD_SUPPORT_POINTS; i++)
			finfo.p[i] = 1;
	}

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		/*enable_irq(touch_irq);*/
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

		if((FG_charging_status != 0) && (charging_flag == 0))
		{
			charging_flag = 1;
            fts_write_reg(i2c_client, 0x8B, 2);
		}
		else
		{
			if((FG_charging_status  == 0) && (charging_flag == 1))
			{
				charging_flag = 0;
                fts_write_reg(i2c_client, 0x8B, 2);
			}
		}


		ret = fts_read_reg(fts_i2c_client, 0xD0, &state);
		if (ret < 0)
		{
			printk("tpd,[Focal][Touch] read value fail");
		}

		if(state == 1)
		{
			fts_read_Gestruedata();
			continue;
		}
		
		ret = fts_read_touchdata(&pevent);
		if (ret == 0) {
			if (tpd_dts_data.use_tpd_button)
				ret = !fts_report_key(&pevent);
			
			if (ret == 0)
				fts_report_value(&pevent);
		}
		
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}
// end rn4x

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	//tuwenzan@wind-mobi.com modify at 20160602 begin
	#if CTP_ESD_PROTECT
	count_irq ++;
    #endif
	//tuwenzan@wind-mobi.com modify at 20160602 end
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

// rn4x
static int tpd_irq_registration(void)
{  
 	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if (ret > 0)
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	
	return 0;
}

#include <dev_info.h>
static char *Version;
static void devinfo_ctp_regchar(char *module, char * vendor, char *version, char *ic, char *used)
{
 	struct devinfo_struct *s_DEVINFO_ctp =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	s_DEVINFO_ctp->device_type="CTP";
	s_DEVINFO_ctp->device_module=module;
	s_DEVINFO_ctp->device_vendor=vendor;
	s_DEVINFO_ctp->device_ic=ic;
	s_DEVINFO_ctp->device_info=DEVINFO_NULL;
	s_DEVINFO_ctp->device_version=version;
	s_DEVINFO_ctp->device_used=used;
	printk("[DEVINFO CTP]registe CTP device! type:<%s> module:<%s> vendor<%s> ic<%s> version<%s> info<%s> used<%s>\n", "CTP", module, vendor, ic, version, "(null)", used);
	devinfo_check_add_device(s_DEVINFO_ctp);
}

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

static struct proc_dir_entry *ctp_status_proc = NULL;
char tp_version;
char vendor_id = 0;
static struct proc_dir_entry *ctp_lockdown_status_proc = NULL;
char tp_lockdown_info[128];

static int ctp_proc_read_show(struct seq_file *m, void *data)
{
	char temp[40] = {0};
	memset(temp, 0, sizeof(temp));
	
	if (vendor_id == 0x3B)
		sprintf(temp, "[Vendor]BOEN, [fw]T%x.%x, [ic]%s\n", tp_version >> 4, tp_version & 0xF, "ft5346");
	else
		sprintf(temp, "[Vendor]O-FLIM, [fw]T%x.%x, [ic]%s\n", tp_version >> 4, tp_version & 0xF, "ft5346");
	  
	seq_printf(m, "%s\n", temp);

	return 0;
}

static int ctp_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ctp_proc_read_show, inode->i_private);
}

static const struct file_operations g_ctp_proc = {

	.open = ctp_proc_open,
	.read = seq_read,
};

static int ctp_lockdown_proc_show(struct seq_file *file, void*data)
{
	char temp[40] = {0};

	sprintf(temp, "%s\n", tp_lockdown_info);
	seq_printf(file, "%s\n", temp);

	return 0;
}

static int ctp_lockdown_proc_open (struct inode*inode, struct file*file)
{
	return single_open(file, ctp_lockdown_proc_show, inode->i_private);
}

static const struct file_operations ctp_lockdown_proc_fops = {
	.open = ctp_lockdown_proc_open,
	.read = seq_read,
};

extern void tpd_gpio_enable_regulator_output(int en);
extern unsigned char ft5x46_ctpm_LockDownInfo_get_from_boot(struct i2c_client *client, char *pProjectCode);
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 report_rate = 0;
	int reset_count = 0;
	int retval = TPD_OK;
	int err = 0;
	char data;
	
	printk("enter tpd_probe\n");
  
	i2c_client = client;
	fts_i2c_client = client;
	fts_input_dev = tpd->dev;

	if(client->addr != 0x38)
		client->addr = 0x38;
  
	of_get_ft5x0x_platform_data(&client->dev);
	tpd_gpio_enable_regulator_output(1);
	
reset_proc:

    tpd_gpio_output(tpd_rst_gpio_number, 0);
    msleep(20);
    tpd_gpio_output(tpd_rst_gpio_number, 1);
    msleep(400);
    err = fts_read_reg(i2c_client, 0x00, &data);
    printk("tpd fts_i2c:err %d,data:%d\n", err, data);
	
	if(err < 0 || data != 0)
	{
		printk("I2C transfer error, line: %d\n", __LINE__);
		if (++reset_count < TPD_MAX_RESET_COUNT)
		{
			goto reset_proc;
		}
		gpio_free(tpd_rst_gpio_number);
		return -1;
	}
	
	tpd_gpio_as_int(1);
	tpd_irq_registration();
	
	msleep(100);
	
	msg_dma_alloct();

	fts_Gesture_init(tpd->dev);
	input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS, 2);
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	
	tpd_load_status = 1;
	
	//MEMORY[0xFFFFFFC0018C3C38] = 1; // Unknown for now
  
	fts_create_sysfs(fts_i2c_client);
	fts_get_upgrade_array();
	fts_create_apk_debug_channel(fts_i2c_client);
	printk("tpd, enter CONFIG_AUTO_UPGRADE_SUPPORT\n");
	
	is_update = true;
	fts_ctpm_auto_upgrade(fts_i2c_client);
	is_update = false;
	
	report_rate = 0x8;
	if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0) {
		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}
	
  	thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd)) {
		retval = PTR_ERR(thread_tpd);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd: %d\n", retval);
	} else {
		wake_up_process(thread_tpd);
	}

	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

	ft5x46_ctpm_LockDownInfo_get_from_boot(i2c_client, tp_lockdown_info);
	printk("tpd_probe, ft5x46_ctpm_LockDownInfo_get_from_boot, tp_lockdown_info=%s\n", tp_lockdown_info);

	fts_read_reg(i2c_client, 0xa6, &tp_version);
	fts_read_reg(i2c_client, 0xa8, &vendor_id);
	
	printk("tpd_probe, fts_read_reg(i2c_client, 0xa6 =0x%x)\n", tp_version);

	ctp_status_proc = proc_create_data("tp_info", 0644, NULL, &g_ctp_proc, NULL);
	if (ctp_status_proc == NULL) {
		printk("tpd, create_proc_entry ctp_status_proc failed\n");
	}

	ctp_lockdown_status_proc = proc_create_data("tp_lockdown_info", 0644, NULL, &ctp_lockdown_proc_fops, NULL);
	if (ctp_lockdown_status_proc == NULL) {
	    printk("tpd, create_proc_entry ctp_lockdown_status_proc failed\n");
	}

	Version = (char *)kmalloc(10, GFP_KERNEL);
	memset(Version, 0, sizeof(char) * 10);

	sprintf(Version, "T%x.%x", tp_version >> 4, tp_version & 0xF);
  
  	if(vendor_id == 0x3B) {
		devinfo_ctp_regchar("BoEn", "BoEn", Version, "Ft5346", DEVINFO_USED);
		devinfo_ctp_regchar("o-film", "o-film", DEVINFO_NULL, "Ft5346", DEVINFO_UNUSED);
	} else {
		devinfo_ctp_regchar("o-film", "o-film", Version, "Ft5346", "true");
		devinfo_ctp_regchar("BoEn", "BoEn", DEVINFO_NULL, "Ft5346", DEVINFO_UNUSED);
	}
	devinfo_ctp_regchar("Mutton", "Mutton", DEVINFO_NULL, "GT915L", DEVINFO_UNUSED);
	devinfo_ctp_regchar("Mutton", "Mutton", DEVINFO_NULL, "MXT308U", DEVINFO_UNUSED);
	
	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");

	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

	return 0;
}
// end rn4x

//tuwenzan@wind-mobi.com modify at 20160608 begin
#if CTP_ESD_PROTECT
 /************************************************************************
 * Name: force_reset_guitar
 * Brief: reset
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
 static void force_reset_guitar(void)
 {
 //tuwenzan@wind-mobi.com modify at 20150607 begin
	 int retval;
 	/* Reset CTP */
//	printk("twz enter reset guitar\n");
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	 msleep(10);
	 retval = regulator_disable(tpd->reg);
	 if (retval != 0)
		 TPD_DMESG("Failed to disable reg-vgp6: %d\n", retval);
	 msleep(200);
	 retval = regulator_enable(tpd->reg);
	 if (retval != 0)
		 TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	 msleep(10);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	 tpd_gpio_as_int(tpd_int_gpio_number);
	 msleep(300);
 //tuwenzan@wind-mobi.com modify at 20150607 end
 }


#define A3_REG_VALUE					0X54 //0x87 12 tuwenzan@wind-mobi modify this value at 20161028
//#define RESET_91_REGVALUE_SAMECOUNT 	5
//tuwenzan@wind-mobi.com modify at 20160608 end
//tuwenzan@wind-mobi.com modify at 20160608 begin
void ctp_esd_check_func(void)
{
		int i;
		int ret = -1;
	 u8 data;
		int reset_flag = 0;

//	 printk("twz enter ctp_esd_check_func for protect tp\n");
		for (i = 0; i < 3; i++)
	 {
		
			ret =  fts_read_reg(i2c_client, 0xA3, &data);
//			printk("focal--fts_esd_check_func-0xA3:%x\n", data);
			if (ret==1 && A3_REG_VALUE==data) {
				break;
			}
	 }
	
		if (i >= 3) {
		 force_reset_guitar();
			printk("focal--tpd reset. i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
			reset_flag = 1;
			goto FOCAL_RESET_A3_REGISTER;
	 }
		
		
FOCAL_RESET_A3_REGISTER:
		data=0;
		// ret = fts_write_reg(i2c_client, 0x8F,data);
	
	 return;
 }
#endif
//tuwenzan@wind-mobi.com modify at 20160608 end

// rn4x
static int tpd_local_init(void)
{
    int retval;

    printk("Focaltech ft53d46 I2C Touchscreen Driver...\n");
    retval = i2c_add_driver(&tpd_i2c_driver);
    if (retval)
    {
		TPD_DMESG("Unable to add i2c driver.\n");
        return -1;
    } else {
        if (tpd_dts_data.use_tpd_button)
            tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local, tpd_dts_data.tpd_key_dim_local);
    }
    tpd_type_cap = 1; // MEMORY[0xFFFFFFC0018C3890] = 1;
    return 0;
}
// end rn4x

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	char gestrue_on = 0x01;
	char gestrue_data;
	int i;

	/* TPD_DEBUG("Entering doze mode..."); */
	pr_alert("Entering doze mode...");

	/* Enter gestrue recognition mode */
	ret = fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);
	if (ret < 0) {
		/* TPD_DEBUG("Failed to enter Doze %d", retry); */
		pr_alert("Failed to enter Doze %d", retry);
		return ret;
	}
	msleep(30);

	for (i = 0; i < 10; i++) {
		fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
		if (gestrue_data == 0x01) {
			doze_status = DOZE_ENABLED;
			/* TPD_DEBUG("FTP has been working in doze mode!"); */
			pr_alert("FTP has been working in doze mode!");
			break;
		}
		msleep(20);
		fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);

	}

	return ret;
}
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
void tpd_scp_wakeup_enable(bool en)
{
	tpd_scp_doze_en = en;
}

void tpd_enter_doze(void)
{

}
#endif

// rn4x
static void tfs_tp_set_cover_mode(void) {
// TODO

/*
  if ( MEMORY[0xFFFFFFC0018C022C] == 1 )
  {
    HIBYTE(v1) = MEMORY[0xFFFFFFC0018C022C];
    LOBYTE(v1) = -63;
  }
  else
  {
    v1 = 193;
  }
  return fts_write_reg(MEMORY[0xFFFFFFC0018C3C08], (__int64)&v1, 2);
*/
}

static void tpd_resume(struct device *h)
{
    printk("tpd_resume enter\n");
	tpd_gpio_enable_regulator_output(1);
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(200);

	fts_write_reg(fts_i2c_client, 0xD0, 2);
	fts_release_all_finger();
	enable_irq(touch_irq);
	FG_charging_status = charging_flag == 0;
	tfs_tp_set_cover_mode();
}

static void tpd_suspend(struct device *h)
{
    printk("tpd_suspend enter\n");
	fts_write_reg(i2c_client, 0xD0, 2);
	fts_write_reg(i2c_client, 0xD0, 2);
	fts_write_reg(i2c_client, 0xD1, 2);
	fts_write_reg(i2c_client, 0xD2, 2);
	fts_write_reg(i2c_client, 0xD5, 2);
	fts_write_reg(i2c_client, 0xD6, 2);
	fts_write_reg(i2c_client, 0xD7, 2);
	fts_write_reg(i2c_client, 0xD8, 2);
	  
	msleep(10);

	disable_irq(touch_irq);
	fts_write_reg(i2c_client, 0xA5, 2);
	tpd_gpio_enable_regulator_output(0);
	fts_release_all_finger();
}
// end rn4x

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT5x0x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft5x0x_attrs,
		.num  = ARRAY_SIZE(ft5x0x_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5x0x driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
