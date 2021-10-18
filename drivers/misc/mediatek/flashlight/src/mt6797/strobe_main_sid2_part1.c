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

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_flashlight_type.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>

#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define TAG_NAME "[strobe_main_sid2_part1.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define LED1_WARM
// #define LED1_COLD

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */
static struct work_struct workTimeOut;
static int g_timeOutTimeMs; 
extern int g_timeOutTimeMs_reg;//led1 led2共用一个FL_TIM
static u32 strobe_Res;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

extern int flashEnable_SGM3784_2(void);
extern int flashDisable_SGM3784_2(void);
extern int setDuty_SGM3784_2(int duty);
extern int flashEnable_SGM3784_1(void);
extern int flashDisable_SGM3784_1(void);
extern int setDuty_SGM3784_1(int duty);
extern int FlashIc_Enable(void);
extern int FlashIc_Disable(void);
extern int m_duty1;	//add by lijin 2015.5.13
extern int m_duty2;
extern int LED1Closeflag;
extern int LED2Closeflag;
static int FL_Enable(void)
{
#ifdef LED1_WARM
    flashEnable_SGM3784_1();
#else
    flashEnable_SGM3784_2();
#endif
	PK_DBG("FL_Enable-");
	return 0;
}

static int FL_Disable(void)
{
#ifdef LED1_WARM
    flashDisable_SGM3784_1();
#else
    flashDisable_SGM3784_2();
#endif
	return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
#ifdef LED1_WARM
    setDuty_SGM3784_1(duty);
#else
    setDuty_SGM3784_2(duty);
#endif
	return 0;
}

static int FL_Init(void)
{
	PK_DBG(" FL_Init line=%d\n", __LINE__);
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	return 0;
}

static int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

static int FL_hasLowPowerDetect(void)
{

	return 1;
}

static int detLowPowerStart(void)
{
	return 0;
}


static int detLowPowerEnd(void)
{
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;

static int g_b1stInit = 1;

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static void timerInit(void)
{
	/* ktime_t ktime; */


	/* mt6333_set_rg_chrwdt_en(0); */

	/* mt6333_set_rg_chrwdt_td(0); //4 sec */
	/* mt6333_set_rg_chrwdt_en(1); */

	/* mt6333_set_rg_chrwdt_en(0); */

	if (g_b1stInit == 1) {
		g_b1stInit = 0;


		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}



}
#define e_DutyNum 16
extern int isMovieMode[e_DutyNum+1][e_DutyNum+1];
static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	/* kal_uint8 valTemp; */
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	PK_DBG
	    ("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			// if(arg>1600)
				// arg= 1600;//最大支持1.6秒亮灯
			// g_timeOutTimeMs_reg=arg/100-1;//预闪的时间长一点 计算主闪的duty值就会更准确 
			// if(g_timeOutTimeMs_reg>15)
				// g_timeOutTimeMs_reg = 15;
			g_timeOutTimeMs = arg;//用于给torch mode的亮灯计时
			#ifdef LED1_WARM
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS LED1: %ld g_timeOutTimeMs_reg=0x%x \n",arg,g_timeOutTimeMs_reg);
			#else
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS LED2: %ld g_timeOutTimeMs_reg=0x%x \n",arg,g_timeOutTimeMs_reg);
			#endif
		break;


	case FLASH_IOC_SET_DUTY:
		#ifdef LED1_WARM
		PK_DBG("FLASH_IOC_SET_DUTY LED1: %d\n", (int)arg);
		#else
		PK_DBG("FLASH_IOC_SET_DUTY LED2: %d\n", (int)arg);
		#endif
		// if(arg < 0)
			// arg = 0;
		// if(arg>=/* e_DutyNum */16)
			// arg=16-1;
		#ifdef LED1_WARM
		m_duty1 = arg;
		#else
		m_duty2 = arg;
		#endif
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		#ifdef LED1_WARM
		PK_DBG("FLASH_IOC_SET_ONOFF LED1: %d\n", (int)arg);
		#else
		PK_DBG("FLASH_IOC_SET_ONOFF LED2: %d\n", (int)arg);
		#endif
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;
			
				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			#ifdef LED1_WARM
			LED1Closeflag = 0;//亮暖灯
			#else
			LED2Closeflag = 0;
			#endif
			FlashIc_Enable();//flashEnable_SGM3784_2();
			#ifdef LED1_WARM
			FL_dim_duty(m_duty1);
			#else
			FL_dim_duty(m_duty2);
			#endif
			FL_Enable();
			// #ifdef LED1_WARM
			// setDuty_SGM3784_1(m_duty1);
			// flashEnable_SGM3784_1();
			// #else
			// setDuty_SGM3784_2(m_duty2);//调用setDuty_SGM3784_2 函数里面区分是LED1还是LED2
			// flashEnable_SGM3784_2();//调用flashEnable_SGM3784_2 函数里面区分是LED1还是LED2//设置timeout time g_timeOutTimeMs_reg
			// #endif
		}
		else{
			#ifdef LED1_WARM
			LED1Closeflag = 1;//灭暖灯
			#else
			LED2Closeflag = 1;
			#endif
			FlashIc_Enable();
			//FL_dim_duty(m_duty2);
			FL_Disable();
			// #ifdef LED1_WARM
			// flashDisable_SGM3784_1();
			// #else
			// flashDisable_SGM3784_2(); //work_timeOutFunc函数中调用，但是这里注释掉的话，校准的时候不打闪 //disable flash mode  调用flashDisable_SGM3784_2				
			// #endif
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
/*
	case FLASH_IOC_PRE_ON:
		PK_DBG("FLASH_IOC_PRE_ON\n");
			FL_preOn();
		break;
	case FLASH_IOC_GET_PRE_ON_TIME_MS:
		PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
		temp=13;
		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
	    {
		PK_DBG(" ioctl copy to user failed\n");
		return -1;
	    }
		break;
*/
	case FLASH_IOC_SET_REG_ADR:
		PK_DBG("FLASH_IOC_SET_REG_ADR: %d\n", (int)arg);
		/* g_reg = arg; */
		break;
	case FLASH_IOC_SET_REG_VAL:
		PK_DBG("FLASH_IOC_SET_REG_VAL: %d\n", (int)arg);
		/* g_val = arg; */
		break;
	case FLASH_IOC_SET_REG:
		/* PK_DBG("FLASH_IOC_SET_REG: %d %d\n",g_reg, g_val); */

		break;

	case FLASH_IOC_GET_REG:
		PK_DBG("FLASH_IOC_GET_REG: %d\n", (int)arg);

		/* i4RetValue = valTemp; */
		/* PK_DBG("FLASH_IOC_GET_REG: v=%d\n",valTemp); */
		break;

	case FLASH_IOC_HAS_LOW_POWER_DETECT:
		PK_DBG("FLASH_IOC_HAS_LOW_POWER_DETECT");
		temp = FL_hasLowPowerDetect();
		if (copy_to_user((void __user *)arg, (void *)&temp, 4)) {
			PK_DBG(" ioctl copy to user failed\n");
			return -1;
		}
		break;
	case FLASH_IOC_LOW_POWER_DETECT_START:
		detLowPowerStart();
		break;
	case FLASH_IOC_LOW_POWER_DETECT_END:
		i4RetValue = detLowPowerEnd();
		break;

	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;


		spin_unlock_irq(&g_strobeSMPLock);
		FL_Uninit();
	}
	PK_DBG(" Done\n");
	return 0;
}


static FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 strobeInit_main_sid2_part1(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
