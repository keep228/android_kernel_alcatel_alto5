////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2012 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
struct notifier_block TPfb_notifier;
#endif
#include <linux/regulator/consumer.h>

/* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
bool tp_suspended =false;

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_GestureWakeupMode;
extern u8 g_GestureWakeupFlag;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

extern struct input_dev *g_InputDevice;

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/
//zxz
#if 1
//void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
void MsDrvInterfaceTouchDeviceSuspend(void)
{
    DBG("*** %s() ***\n", __func__);
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
 if( _gIsUpdateFirmware != 0x00)
	return;
#endif
/* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
	if (tp_suspended) {
		DBG("****Already in suspend state*****\n");
		return;
	}

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
//    g_GestureWakeupMode = 0x1FFF; // Enable all gesture wakeup mode for testing 

    if (g_GestureWakeupMode != 0x0000)
    {
        DrvIcFwLyrOpenGestureWakeup(g_GestureWakeupMode);
	 tp_suspended=true;
        return;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
    input_sync(g_InputDevice);

    DrvPlatformLyrDisableFingerTouchReport();
    DrvPlatformLyrTouchDevicePowerOff(); 
    DrvPlatformLyrTouchDeviceVoltageControl(false);//zxzadd
/* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
tp_suspended=true;

    }

void MsDrvInterfaceTouchDeviceResume(void)
{
    DBG("*** %s() ***\n", __func__);

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
 if( _gIsUpdateFirmware != 0x00)
	return;
 #endif
/* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
	if (!tp_suspended) {
		DBG("****Already in awake state*****\n");
		return;
	}

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (g_GestureWakeupFlag == 1)
    {
        DrvIcFwLyrCloseGestureWakeup();
    }
    else
    {
        DrvPlatformLyrEnableFingerTouchReport(); 
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    DrvPlatformLyrTouchDeviceVoltageControl(true);//zxzadd

    DrvPlatformLyrTouchDevicePowerOn();
    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
//    DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
    DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
/* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
tp_suspended=false;

}
#endif


//zxzadd
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			MsDrvInterfaceTouchDeviceResume();
		else if (*blank == FB_BLANK_POWERDOWN)
			MsDrvInterfaceTouchDeviceSuspend();
	}

	return 0;
}
#endif

/* [PLATFORM]-Add-BEGIN by TCTNB.ZXZ, FR-810275, 2014/10/14, Add I2C test before add driver */
static int MsDrvI2cTest(struct i2c_client *client)
{
	int rc;
	char temp;
	struct i2c_msg msgs[] =
	{
		{
		.addr = SLAVE_I2C_ID_DBBUS,//client->addr,
		.flags = 0,//I2C_M_RD,
		.len = 1,
		.buf = &temp,
		},
	};
	printk("==============Mstar I2C test --client addr : %x\n", client->addr);

	rc = i2c_transfer(client->adapter, msgs, 1);
	DBG("Mstar I2C test-- i2c transfer rc = %d\n", rc);
	if( rc < 0 )
	{
		printk("==============Mstar I2C test fail !=========\n");
		return rc;
	}

	msleep(10);
	printk("==============Mstar I2C test OK !=========\n");
	return 0;
}
/* [PLATFORM]-Add-END by TCTNB.ZXZ */

/* [PLATFORM]-ADD-BEGIN by TCTNB.Yubin, PR963868, 2015/04/04, add i2c scan function */
#ifdef CONFIG_TCT_8X16_TAPTI
static int i2c_ok = 0;
module_param_named(i2c_ok, i2c_ok, int, 0644);
#endif
/* [PLATFORM]-ADD-END by TCTNB.Yubin */

/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;
    int err = 0;
    int rc=0;
    DBG("*** %s() ***\n", __func__);

    DrvPlatformLyrInputDeviceInitialize(pClient);
	
    DrvPlatformLyrTouchDeviceVoltageInit(pClient,true);//zxzadd
    DrvPlatformLyrTouchDeviceVoltageControl(true);//zxzadd

    DrvPlatformLyrTouchDeviceRequestGPIO();

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    DrvPlatformLyrTouchDeviceRegulatorPowerOn();
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    DrvPlatformLyrTouchDevicePowerOn();
	//zxz add for release resoutce when isn't mstar TP
#if 1
    err=MsDrvI2cTest(pClient);

    if(err<0)
	{
		rc = regulator_disable(TPvdd);
	if (rc) {
		pr_err("Regulator vdd disable failed rc=%d\n", rc);
	}

	rc = regulator_disable(TPvcc_i2c);
	if (rc) {
		pr_err("Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(TPvdd);
		if (rc) {
			pr_err("Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	if (gpio_is_valid(global_reset_gpio))
		{
		pr_err("=========free=global_reset_gpio==========\n");
		gpio_free(global_reset_gpio);
		}
	if (gpio_is_valid(global_irq_gpio))
		{
		pr_err("=========free=global_irq_gpio==========\n");
		gpio_free(global_irq_gpio);
		}
	input_unregister_device(g_InputDevice);
	g_InputDevice = NULL;

	input_free_device(g_InputDevice);

	return err;
		}
/* [PLATFORM]-ADD-BEGIN by TCTNB.Yubin, PR963868, 2015/04/04, add i2c scan function */
#ifdef CONFIG_TCT_8X16_TAPTI
	else
		i2c_ok = 1;
#endif
/* [PLATFORM]-ADD-END by TCTNB.Yubin */

  #endif
    DrvMainTouchDeviceInitialize();

    DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();
//zxz
    //vPlatformLyrTouchDeviceRegisterEarlySuspend();

#if defined(CONFIG_FB)
	TPfb_notifier.notifier_call = fb_notifier_callback;

	err = fb_register_client(&TPfb_notifier);

	if (err)
		dev_err(&pClient->dev, "Unable to register fb_notifier: %d\n",
			err);
#endif

    DBG("*** MStar touch driver registered ***\n");
    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

    return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() ***\n", __func__);

    DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}    
