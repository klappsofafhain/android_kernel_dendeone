/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"


/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...) pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)  pr_debug(PFX  fmt, ##args)

#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif


/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
//struct pinctrl_state *cam0_pnd_h = NULL;
//struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_ldo0_h = NULL;
struct pinctrl_state *cam_ldo0_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
/*	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}*/


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	/*externel LDO enable */
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}
	return ret;
}
EXPORT_SYMBOL(mtkcam_gpio_init);

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
	case CAMPDN:
		if (PinIdx == 0) {
			//if (Val == 0)
				//pinctrl_select_state(camctrl, cam0_pnd_l);
			//else
			//	pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
	case CAMLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}




int cntVCAMD = 0;
int cntVCAMA = 0;
int cntVCAMIO = 0;
int cntVCAMAF = 0;
int cntVCAMD_SUB = 0;

static DEFINE_SPINLOCK(kdsensor_pw_cnt_lock);


bool _hwPowerOnCnt(KD_REGULATOR_TYPE_T powerId, int powerVolt, char *mode_name)
{

	if (_hwPowerOn(powerId, powerVolt)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD += 1;
		else if (powerId == VCAMA)
			cntVCAMA += 1;
		else if (powerId == VCAMIO)
			cntVCAMIO += 1;
		else if (powerId == VCAMAF)
			cntVCAMAF += 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB += 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

bool _hwPowerDownCnt(KD_REGULATOR_TYPE_T powerId, char *mode_name)
{

	if (_hwPowerDown(powerId)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD -= 1;
		else if (powerId == VCAMA)
			cntVCAMA -= 1;
		else if (powerId == VCAMIO)
			cntVCAMIO -= 1;
		else if (powerId == VCAMAF)
			cntVCAMAF -= 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB -= 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

void checkPowerBeforClose(char *mode_name)
{

	int i = 0;

	PK_DBG("[checkPowerBeforClose]cntVCAMD:%d, cntVCAMA:%d,cntVCAMIO:%d, cntVCAMAF:%d, cntVCAMD_SUB:%d,\n",
	     cntVCAMD, cntVCAMA, cntVCAMIO, cntVCAMAF, cntVCAMD_SUB);


	for (i = 0; i < cntVCAMD; i++)
		_hwPowerDown(VCAMD);
	for (i = 0; i < cntVCAMA; i++)
		_hwPowerDown(VCAMA);
	for (i = 0; i < cntVCAMIO; i++)
		_hwPowerDown(VCAMIO);
	for (i = 0; i < cntVCAMAF; i++)
		_hwPowerDown(VCAMAF);
	for (i = 0; i < cntVCAMD_SUB; i++)
		_hwPowerDown(SUB_VCAMD);

	cntVCAMD = 0;
	cntVCAMA = 0;
	cntVCAMIO = 0;
	cntVCAMAF = 0;
	cntVCAMD_SUB = 0;

}
EXPORT_SYMBOL(checkPowerBeforClose);



extern int subcam_power_lowactive;
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On,
		       char *mode_name)
{

	u32 pinSetIdx = 0;
	int poweron_err = -1;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

	u32 pinSet[3][8] = {

		{CAMERA_CMRST_PIN,
		 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 CAMERA_CMPDN_PIN,
		 CAMERA_CMPDN_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{CAMERA_CMRST1_PIN,
		 CAMERA_CMRST1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 CAMERA_CMPDN1_PIN,
		 CAMERA_CMPDN1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 }
	};



	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	 else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	 else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;

	 PK_DBG("pinSetIdx = %d,SensorIdx = %d\n",pinSetIdx,SensorIdx);
	 if (On) {
		 if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K3H7YX_MIPI_RAW, currSensorName))&& (pinSetIdx == 0)) {

			 poweron_err = 0;
			 /* Set CLK ON */
			 ISP_MCLK1_EN(1);

			 /* First Reset Pin Low */
			 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				 mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
				 if(subcam_power_lowactive)
				 {
					mtkcam_gpio_set(1, CAMRST,pinSet[1][IDX_PS_CMRST + IDX_PS_ON]);
				 }
			 }

			 mdelay(1);

			 /* VCAM_D */
			 if (TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name)) {
				 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D),power id = %d\n",VCAMD);
				 goto _kdCISModulePowerOn_exit_;
			 }

			 /* VCAM_A */
			 if (TRUE != _hwPowerOnCnt(VCAMA, VOL_2800, mode_name)) {
				 PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A),power id = %d\n",VCAMA);
				 goto _kdCISModulePowerOn_exit_;
			 }

			 /* VCAM_IO */
			 if (TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name)) {
				 PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO),power id = %d\n",VCAMIO);
				 goto _kdCISModulePowerOn_exit_;
			 }

			 mdelay(1);

			 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				 mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
				 if(subcam_power_lowactive)
				 {
					mtkcam_gpio_set(1, CAMRST,pinSet[1][IDX_PS_CMRST + IDX_PS_ON]);
				 }
			 }
	 }
		else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, currSensorName)) && (pinSetIdx == 1)) {

			 poweron_err = 0;
			/* First Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
				 if(0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, currSensorName))
				 {
					mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
				 }
				 else
				 {
					mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
				 }
			}

			mdelay(1);

			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO),power id = %d\n",VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			/* VCAM_D */
			if (TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D),power id = %d\n",VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			/* VCAM_A */
			if (TRUE != _hwPowerOnCnt(VCAMA, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A),power id = %d\n",VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}

			/* Set CLK ON */
			ISP_MCLK1_EN(1);

			mdelay(1);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				 if(0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, currSensorName))
				 {
					mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
				 }
				 else
				 {
					mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
				 }
			}
		}
} else {		/* power OFF */

	PK_DBG("[PowerOFF]pinSetIdx:%d\n", pinSetIdx);
		 if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K3H7YX_MIPI_RAW, currSensorName)) && (pinSetIdx == 0)) {

			 poweron_err = 0;
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
			}

			/* Set CLK OFF */
			ISP_MCLK1_EN(0);

			mdelay(1);

			/* VCAM_D */
			if (TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d\n",VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_A */
			if (TRUE != _hwPowerDownCnt(VCAMA, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d)\n",VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
		}
		else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, currSensorName)) && (pinSetIdx == 1)) {


			 poweron_err = 0;
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				 if(0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, currSensorName) || subcam_power_lowactive)
				 {
					mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
				 }
				 else
				 {
					mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
				 }
			}

			/* Set CLK OFF */
			ISP_MCLK1_EN(0);

			/* VCAM_A */
			if (TRUE != _hwPowerDownCnt(VCAMA, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d)\n",VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_D */
			if (TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d\n",VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

		}
	}
//misty for test tmp
/*
if(On){
		switch(pinSetIdx){
        case 0:	//main
		    PK_ERR("[MIMI CAMERA SENSOR]%s\n %s()%4d\n", __FILE__, __func__  , __LINE__);
			fih_bbs_camera_msg_by_addr(FIH_I2C_ADDR_S5K5E8_CAMERA, FIH_BBS_CAMERA_ERRORCODE_POWER_UP);
			break;
		case 1://front
		    PK_ERR("[MIMI CAMERA SENSOR]%s\n %s()%4d\n", __FILE__, __func__  , __LINE__);
		    fih_bbs_camera_msg_by_addr(FIH_I2C_ADDR_GC2385_CAMERA, FIH_BBS_CAMERA_ERRORCODE_POWER_UP);
			break;
		}
	}
else{
		switch(pinSetIdx){
        case 0:	//main
		    PK_ERR("[MIMI CAMERA SENSOR]%s\n %s()%4d\n", __FILE__, __func__  , __LINE__);
			fih_bbs_camera_msg_by_addr(FIH_I2C_ADDR_S5K5E8_CAMERA, FIH_BBS_CAMERA_ERRORCODE_POWER_DW);
			break;
		case 1://front
		    PK_ERR("[MIMI CAMERA SENSOR]%s\n %s()%4d\n", __FILE__, __func__  , __LINE__);
		    fih_bbs_camera_msg_by_addr(FIH_I2C_ADDR_GC2385_CAMERA, FIH_BBS_CAMERA_ERRORCODE_POWER_DW);
			break;
		}
	}
*/
return poweron_err;


_kdCISModulePowerOn_exit_:

	return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

/* !-- */
/*  */
