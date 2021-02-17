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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/*include for pio*/
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

/* define device tree */
/* modify temp device tree name */
#ifndef BD7710_FRONT_GPIO_DTNAME
#define BD7710_FRONT_GPIO_DTNAME "mediatek,flashlights_bd7710_front"
#endif

/*define driver name */
#define BD7710_FRONT_NAME "flashlights-bd7710_front"



/* define mutex and work queue */
static DEFINE_MUTEX(bd7710_front_mutex);
static struct work_struct bd7710_front_work;

#define FRONTFLASH_IC_ENABLE_PIN       (GPIO85 | 0x80000000)  //GPIO_CAMERA_FLASH_EN_PIN
#define FRONTFLASH_OUTPUT_PIN    (GPIO86 | 0x80000000)   //GPIO_CAMERA_FLASH_MODE_PIN


/* define usage count */
static int use_count;

/* platform data */
struct bd7710_front_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};
/* define level */
static int g_duty=-1;

extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/



/******************************************************************************
 * bd7710_front operations
 *****************************************************************************/
/* flashlight enable function */
#if 1
static int bd7710_front_enable(void)
{


    mt_set_gpio_mode(FRONTFLASH_IC_ENABLE_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(FRONTFLASH_IC_ENABLE_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(FRONTFLASH_IC_ENABLE_PIN,GPIO_OUT_ONE);
	pr_debug(" frontflash_enable line=%d\n",__LINE__);

    return 0;
}

#else 

static int bd7710_front_enable(void)
{
if(g_duty==1)
	{
				
		mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
		//mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
		pr_debug(" frontflash_enable line=%d\n",__LINE__);
	}
	else
	{
		mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
		pr_debug(" frontflash_enable line=%d\n",__LINE__);
	}

    return 0;

}
#endif

/* flashlight disable function */
static int bd7710_front_disable(void)
{
	mt_set_gpio_mode(FRONTFLASH_IC_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_IC_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_IC_ENABLE_PIN,GPIO_OUT_ZERO);

	pr_debug(" frontflash_disable line=%d\n",__LINE__);
    return 0;
}

/* set flashlight level */
static int bd7710_front_set_level(int level)
{
	g_duty=level;
	pr_debug(" frontflash_level line=%d,duty=%d\n",__LINE__,level);
    return 0;
}

/* flashlight init */
static int bd7710_front_init(void)
{
	mt_set_gpio_mode(FRONTFLASH_IC_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_IC_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_IC_ENABLE_PIN,GPIO_OUT_ZERO);
		
	mt_set_gpio_mode(FRONTFLASH_OUTPUT_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_OUTPUT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_OUTPUT_PIN,GPIO_OUT_ONE);


    
    pr_debug(" frontflash_init line=%d\n",__LINE__);
    return 0;
}

/* flashlight uninit */
static int bd7710_front_uninit(void)
{
	pr_debug(" frontflash_uninit line=%d\n",__LINE__);

	mt_set_gpio_mode(FRONTFLASH_IC_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_IC_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_IC_ENABLE_PIN,GPIO_OUT_ZERO);
    return 0;
}

static void frontflash_flash_mode(void)
{
    pr_debug(" frontflash_flash_mode line=%d\n",__LINE__);

	
	mt_set_gpio_mode(FRONTFLASH_OUTPUT_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_OUTPUT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_OUTPUT_PIN,GPIO_OUT_ONE);
	
	mt_set_gpio_mode(FRONTFLASH_IC_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_IC_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_IC_ENABLE_PIN,GPIO_OUT_ONE);



}

static void frontflash_torch_mode(void)
{
    pr_debug(" frontflash_torch_mode=%d\n",__LINE__);

	mt_set_gpio_mode(FRONTFLASH_OUTPUT_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_OUTPUT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_OUTPUT_PIN,GPIO_OUT_ZERO);
	
	mt_set_gpio_mode(FRONTFLASH_IC_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_IC_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_IC_ENABLE_PIN,GPIO_OUT_ONE);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer bd7710_front_timer;
static unsigned int bd7710_front_timeout_ms;

static void bd7710_front_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	bd7710_front_disable();
}

static enum hrtimer_restart bd7710_front_timer_func(struct hrtimer *timer)
{
	schedule_work(&bd7710_front_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int bd7710_front_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		bd7710_front_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
				if (fl_arg->arg > 1)
				frontflash_flash_mode();
			else
				frontflash_torch_mode();
		bd7710_front_set_level(fl_arg->arg);
		break;


	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (bd7710_front_timeout_ms) {
				s = bd7710_front_timeout_ms / 1000;
				ns = bd7710_front_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&bd7710_front_timer, ktime,
						HRTIMER_MODE_REL);
			}
			bd7710_front_enable();
		} else {
			bd7710_front_disable();
			hrtimer_cancel(&bd7710_front_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int bd7710_front_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int bd7710_front_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int bd7710_front_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&bd7710_front_mutex);
	if (set) {
		if (!use_count)
			ret = bd7710_front_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = bd7710_front_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&bd7710_front_mutex);

	return ret;
}

static ssize_t bd7710_front_strobe_store(struct flashlight_arg arg)
{
	bd7710_front_set_driver(1);
	bd7710_front_set_level(arg.level);
	bd7710_front_timeout_ms = 0;
	bd7710_front_enable();
	msleep(arg.dur);
	bd7710_front_disable();
	bd7710_front_set_driver(0);

	return 0;
}

static struct flashlight_operations bd7710_front_ops = {
	bd7710_front_open,
	bd7710_front_release,
	bd7710_front_ioctl,
	bd7710_front_strobe_store,
	bd7710_front_set_driver
};


static int bd7710_front_probe(struct platform_device *pdev)
{
	struct bd7710_front_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

    
	mt_set_gpio_mode(FRONTFLASH_IC_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_IC_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_IC_ENABLE_PIN,GPIO_OUT_ZERO);
		
	mt_set_gpio_mode(FRONTFLASH_OUTPUT_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(FRONTFLASH_OUTPUT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(FRONTFLASH_OUTPUT_PIN,GPIO_OUT_ZERO);
	
	/* init work queue */
	INIT_WORK(&bd7710_front_work, bd7710_front_work_disable);

	/* init timer */
	hrtimer_init(&bd7710_front_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bd7710_front_timer.function = bd7710_front_timer_func;
	bd7710_front_timeout_ms = 100;



	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	pr_debug("bd7710_front_flashlight_dev_register start.\n");
		if (flashlight_dev_register(BD7710_FRONT_NAME, &bd7710_front_ops)) {
			err = -EFAULT;
			goto err;
		
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int bd7710_front_remove(struct platform_device *pdev)
{
	struct bd7710_front_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(BD7710_FRONT_NAME);

	/* flush work queue */
	flush_work(&bd7710_front_work);

	pr_debug("Remove done.\n");

	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id bd7710_front_gpio_of_match[] = {
	{.compatible = BD7710_FRONT_GPIO_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, bd7710_front_gpio_of_match);
#else
static struct platform_device bd7710_front_gpio_platform_device[] = {
	{
		.name = BD7710_FRONT_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, bd7710_front_gpio_platform_device);
#endif

static struct platform_driver bd7710_front_platform_driver = {
	.probe = bd7710_front_probe,
	.remove = bd7710_front_remove,
	.driver = {
		.name = BD7710_FRONT_GPIO_DTNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bd7710_front_gpio_of_match,
#endif
	},
};

static int __init flashlight_bd7710_front_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&bd7710_front_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&bd7710_front_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_bd7710_front_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&bd7710_front_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_bd7710_front_init);
module_exit(flashlight_bd7710_front_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight BD7710 frontflash GPIO Driver");


