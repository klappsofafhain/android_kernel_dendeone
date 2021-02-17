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
#include <linux/i2c.h>
#include <linux/slab.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/*include for pio*/
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

/* define device tree */
#ifndef BD7710_DTNAME
#define BD7710_DTNAME "mediatek,flashlights_bd7710"
#endif
#ifndef BD7710_DTNAME_I2C
#define BD7710_DTNAME_I2C "mediatek,bd7710" //use custom i2c reg 0x53
#endif

#define BD7710_NAME "flashlights-bd7710"

/* define level */
#define BD7710_LEVEL_NUM 1
#define BD7710_LEVEL_TORCH 1
#define BD7710_HW_TIMEOUT 1000 /* ms */

/*define pins*/
#define REAR_LED_ENABLE_PIN    (GPIO43 | 0x80000000)  //GPIO_TORCH_EN

/* define mutex and work queue */
static DEFINE_MUTEX(bd7710_mutex);
static struct work_struct bd7710_work;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *bd7710_i2c_client;

/* platform data */
struct bd7710_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* bd7710 chip data */
struct bd7710_chip_data {
	struct i2c_client *client;
	struct bd7710_platform_data *pdata;
	struct mutex lock;
};


/******************************************************************************
 * bd7710 operations
 *****************************************************************************/

static int bd7710_level = -1;

static int bd7710_is_torch(int level)
{
	if (level >= BD7710_LEVEL_TORCH)
		return -1;

	return 0;
}

static int bd7710_read_reg(struct i2c_client *client, u8 addr)//, u8 data)
{
    u8 beg = addr;
	int err;
	struct bd7710_chip_data *chip = i2c_get_clientdata(client);
	
	//mutex_lock(&bma050_i2c_mutex);
	mutex_lock(&chip->lock);

	if (!client)
	{
	   // mutex_unlock(&bma050_i2c_mutex);
	mutex_unlock(&chip->lock);
		return -EINVAL;
	}
		

		client->addr &= I2C_MASK_FLAG;
		client->addr |= I2C_WR_FLAG;
		client->addr |= I2C_RS_FLAG;
                client->timing = 100;//zbl add     

	    	err = i2c_master_send(client, &addr, 0x101);


		client->addr &= ~I2C_WR_FLAG;

		mutex_unlock(&chip->lock);

	printk("bd7710 Get i2c addr:0x%x,data=0x%x\n",beg,addr);
	return err;

}

/* i2c wrapper function */
static int bd7710_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct bd7710_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* flashlight enable function */
#define TORCH_DUTY_THR   2
#define MAX_DUTY_THR     11
/* flashlight enable function */
static int bd7710_enable(void)
{
        
        int brightness = 0;
        
       mt_set_gpio_mode(REAR_LED_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(REAR_LED_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(REAR_LED_ENABLE_PIN,GPIO_OUT_ONE);
        mdelay(10);

	if (bd7710_level < TORCH_DUTY_THR)
		/*toucrh mode*/
		{
		brightness = (bd7710_level<<3) | bd7710_level; 
		
            // 200 ma
        bd7710_write_reg(bd7710_i2c_client, 0x10,0x1A); 
        bd7710_write_reg(bd7710_i2c_client, 0xA0, brightness);
        udelay(50);

		
        }   
	    else 
	    {
		/*flashlight mode*/
  		brightness = ((bd7710_level-TORCH_DUTY_THR)<<4) | (bd7710_level - TORCH_DUTY_THR);

        bd7710_write_reg(bd7710_i2c_client, 0x10,0x1B);
        bd7710_write_reg(bd7710_i2c_client, 0xB0 , brightness);
        udelay(50);

		
	}
    return 0;
}

/* flashlight disable function */
static int bd7710_disable(void)
{
	
		
		pr_debug("FL_disable g_duty=%d\n",bd7710_level);
	if( NULL == bd7710_i2c_client)
		return 0;
		
		bd7710_write_reg(bd7710_i2c_client, 0x10,0x00);
     udelay(50);

	mdelay(1);

	
	
	mt_set_gpio_mode(REAR_LED_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(REAR_LED_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(REAR_LED_ENABLE_PIN,GPIO_OUT_ZERO);

	return 0;
}

/* set flashlight level */
static int bd7710_set_level(int level)
{
  bd7710_level = level;
	return 0;
}

/* flashlight init */
static int bd7710_init(void)
{
	return 0;
}

/* flashlight uninit */
static int bd7710_uninit(void)
{
	bd7710_disable();
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer bd7710_timer;
static unsigned int bd7710_timeout_ms;

static void bd7710_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	bd7710_disable();
}

static enum hrtimer_restart bd7710_timer_func(struct hrtimer *timer)
{
	schedule_work(&bd7710_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int bd7710_ioctl(unsigned int cmd, unsigned long arg)
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
		bd7710_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		bd7710_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (bd7710_timeout_ms) {
				s = bd7710_timeout_ms / 1000;
				ns = bd7710_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&bd7710_timer, ktime, HRTIMER_MODE_REL);
			}
			bd7710_enable();
		} else {
			bd7710_disable();
			hrtimer_cancel(&bd7710_timer);
		}
		break;
		
	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = BD7710_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = BD7710_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = BD7710_HW_TIMEOUT;
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int bd7710_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int bd7710_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int bd7710_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&bd7710_mutex);
	if (set) {
		if (!use_count)
			ret = bd7710_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = bd7710_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&bd7710_mutex);

	return ret;
}

static ssize_t bd7710_strobe_store(struct flashlight_arg arg)
{
	bd7710_set_driver(1);
	bd7710_set_level(arg.level);
	bd7710_timeout_ms = 0;
	bd7710_enable();
	msleep(arg.dur);
	bd7710_disable();
	bd7710_set_driver(0);

	return 0;
}

static struct flashlight_operations bd7710_ops = {
	bd7710_open,
	bd7710_release,
	bd7710_ioctl,
	bd7710_strobe_store,
	bd7710_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int bd7710_chip_init(struct bd7710_chip_data *chip)
{
	   int ret =0;
	struct i2c_client *client = chip->client;
	mt_set_gpio_mode(REAR_LED_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(REAR_LED_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(REAR_LED_ENABLE_PIN,GPIO_OUT_ONE);
    ret = bd7710_read_reg(client, 0xFF);
	msleep(10);

	mt_set_gpio_mode(REAR_LED_ENABLE_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(REAR_LED_ENABLE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(REAR_LED_ENABLE_PIN,GPIO_OUT_ZERO);

	if(ret >= 0)
	{
		ret = 0;
	}
	else
	{
		ret = -1;
	}
	return ret;
}

static int bd7710_parse_dt(struct device *dev,
		struct bd7710_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				BD7710_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int bd7710_i2c_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bd7710_chip_data *chip;
	int err;

	pr_debug("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct bd7710_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	i2c_set_clientdata(client, chip);
	bd7710_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	bd7710_chip_init(chip);

	pr_debug("Probe done.\n");

	return 0;

err_out:
	return err;
}

static int bd7710_i2c_remove(struct i2c_client *client)
{
	struct bd7710_chip_data *chip = i2c_get_clientdata(client);

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id bd7710_i2c_id[] = {
	{BD7710_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bd7710_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id bd7710_i2c_of_match[] = {
	{.compatible = BD7710_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, bd7710_i2c_of_match);
#endif

static struct i2c_driver bd7710_i2c_driver = {
	.driver = {
		.name = BD7710_NAME,
#ifdef CONFIG_OF
		.of_match_table = bd7710_i2c_of_match,
#endif
	},
	.probe = bd7710_i2c_probe,
	.remove = bd7710_i2c_remove,
	.id_table = bd7710_i2c_id,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int bd7710_probe(struct platform_device *pdev)
{
	struct bd7710_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct bd7710_chip_data *chip = NULL;
	int err;
	int i;

	pr_debug("Probe start.\n");

	if (i2c_add_driver(&bd7710_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		pdev->dev.platform_data = pdata;
		err = bd7710_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_free;
	}

	/* init work queue */
	INIT_WORK(&bd7710_work, bd7710_work_disable);

	/* init timer */
	bd7710_timeout_ms = 1000;
	hrtimer_init(&bd7710_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bd7710_timer.function = bd7710_timer_func;

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&bd7710_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(BD7710_NAME, &bd7710_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err_free:
	chip = i2c_get_clientdata(bd7710_i2c_client);
	i2c_set_clientdata(bd7710_i2c_client, NULL);
	kfree(chip);
	return err;
}

static int bd7710_remove(struct platform_device *pdev)
{
	struct bd7710_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	i2c_del_driver(&bd7710_i2c_driver);

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(BD7710_NAME);

	/* flush work queue */
	flush_work(&bd7710_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bd7710_of_match[] = {
	{.compatible = BD7710_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, bd7710_of_match);
#else
static struct platform_device bd7710_platform_device[] = {
	{
		.name = BD7710_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, bd7710_platform_device);
#endif

static struct platform_driver bd7710_platform_driver = {
	.probe = bd7710_probe,
	.remove = bd7710_remove,
	.driver = {
		.name = BD7710_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bd7710_of_match,
#endif
	},
};

static int __init flashlight_bd7710_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&bd7710_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&bd7710_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_bd7710_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&bd7710_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_bd7710_init);
module_exit(flashlight_bd7710_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight BD7710 Driver");


