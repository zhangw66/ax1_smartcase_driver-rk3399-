/* drivers/staging/android/neolix_timed_gpio.c
 *
 * Copyright (C) 2012-2016 ROCKCHIP.
 * Author: jerry <jerry.zhang@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include "timed_output.h"

#define MAX_TIMEOUT        10000	/* 10s */
#define NEOLIX_MOTOR_DEBUG 
#ifdef NEOLIX_MOTOR_DEBUG
#define LOG_LEVEL KERN_DEBUG
#define log_fmt(fmt) "[neolix_motor_debug]" fmt

#define MOTOR_DEBUG(fmt, ...) \
	printk(LOG_LEVEL log_fmt(fmt), ##__VA_ARGS__)
#else 
#define MOTOR_DEBUG(fmt, ...) do{}while(0)
#endif
enum {
	MOTOR_OPEN_DOOR_ID = 0,
	MOTOR_ROTATE_ID = 1,
} motor_name_t;
#define MOTOR_NUMS 2
enum {
	MOTOR_GPIO_POSITIVE = 0,
	MOTOR_GPIO_NEGATIVE = 1,
} motor_gpio_type_t;
 enum  {
	MOTOR_STOP = 0,
	MOTOR_FORWARD,
	MOTOR_REVERSE,
 };
typedef struct motor {
	int gpio[2];
	char motor_type[32];
	struct hrtimer timer;
	struct mutex lock;	/* mutex lock */
	struct work_struct work;
	struct timed_output_dev timed_dev;
} motor_dev_t;

motor_dev_t motor_data[MOTOR_NUMS];


static void motor_control(motor_dev_t *pmotor, int cmd)
{
	char pos_cmd, neg_cmd;
	MOTOR_DEBUG("%s[%d]:func in\n",__func__, __LINE__);
	MOTOR_DEBUG("motor cmd is:%d\n", cmd);
	switch (cmd) {
	case MOTOR_FORWARD:pos_cmd = 0; neg_cmd = 1;
	case MOTOR_REVERSE:pos_cmd = 1; neg_cmd = 0;
	case MOTOR_STOP:pos_cmd = 0; neg_cmd = 0;
	default:
	pos_cmd = 0; neg_cmd = 0;
	}
	gpio_set_value((pmotor->gpio)[MOTOR_GPIO_POSITIVE], pos_cmd);    
   	gpio_set_value((pmotor->gpio)[MOTOR_GPIO_NEGATIVE], neg_cmd);   
	MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);
}
static int motor_id;




static void rk_motor_enable(struct timed_output_dev *sdev, int value)
{
	motor_dev_t *pmotor = container_of(sdev, motor_dev_t, timed_dev);
	MOTOR_DEBUG("%s: fun in\n", __func__);
	MOTOR_DEBUG("control motor name is:%s\n", pmotor->motor_type);
	mutex_lock(&pmotor->lock); 
	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&pmotor->timer);
	cancel_work_sync(&pmotor->work);
	MOTOR_DEBUG("%s[%d]:echo value is：%d\n",__func__, __LINE__, value);
	if (value) {
		if (value > 0) {
			MOTOR_DEBUG("%s[%d]:motor forward\n",__func__, __LINE__);
			motor_control(pmotor, MOTOR_FORWARD);
		} else if (value < 0){
			MOTOR_DEBUG("%s[%d]:motor reverse\n",__func__, __LINE__);
			motor_control(pmotor, MOTOR_REVERSE);
			value *= -1;
		}	
		
		if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;
		MOTOR_DEBUG("%s[%d]:hrtimer time %dms:\n",__func__, __LINE__, value);
		hrtimer_start(&pmotor->timer,
			ns_to_ktime((u64)value * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);
		
	} else {
			motor_control(pmotor, MOTOR_STOP);
	}

	mutex_unlock(&pmotor->lock);
	MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);
}

static int rk_motor_get_time(struct timed_output_dev *sdev)
{
	motor_dev_t *pmotor = container_of(sdev, motor_dev_t, timed_dev);
	MOTOR_DEBUG("%s: fun in\n", __func__);
	MOTOR_DEBUG("name:%s\n", pmotor->motor_type);
	if (hrtimer_active(&(pmotor->timer))) {
		ktime_t r = hrtimer_get_remaining(&(pmotor->timer));
		MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);

		return ktime_to_ms(r);
	}
	MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);

	return 0;
}

static enum hrtimer_restart rk_motor_timer_func(struct hrtimer *timer)
{
	motor_dev_t *pmotor = container_of(timer, motor_dev_t, timer);
	MOTOR_DEBUG("%s: fun in\n", __func__);

	schedule_work(&pmotor->work);
	MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);
	return HRTIMER_NORESTART;
}

static void rk_motor_work(struct work_struct *work)
{
	motor_dev_t *pmotor = container_of(work, motor_dev_t, work);
	MOTOR_DEBUG("%s: fun in\n", __func__);
	motor_control(pmotor, MOTOR_STOP);
	MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);
}


/*
MOTOR_DEBUG("%s[%d]: func in!\n", __func__, __LINE__);
	MOTOR_DEBUG("%s[%d]: dev node info!\n", __func__, __LINE__);
	MOTOR_DEBUG("%s\n",np->full_name);
	MOTOR_DEBUG("%s\n",np->name);
	MOTOR_DEBUG("%s\n",np->type);
	MOTOR_DEBUG("%s[%d]: print dev node info OK!\n", __func__, __LINE__);

*/
static int motor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	enum of_gpio_flags flag;
	motor_dev_t *pmotordata;
	MOTOR_DEBUG("%s: fun in\n", __func__);
	if (!strcmp("motor_open_door", np->name)) {
		motor_id = MOTOR_OPEN_DOOR_ID;
		pmotordata = &motor_data[MOTOR_OPEN_DOOR_ID];
		(*pmotordata).timed_dev.name = "motor_open_door";
		strcpy((*pmotordata).motor_type, "motor_open_door");
		MOTOR_DEBUG("dev node is motor_open_door\n");
	}
	else if(!strcmp("motor_rotate_projector", np->name)) {
		motor_id = MOTOR_ROTATE_ID;
		pmotordata = &motor_data[MOTOR_ROTATE_ID];
		(*pmotordata).timed_dev.name = "motor_rotate_projector";
		strcpy((*pmotordata).motor_type, "motor_rotate_projector");

		MOTOR_DEBUG("dev node is motor_rotate_projector\n");
		
	} else {
		MOTOR_DEBUG("unknown motor type,return with failture!!!\n");
		return -1;
	}
	(*pmotordata).timed_dev.enable = rk_motor_enable; 
	(*pmotordata).timed_dev.get_time = rk_motor_get_time;

		
	(*pmotordata).gpio[MOTOR_GPIO_POSITIVE] = of_get_named_gpio_flags(np, "motor-gpio-positive", 0, &flag);
	if (!gpio_is_valid((*pmotordata).gpio[MOTOR_GPIO_POSITIVE]))
		return -1;
	ret = devm_gpio_request(&pdev->dev, (*pmotordata).gpio[MOTOR_GPIO_POSITIVE], (motor_id == MOTOR_ROTATE_ID) ? "rotate_gpio_positive":"openDoor_gpio_positive");
	if (ret < 0)
		return ret;
	gpio_direction_output((*pmotordata).gpio[MOTOR_GPIO_POSITIVE], flag == OF_GPIO_ACTIVE_LOW ? 0:1);
	
	(*pmotordata).gpio[MOTOR_GPIO_NEGATIVE] = of_get_named_gpio_flags(np, "motor-gpio-negative", 0, &flag);
	if (!gpio_is_valid((*pmotordata).gpio[MOTOR_GPIO_NEGATIVE]))
		return -1;
	ret = devm_gpio_request(&pdev->dev, (*pmotordata).gpio[MOTOR_GPIO_NEGATIVE], (motor_id == MOTOR_ROTATE_ID) ? "rotate_gpio_negative":"openDoor_gpio_negative");
	if (ret < 0)
		return ret;
	gpio_direction_output((*pmotordata).gpio[MOTOR_GPIO_NEGATIVE], flag == OF_GPIO_ACTIVE_LOW ? 0:1);

	hrtimer_init(&(*pmotordata).timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	(*pmotordata).timer.function = rk_motor_timer_func;
	INIT_WORK(&(*pmotordata).work, rk_motor_work);
	//wake_lock_init(&(*pmotordata).wklock, WAKE_LOCK_SUSPEND, NULL);
	mutex_init(&(*pmotordata).lock);
	ret = timed_output_dev_register(&(*pmotordata).timed_dev);
	if (ret < 0)
		goto err_to_dev_reg;
	platform_set_drvdata(pdev, &motor_data[motor_id]);
	MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);
	return 0;

err_to_dev_reg:
	mutex_destroy(&(*pmotordata).lock);
	//wake_lock_destroy(&(*pmotordata).wklock);
	MOTOR_DEBUG("%s[%d]:func out\n",__func__, __LINE__);

	return ret;
}

static int motor_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 2; i++) {
	timed_output_dev_unregister(&(motor_data[i].timed_dev));
	mutex_destroy(&(motor_data[i].lock));
	//for (j = 0; j < 2; j++)
	//	devm_gpio_free(&((motor_data[i].ownpdev)->dev), motor_data[i].gpio[j]);  //automatically freed

}
	return 0;
}

static const struct of_device_id motor_of_match[] = {
	{ .compatible = "neolix-motor-gpio" },
	{ }
};

static struct platform_driver motor_driver = {
	.probe = motor_probe,
	.remove = motor_remove,
	.driver = {
		.name           = "neolix-motor-driver",
		.of_match_table = of_match_ptr(motor_of_match),
	},
};

module_platform_driver(motor_driver);

MODULE_AUTHOR("zhangwei@neolix.cn");
MODULE_DESCRIPTION("neolix motor driver");
MODULE_LICENSE("GPL");
