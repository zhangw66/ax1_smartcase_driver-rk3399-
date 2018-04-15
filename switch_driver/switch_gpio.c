/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#define NEOLIX_SWITCH_DEBUG 
#ifdef NEOLIX_SWITCH_DEBUG
#define LOG_LEVEL KERN_DEBUG
#define log_fmt(fmt) "[neolix_switch_debug]" fmt

#define SWITCH_DEBUG(fmt, ...) \
	printk(LOG_LEVEL log_fmt(fmt), ##__VA_ARGS__)
#else 
#define SWITCH_DEBUG(fmt, ...) do{}while(0)
#endif

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
};

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
	SWITCH_DEBUG("%s[%d]:func in\n",__func__, __LINE__);

	state = gpio_get_value(data->gpio);
	SWITCH_DEBUG("%s[%d]:switch state:%d\n",__func__, __LINE__, state);
	switch_set_state(&data->sdev, state);
	SWITCH_DEBUG("%s[%d]:func out\n",__func__, __LINE__);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	SWITCH_DEBUG("%s[%d]:func in\n",__func__, __LINE__);
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;
	SWITCH_DEBUG("%s[%d]:func out\n",__func__, __LINE__);

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int gpio_switch_probe(struct platform_device *pdev)
{
	//struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	struct device_node *np = pdev->dev.of_node;
	enum of_gpio_flags flag;
	int ret = 0;
	SWITCH_DEBUG("%s[%d]:func in\n",__func__, __LINE__);

	/*if (!pdata)
		return -EBUSY;*/

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = "neolix_switch";
	switch_data->gpio = of_get_named_gpio_flags(np, "switch-irq-gpio", 0, &flag);
	if (!gpio_is_valid(switch_data->gpio))
		return -ENXIO;
	switch_data->name_on = NULL;
	switch_data->name_off = NULL;
	switch_data->state_on = "switch_state_on";
	switch_data->state_off = "switch_state_off";
	switch_data->sdev.print_state = switch_gpio_print_state; //state show callback
	SWITCH_DEBUG("%s[%d]:1\n",__func__, __LINE__);

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
	SWITCH_DEBUG("%s[%d]:2\n",__func__, __LINE__);

	ret = devm_gpio_request(&pdev->dev, switch_data->gpio, "echelette_switch_gpio");
	if (ret < 0)
		goto err_request_gpio;
	SWITCH_DEBUG("%s[%d]:3\n",__func__, __LINE__);

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, gpio_switch_work);
	SWITCH_DEBUG("%s[%d]:4\n",__func__, __LINE__);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}
	SWITCH_DEBUG("%s[%d]:5, irq:%d\n",__func__, __LINE__, switch_data->irq);

	ret = request_irq(switch_data->irq, gpio_irq_handler,
			  IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);
	SWITCH_DEBUG("%s[%d]:func out\n",__func__, __LINE__);
	platform_set_drvdata(pdev, switch_data);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);
	SWITCH_DEBUG("%s[%d]:name :%s\n",__func__, __LINE__, switch_data->state_on);
	SWITCH_DEBUG("%s[%d]:irq :%d\n",__func__, __LINE__, switch_data->irq);
	free_irq(switch_data->irq, switch_data);    //最后一个参数一定要写，否则无法free掉irq
	//cancel_work_sync(&switch_data->work);
	//gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}
static const struct of_device_id switch_of_match[] = {
	{ .compatible = "neolix-switch-gpio" },
	{ }
};

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= gpio_switch_remove,
	.driver		= {
		.name	= "switch-gpio",
		.of_match_table = of_match_ptr(switch_of_match),
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("zhangwei@neolix.cn");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
