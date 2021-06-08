/**
 ******************************************************************************
 * @file    aba-reader.c
 * @author  Dimitri Marques - dimitri@ddembedded.com.br
 * @version V0.0.0
 * @date    2016-05-11
 * @brief   Magnetic Reader Linux Kernel module using GPIO interrupts.
 ******************************************************************************
 * @attention
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
 ******************************************************************************
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/kdev_t.h>
#include <linux/spinlock.h>
#include <linux/gpio/consumer.h>
#include "aba-reader.h"
#include "systime.h"

static int aba_minor = 0;

/**
 * @brief Routine to save data from data interrupt
 * @param value :
 *        @arg 0 : From data 0 interrupt
 *        @arg 1 : From data 1 interrupt
 */
static void aba_save_data(struct s_aba_driver * dev_data) {
    int value = !gpiod_get_value(dev_data->data_gpiod);

    if (dev_data->info.pulses == 0) {
        if (value) {
            dev_data->info.in_progress = true;
            ABAREADER_INSERTBIT2BUF(dev_data->info.buffer,
                                    dev_data->info.pulses,
                                    value);
            dev_data->info.pulses++;
        }
    } else if (dev_data->info.pulses < ABAREADER_MAX_BITS
            && dev_data->info.pulses < (ABAREADER_BUF_SIZE * 8)) {
        ABAREADER_INSERTBIT2BUF(dev_data->info.buffer,
                                dev_data->info.pulses,
                                value);
        dev_data->info.pulses++;
    }
    
}

/**
 * @brief The interrupt service routine called on clock falling
 * @param irq   : Interrupt value that was fired
 * @param data  : Data pointer to this driver interrupt
 * @return \ref irqreturn_t
 */
static irqreturn_t aba_reader_data_irq(int irq, void *data) {
    struct s_aba_driver * p = (struct s_aba_driver *) data;

    spin_lock(&p->spinlock);

    if (irq == p->clock_irq) {
        if (p->sysfs.status == _E_AS_NONE) {
            aba_save_data(p);
            systime_start(p->info.stamp);
        }
    }

    spin_unlock(&p->spinlock);

    return IRQ_HANDLED;
}

/**
 * \brief Remove data of the device driver
 * @param pdev  : plataform device driver
 */
void aba_reader_destroy(struct platform_device *pdev) {
    s_aba_driver_t * aba = platform_get_drvdata(pdev);

    if (aba != NULL) {
        aba_setup_remove(aba);
        if (aba->clock_irq > 0)
            free_irq(aba->clock_irq, aba);
        if (aba->dev)
            device_unregister(aba->dev);
        devm_kfree(&pdev->dev, aba);
    }
}

/**
 * \brief Create data of the device driver
 * @param pdev  : plataform device driver
 */
int aba_reader_create(struct platform_device *pdev, struct class * drv_class) {
    int err;
    s_aba_driver_t * aba = NULL;
    struct device_node *np = NULL;

    //*****************************************************************************************************************
    aba = devm_kzalloc(&pdev->dev, sizeof(*aba), GFP_KERNEL);
    if (aba == NULL)
        return -ENOMEM;

    aba->name = NULL;
    aba->dev = NULL;
    aba->minor = aba_minor;
    aba->clock_irq = -1;
    aba->info.pulses = 0;
    aba->info.in_progress = 0;
    memset(aba->info.buffer, 0, ABAREADER_BUF_SIZE);
    spin_lock_init(&aba->spinlock);

    platform_set_drvdata(pdev, aba);

    // Load label name from device tree
    np = of_node_get(pdev->dev.of_node);

    // Determine label to char device
    err = of_property_read_string_index(np, "label", 0, &aba->name);

    if (err < 0) {
        aba->name = pdev->dev.of_node->name;
    }

    pr_info(" node: %s\n", aba->name);

    // Settings data pin first
    aba->data_gpiod = devm_gpiod_get(&pdev->dev, "data", GPIOD_IN);
    if (IS_ERR(aba->data_gpiod)) {
        dev_err(&pdev->dev, " unable to get data gpiod\n");
        return PTR_ERR(aba->data_gpiod);
    }

    // create device driver to system class
    aba->dev = device_create(drv_class,
                             NULL,
                             MKDEV(0, 0),
                             aba,
                             "%s%d",
                             "aba-in",
                             aba->minor);

    if (IS_ERR(aba->dev)) {
        dev_warn(&pdev->dev,
             "device_create failed for aba sysfs\n");
        return -EIO;
    }

    // Set clock interrupt
    aba->clock_irq = platform_get_irq_byname(pdev, "clock");

    if (aba->clock_irq < 0) {
        pr_err(" clock: invalid IRQ %d\n", aba->clock_irq);
        return -EIO;
    }

    err = request_irq(aba->clock_irq,
                      aba_reader_data_irq,
                      IRQF_TRIGGER_FALLING,
                      "aba_clock",
                      aba);

    if (err < 0) {
        aba->clock_irq = -1;
        pr_err(" clock: cannot register IRQ %d\n", aba->clock_irq);
        return -EIO;
    }

    // create sysfs group
    err = aba_setup_initializes(aba);

    if (err < 0)
        return -EIO;

    aba_minor++;

    return 0;
}