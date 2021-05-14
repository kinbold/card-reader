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
#include "em125-reader.h"
#include "systime.h"

static int em125_minor = 0;

/**
 * @brief Routine to save data from data interrupt
 * @param value :
 *        @arg 0 : From data 0 interrupt
 *        @arg 1 : From data 1 interrupt
 */
static void em125_save_data(struct s_em125_driver * dev_data) {
    int value = !gpiod_get_value(dev_data->data_gpiod);

    if (dev_data->info.pulses == 0) {
        if (value) {
            dev_data->info.in_progress = true;
            EM125READER_INSERTBIT2BUF(dev_data->info.buffer,
                                    dev_data->info.pulses,
                                    value);
            dev_data->info.pulses++;
        }
    } else if (dev_data->info.pulses < EM125READER_MAX_BITS
            && dev_data->info.pulses < (EM125READER_BUF_SIZE * 8)) {
        EM125READER_INSERTBIT2BUF(dev_data->info.buffer,
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
static irqreturn_t em125_reader_data_irq(int irq, void *data) {
    struct s_em125_driver * p = (struct s_em125_driver *) data;

    spin_lock(&p->spinlock);

    if (irq == p->data_irq) {
        if (p->sysfs.status == _E_ES_NONE) {
            em125_save_data(p);
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
void em125_reader_destroy(struct platform_device *pdev) {
    s_em125_driver_t * em125 = platform_get_drvdata(pdev);

    if (em125 != NULL) {
        em125_setup_remove(em125);
        if (em125->data_irq > 0)
            free_irq(em125->data_irq, em125);
        if (em125->dev)
            device_unregister(em125->dev);
        devm_kfree(&pdev->dev, em125);
    }
}

/**
 * \brief Create data of the device driver
 * @param pdev  : plataform device driver
 */
int em125_reader_create(struct platform_device *pdev, struct class * drv_class) {
    int err;
    s_em125_driver_t * em125 = NULL;
    struct device_node *np = NULL;

    //*****************************************************************************************************************
    em125 = devm_kzalloc(&pdev->dev, sizeof(*em125), GFP_KERNEL);
    if (em125 == NULL)
        return -ENOMEM;

    em125->name = NULL;
    em125->dev = NULL;
    em125->minor = em125_minor;
    em125->data_irq = -1;
    em125->info.pulses = 0;
    em125->info.in_progress = 0;
    memset(em125->info.buffer, 0, EM125READER_BUF_SIZE);
    spin_lock_init(&em125->spinlock);

    platform_set_drvdata(pdev, em125);

    // Load label name from device tree
    np = of_node_get(pdev->dev.of_node);

    // Determine label to char device
    err = of_property_read_string_index(np, "label", 0, &em125->name);

    if (err < 0) {
        em125->name = pdev->dev.of_node->name;
    }

    pr_info(" node: %s\n", em125->name);

    // create device driver to system class
    em125->dev = device_create(drv_class,
                             NULL,
                             MKDEV(0, 0),
                             em125,
                             "%s%d",
                             "em125-in",
                             em125->minor);

    if (IS_ERR(em125->dev)) {
        dev_warn(&pdev->dev,
             "device_create failed for em125 sysfs\n");
        return -EIO;
    }

    // Set data interrupt
    em125->data_irq = platform_get_irq_byname(pdev, "data");

    if (em125->data_irq < 0) {
        pr_err(" data: invalid IRQ %d\n", em125->data_irq);
        return -EIO;
    }

    err = request_irq(em125->data_irq,
                      em125_reader_data_irq,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                      "em125_data",
                      em125);

    if (err < 0) {
        em125->data_irq = -1;
        pr_err(" data: cannot register IRQ %d\n", em125->data_irq);
        return -EIO;
    }

    // create sysfs group
    err = em125_setup_initializes(em125);

    if (err < 0)
        return -EIO;

    em125_minor++;

    return 0;
}
