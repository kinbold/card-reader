/**
 ******************************************************************************
 * @file    wiegand-reader.c
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

#include "wiegand-reader.h"
#include "systime.h"

static int wieg_minor = 0;

/**
 * @brief Routine to save data from data interrupt
 * @param value :
 *        @arg 0 : From data 0 interrupt
 *        @arg 1 : From data 1 interrupt
 */
static void wiegreader_save_data(struct s_wieg_driver * dev_data, int value) {
    if (dev_data->info.pulses == 0) {
        dev_data->info.in_progress = true;
        WIEGREADER_INSERTBIT2BUF(dev_data->info.buffer,
                                 dev_data->info.pulses,
                                 value);
        dev_data->info.pulses++;
        dev_data->sysfs.id_code = (value != 0x0);
    } else if (dev_data->info.pulses < WIEGREADER_MAX_BITS
            && dev_data->info.pulses < (WIEGREADER_BUF_SIZE * 8)) {
        WIEGREADER_INSERTBIT2BUF(dev_data->info.buffer,
                                 dev_data->info.pulses,
                                 value);
        dev_data->info.pulses++;
        dev_data->sysfs.id_code <<= 1;
        dev_data->sysfs.id_code |= (value != 0x0);
    }
}

/**
 * @brief The interrupt service routine called on clock falling
 * @param irq   : Interrupt value that was fired
 * @param data  : Data pointer to this driver interrupt
 * @return \ref irqreturn_t
 */
static irqreturn_t wiegand_reader_data_irq(int irq, void *data) {
    struct s_wieg_driver * p = (struct s_wieg_driver *) data;

    spin_lock(&p->spinlock);

    if (p->sysfs.status == _E_WS_NONE) {
        if (irq == p->data0_irq)
            wiegreader_save_data(p, 0);
        else if (irq == p->data1_irq)
            wiegreader_save_data(p, 1);
        systime_start(p->info.stamp);
    }

    spin_unlock(&p->spinlock);

    return IRQ_HANDLED;
}

/**
 * \brief Remove data of the device driver
 * @param pdev  : plataform device driver
 */
void wiegand_reader_destroy(struct platform_device *pdev) {
    s_wieg_driver_t * wieg = platform_get_drvdata(pdev);

    if (wieg != NULL) {
        wieg_setup_remove(wieg);

        if (wieg->data0_irq > 0)
            free_irq(wieg->data0_irq, wieg);
        if (wieg->data1_irq > 0)
            free_irq(wieg->data1_irq, wieg);
        if (wieg->dev)
            device_unregister(wieg->dev);
        devm_kfree(&pdev->dev, wieg);
    }
}

/**
 * \brief Create data of the device driver
 * @param pdev  : plataform device driver
 */
int wiegand_reader_create(struct platform_device *pdev, struct class * drv_class) {
    int err;
    s_wieg_driver_t * wieg = NULL;
    struct device_node *np = NULL;

    wieg = devm_kzalloc(&pdev->dev, sizeof(*wieg), GFP_KERNEL);
    if (wieg == NULL)
        return -ENOMEM;

    wieg->name = NULL;
    wieg->dev = NULL;
    wieg->minor = wieg_minor;
    wieg->data0_irq = -1;
    wieg->data1_irq = -1;
    wieg->info.pulses = 0;
    wieg->info.in_progress = 0;
    memset(wieg->info.buffer, 0, WIEGREADER_BUF_SIZE);
    spin_lock_init(&wieg->spinlock);

    platform_set_drvdata(pdev, wieg);

    // Load label name from device tree
    np = of_node_get(pdev->dev.of_node);

    // Determine label to char device
    err = of_property_read_string_index(np, "label", 0, &wieg->name);

    if (err < 0) {
        wieg->name = pdev->dev.of_node->name;
    }

    pr_info(" node: %s\n", wieg->name);

    // Set data0 interrupt
    wieg->data0_irq = platform_get_irq_byname(pdev, "data0");

    if (wieg->data0_irq < 0) {
        pr_err(" data0: invalid IRQ %d\n", wieg->data0_irq);
        return -EIO;
    }

    err = request_irq(wieg->data0_irq,
                      wiegand_reader_data_irq,
                      IRQF_TRIGGER_FALLING,
                      "wiegand_data0",
                      wieg);

    if (err < 0) {
        wieg->data0_irq = -1;
        pr_err(" data0: cannot register IRQ %d\n", wieg->data0_irq);
        return -EIO;
    }

    // Set data1 interrupt
    wieg->data1_irq = platform_get_irq_byname(pdev, "data1");

    if (wieg->data1_irq < 0) {
        pr_err(" data1: invalid IRQ %d\n", wieg->data1_irq);
        return -EIO;
    }

    err = request_irq(wieg->data1_irq,
                      &wiegand_reader_data_irq,
                      IRQF_TRIGGER_FALLING,
                      "wiegand_data1",
                      wieg);

    if (err < 0) {
        wieg->data1_irq = -1;
        pr_err(" data1: cannot register IRQ %d\n", wieg->data1_irq);
        return -EIO;
    }

    wieg->dev = device_create(drv_class,
                              NULL,
                              MKDEV(0, 0),
                              wieg,
                              "%s%d",
                              "wieg-in",
                              wieg->minor);

    if (IS_ERR(wieg->dev)) {
        dev_warn(&pdev->dev,
             "device_create failed for wiegand sysfs\n");
        return -EIO;
    }

    // create sysfs group
    err = wieg_setup_initializes(wieg);

    if (err < 0)
        return -EIO;

    wieg_minor++;

    return 0;
}
