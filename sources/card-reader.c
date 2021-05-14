/**
 ******************************************************************************
 * @file    card-reader.c
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
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include "aba-reader.h"
#include "wiegand-reader.h"
#include "em125-reader.h"

#define DRV_NAME                "card-reader"
#define DRV_VERSION             "3.0.0"

static int card_drivers = 0;

static struct class_attribute card_class_attrs[] = {
__ATTR_NULL
};

static struct class card_class = {
    .name = DRV_NAME,
    .owner = THIS_MODULE,
    /*.class_attrs = card_class_attrs*/
};

static const struct of_device_id card_reader_dt_ids[] = {
    { .compatible = "wiegand-reader,input", },
    { .compatible = "aba-reader,input", },
    { .compatible = "em125,input", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, card_reader_dt_ids);

/**
 * @brief Probe WIEGAND device driver into Kernel
 */
static int card_reader_probe(struct platform_device *pdev) {
    int err;
    const struct of_device_id *match;

    pr_info("card-reader %d probe\n", card_drivers);
    pr_info(" version %s\n", DRV_VERSION);

    match = of_match_device(card_reader_dt_ids, &pdev->dev);
    pr_info(" %s will be created\n", (const char*)match->compatible);

    if (!card_drivers) {
        err = class_register(&card_class);
        if (err < 0)
            goto class_exit;
    }

    // create date of the device driver
    if (strcmp(match->compatible, "wiegand-reader,input") == 0) {
        err = wiegand_reader_create(pdev, &card_class);
    }
    else if (strcmp(match->compatible, "aba-reader,input") == 0) {
        err = aba_reader_create(pdev, &card_class);
    }
    else if (strcmp(match->compatible, "em125,input") == 0) {
        err = em125_reader_create(pdev, &card_class);
    }
    else {
        err = -1;
    }

    if (err < 0)
        goto exit;

    pr_info("card-reader %d success\n", card_drivers);
    card_drivers++;

    return 0;
exit:
    if (strcmp(match->compatible, "wiegand-reader,input") == 0) {
        wiegand_reader_destroy(pdev);
    }
    else if (strcmp(match->compatible, "aba-reader,input") == 0) {
        aba_reader_destroy(pdev);
    }
    else if (strcmp(match->compatible, "em125,input") == 0) {
        em125_reader_destroy(pdev);
    }
class_exit:
    return err;
}

static int card_reader_remove(struct platform_device *pdev) {
    const struct of_device_id *match = of_match_device(card_reader_dt_ids, &pdev->dev);

    if (strcmp(match->compatible, "wiegand-reader,input") == 0) {
        wiegand_reader_destroy(pdev);
    }
    else if (strcmp(match->compatible, "aba-reader,input") == 0) {
        aba_reader_destroy(pdev);
    }
    else if (strcmp(match->compatible, "em125,input") == 0) {
        em125_reader_destroy(pdev);
    }

    if (card_drivers) {
        card_drivers--;

        if (!card_drivers) {
            class_unregister(&card_class);
        }
    }

    pr_info(" card input remove %d driver\n", card_drivers);
    return 0;
}

static struct platform_driver card_reader_driver = {
    .driver = {
        .name = DRV_NAME,
        .of_match_table = card_reader_dt_ids,
    },
    .probe = card_reader_probe,
    .remove = card_reader_remove,
};

module_platform_driver(card_reader_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dimitri Marques <dimitri@ddembedded.com.br>");
MODULE_DESCRIPTION("Card Reader Device Driver, Wiegand and ABA Technologies");
MODULE_VERSION(DRV_VERSION);
