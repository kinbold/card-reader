/**
 ******************************************************************************
 * @file    card-reader.c
 * @author  Dimitri Marques - dimitri@ddembedded.com.br
 * @version V0.0.0
 * @date    2016-05-11
 * @brief   Magnetic Reader Linux Kernel module using GPIO interrupts.
 ******************************************************************************
 * @author  Vitor Gomes - vitor.gomes@csgd.com.br
 * @version V3.0.0
 * @date    2021-06-10
 * @brief   Add EM125 driver and mode (wiegand/aba) control
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
#include <linux/fs.h>
#include "aba-reader.h"
#include "wiegand-reader.h"
#include "em125-reader.h"

#define DRV_NAME                "card-reader"
#define DRV_VERSION             "3.0.0"

enum _e_card_reader_mode {
    _E_CDM_NONE,
    _E_CDM_WIEGAND,
    _E_CDM_ABA,
};

// Mode Control
unsigned int mode = _E_CDM_WIEGAND;

static int card_drivers = 0;

struct platform_device *pdev_cardreader;

static ssize_t mode_show(struct class *class,
				struct class_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", mode);
}

static ssize_t mode_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t size)
{
	unsigned int bt;
    int err;
	ssize_t result;
	result = sscanf(buf, "%u", &bt);
	if (result != 1)
		return -EINVAL;

    switch (bt)
    {
        case _E_CDM_WIEGAND:
            
            if (mode == _E_CDM_ABA){
                aba_reader_destroy(pdev_cardreader);
            }

            err = wiegand_reader_create(pdev_cardreader, class);

            if (err < 0) wiegand_reader_destroy(pdev_cardreader);

        break;

        case _E_CDM_ABA:
            if (mode == _E_CDM_WIEGAND){
                wiegand_reader_destroy(pdev_cardreader);
            }

            err = aba_reader_create(pdev_cardreader, class);
            
            if (err < 0) aba_reader_destroy(pdev_cardreader);

        break;

        default:
        break;
    }

    mode = bt;
	return size;
}
static CLASS_ATTR_RW(mode);

static struct attribute *card_class_attrs[] = {
    &class_attr_mode.attr,
    NULL,
};
ATTRIBUTE_GROUPS(card_class);

static struct class card_class = {
    .name = DRV_NAME,
    .owner = THIS_MODULE,
    .class_groups = card_class_groups,
};

static const struct of_device_id card_reader_dt_ids[] = {
    { .compatible = "card-reader,input", },
    { .compatible = "em125-reader,input", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, card_reader_dt_ids);

/**
 * @brief Probe WIEGAND device driver into Kernel
 */
static int card_reader_probe(struct platform_device *pdev) {
    int err = -1;
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

    // create data of the device driver
    if (strcmp(match->compatible, "card-reader,input") == 0) {

        pdev_cardreader = pdev;

        if (mode == _E_CDM_WIEGAND)
            err = wiegand_reader_create(pdev, &card_class);

        if (mode == _E_CDM_ABA)
            err = aba_reader_create(pdev, &card_class);
    }
    else if (strcmp(match->compatible, "em125-reader,input") == 0) {
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

    if (strcmp(match->compatible, "card-reader,input") == 0) { 
        if (mode == _E_CDM_WIEGAND)
            wiegand_reader_destroy(pdev);

        if (mode == _E_CDM_ABA)
            aba_reader_destroy(pdev);
    }
    else if (strcmp(match->compatible, "em125-reader,input") == 0) {
        em125_reader_destroy(pdev);
    }
class_exit:
    return err;
}

static int card_reader_remove(struct platform_device *pdev) {
    const struct of_device_id *match = of_match_device(card_reader_dt_ids, &pdev->dev);

    if (strcmp(match->compatible, "card-reader,input") == 0) { 
        if (mode == _E_CDM_WIEGAND)
            wiegand_reader_destroy(pdev);

        if (mode == _E_CDM_ABA)
            aba_reader_destroy(pdev);
    }
    else if (strcmp(match->compatible, "em125-reader,input") == 0) {
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
