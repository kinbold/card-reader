/**
 ******************************************************************************
 * @file    aba-reader-sysfs.c
 * @author  Dimitri Marques - dimitri@ddembedded.com.br
 * @version V0.0.0
 * @date    2016-04-26
 * @brief   Wiegand Reader Sysfs implements
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

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/spinlock.h>

#include "aba-reader.h"
#include "systime.h"

/**
 * @defgroup Sysfs_Control
 * @brief SYSFS Store and Show Functions
 * @{
 */
static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bits_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t start_bit_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t id_code_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t id_bits_show(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t start_bit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t clean_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(bits);
static DEVICE_ATTR_RW(start_bit);
static DEVICE_ATTR_RO(id_code);
static DEVICE_ATTR_RO(id_bits);
static DEVICE_ATTR_WO(clean);

static const struct attribute *aba_attrs[] = {
    &dev_attr_status.attr,
    &dev_attr_bits.attr,
    &dev_attr_start_bit.attr,
    &dev_attr_id_code.attr,
    &dev_attr_id_bits.attr,
    &dev_attr_clean.attr,
    NULL,
};

static const struct attribute_group aba_attr_group = {
    .attrs = (struct attribute **) aba_attrs,
    .name = "info"
};

int aba_setup_initializes(struct s_aba_driver * aba) {
    int err;

    pr_info(" aba setup initializes\n");

    // configuração da estrutura de dados do WIEGAND
    aba->sysfs.status = _E_AS_NONE;
    aba->sysfs.start_bit = 0;

    // criar o grupo de atributos para controle do driver
    err = sysfs_create_group(&aba->dev->kobj, &aba_attr_group);

    return err;
}

void aba_setup_remove(struct s_aba_driver * aba) {
    sysfs_remove_group(&aba->dev->kobj, &aba_attr_group);
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_aba_driver_t * aba = (s_aba_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&aba->spinlock, flags))
        return 0;
    if (aba->sysfs.status == _E_AS_NONE && aba->info.in_progress) {
        if (systime_timeout(aba->info.stamp, ABA_TIMEOUT)) {
            aba->sysfs.status = _E_AS_CARD_AVAILABLE;
        }
    }
    len = sprintf(buf, "%d", aba->sysfs.status);
    spin_unlock_irqrestore(&aba->spinlock, flags);
    return len;
}

static ssize_t bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_aba_driver_t * aba = (s_aba_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&aba->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", aba->info.pulses);
    spin_unlock_irqrestore(&aba->spinlock, flags);
    return len;
}

static ssize_t start_bit_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_aba_driver_t * aba = (s_aba_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&aba->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", aba->sysfs.start_bit);
    spin_unlock_irqrestore(&aba->spinlock, flags);
    return len;
}

static ssize_t id_code_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_aba_driver_t * aba = (s_aba_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    
    if (!spin_trylock_irqsave(&aba->spinlock, flags))
        return 0;

    switch (aba->sysfs.status) {
        case _E_AS_NONE:
            if (aba->info.in_progress && systime_timeout(aba->info.stamp, ABA_TIMEOUT)) {
                aba->sysfs.status = _E_AS_CARD_AVAILABLE;
                len = aba->info.pulses / 8;
                if (len % 8)
                    len++;
                memcpy(buf, aba->info.buffer, len);
                //pr_info("id_code_show1 _E_AS_NONE - buf = %s\n", aba->info.buffer);
            }
            break;
        case _E_AS_CARD_AVAILABLE:
            len = aba->info.pulses / 8;
            if (len % 8)
                len++;
            memcpy(buf, aba->info.buffer, len);
            //pr_info("id_code_show2 _E_AS_CARD_AVAILABLE - buf = %s\n", aba->info.buffer);
            break;
        default:
            break;
    }

    spin_unlock_irqrestore(&aba->spinlock, flags);
    return len;
}

static ssize_t id_bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_aba_driver_t * aba = (s_aba_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&aba->spinlock, flags))
        return 0;
    switch (aba->sysfs.status) {
        case _E_AS_NONE:
            if (aba->info.in_progress && systime_timeout(aba->info.stamp, ABA_TIMEOUT)) {
                aba->sysfs.status = _E_AS_CARD_AVAILABLE;
                for (;len < aba->info.pulses; len++) {
                    buf[len] = (INTERPRET_GET_BITS_BUFF_DATA(aba->info.buffer, len)) ? '1' : '0';
                }
            }
            break;
        case _E_AS_CARD_AVAILABLE:
            for (;len < aba->info.pulses; len++) {
                buf[len] = (INTERPRET_GET_BITS_BUFF_DATA(aba->info.buffer, len)) ? '1' : '0';
            }
            break;
        default:
            break;
    }
    spin_unlock_irqrestore(&aba->spinlock, flags);
    return len;
}

static ssize_t start_bit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_aba_driver_t * aba = (s_aba_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value < ABAREADER_MAX_BITS) {
        if (!spin_trylock_irqsave(&aba->spinlock, flags))
            return 0;
        aba->sysfs.start_bit = value;
        spin_unlock_irqrestore(&aba->spinlock, flags);
        status = size;
    }
    return status;
}

static ssize_t clean_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_aba_driver_t * aba = (s_aba_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (!spin_trylock_irqsave(&aba->spinlock, flags))
        return 0;
    aba->info.pulses = 0;
    aba->info.in_progress = 0;
    aba->sysfs.status = _E_AS_NONE;
    memset(aba->info.buffer, 0, ABAREADER_BUF_SIZE);
    spin_unlock_irqrestore(&aba->spinlock, flags);
    status = size;
    return status;
}

/**
 * @}
 */
