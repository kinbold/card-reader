/**
 ******************************************************************************
 * @file    em125-sysfs.c
 * @author  Vitor Gomes - vitor.gomes@csgd.com.br
 * @version V0.0.1
 * @date    2021-06-07
 * @brief   Em125 Reader Sysfs implements
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

#include "em125-reader.h"
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

static const struct attribute *em125_attrs[] = {
    &dev_attr_status.attr,
    &dev_attr_bits.attr,
    &dev_attr_start_bit.attr,
    &dev_attr_id_code.attr,
    &dev_attr_id_bits.attr,
    &dev_attr_clean.attr,
    NULL,
};

static const struct attribute_group em125_attr_group = {
    .attrs = (struct attribute **) em125_attrs,
    .name = "info"
};

int em125_setup_initializes(struct s_em125_driver * em125) {
    int err;

    pr_info(" em125 setup initializes\n");

    // configuração da estrutura de dados do WIEGAND
    em125->sysfs.status = _E_ES_NONE;
    em125->sysfs.start_bit = 0;
    em125->sysfs.id_code = 0;

    // criar o grupo de atributos para controle do driver
    err = sysfs_create_group(&em125->dev->kobj, &em125_attr_group);

    return err;
}

void em125_setup_remove(struct s_em125_driver * em125) {
    sysfs_remove_group(&em125->dev->kobj, &em125_attr_group);
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_em125_driver_t * em125 = (s_em125_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&em125->spinlock, flags))
        return 0;
    if (em125->sysfs.status == _E_ES_NONE && em125->info.in_progress) {
        if (systime_timeout(em125->info.stamp, EM125_TIMEOUT)) {
            em125->sysfs.status = _E_ES_CARD_AVAILABLE;
        }
    }
    len = sprintf(buf, "%d", em125->sysfs.status);
    spin_unlock_irqrestore(&em125->spinlock, flags);
    return len;
}

static ssize_t bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_em125_driver_t * em125 = (s_em125_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&em125->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", em125->info.pulses);
    spin_unlock_irqrestore(&em125->spinlock, flags);
    return len;
}

static ssize_t start_bit_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_em125_driver_t * em125 = (s_em125_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&em125->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", em125->sysfs.start_bit);
    spin_unlock_irqrestore(&em125->spinlock, flags);
    return len;
}

static ssize_t id_code_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_em125_driver_t * em125 = (s_em125_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&em125->spinlock, flags))
        return 0;
        
    len = sprintf(buf, "%-14.14s", /*em125->sysfs.id_code*/em125->info.buffer);
    spin_unlock_irqrestore(&em125->spinlock, flags);
    return len;
}

static ssize_t id_bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_em125_driver_t * em125 = (s_em125_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&em125->spinlock, flags))
        return 0;
    switch (em125->sysfs.status) {
        case _E_ES_NONE:
            if (em125->info.in_progress && systime_timeout(em125->info.stamp, EM125_TIMEOUT)) {
                em125->sysfs.status = _E_ES_CARD_AVAILABLE;
                /*for (;len < em125->info.pulses; len++) {
                    buf[len] = (INTERPRET_GET_BITS_BUFF_DATA(em125->info.buffer, len)) ? '1' : '0';
                }*/
                memcpy(buf, em125->info.buffer, 14);
            }
            break;
        case _E_ES_CARD_AVAILABLE:
           /* for (;len < em125->info.pulses; len++) {
                buf[len] = (INTERPRET_GET_BITS_BUFF_DATA(em125->info.buffer, len)) ? '1' : '0';
            }*/
            memcpy(buf, em125->info.buffer, 14);
            break;
        default:
            break;
    }
    spin_unlock_irqrestore(&em125->spinlock, flags);
    return len;
}

static ssize_t start_bit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_em125_driver_t * em125 = (s_em125_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value < EM125READER_MAX_BITS) {
        if (!spin_trylock_irqsave(&em125->spinlock, flags))
            return 0;
        em125->sysfs.start_bit = value;
        spin_unlock_irqrestore(&em125->spinlock, flags);
        status = size;
    }
    return status;
}

static ssize_t clean_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_em125_driver_t * em125 = (s_em125_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (!spin_trylock_irqsave(&em125->spinlock, flags))
        return 0;
    em125->info.pulses = 0;
    em125->info.in_progress = 0;
    em125->sysfs.status = _E_ES_NONE;
    em125->sysfs.id_code = 0;
    memset(em125->info.buffer, 0, EM125READER_BUF_SIZE);
    spin_unlock_irqrestore(&em125->spinlock, flags);
    status = size;
    return status;
}

/**
 * @}
 */
