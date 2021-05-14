/**
 ******************************************************************************
 * @file    wiegand-reader-sysfs.c
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

#include "wiegand-reader.h"
#include "systime.h"

/**
 * @defgroup Sysfs_Control
 * @brief SYSFS Store and Show Functions
 * @{
 */
static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bits_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t parity_enabled_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t parity_first_bits_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t parity_first_type_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t parity_last_bits_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t parity_last_type_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t facility_enabled_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t facility_bits_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t start_bit_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t facility_code_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t id_code_show(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t parity_enabled_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t parity_first_bits_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t parity_first_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t parity_last_bits_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t parity_last_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t facility_enabled_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t facility_bits_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t start_bit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t facility_code_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t clean_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(bits);
static DEVICE_ATTR_RW(parity_enabled);
static DEVICE_ATTR_RW(parity_first_bits);
static DEVICE_ATTR_RW(parity_first_type);
static DEVICE_ATTR_RW(parity_last_bits);
static DEVICE_ATTR_RW(parity_last_type);
static DEVICE_ATTR_RW(facility_enabled);
static DEVICE_ATTR_RW(facility_bits);
static DEVICE_ATTR_RW(start_bit);
static DEVICE_ATTR_RW(facility_code);
static DEVICE_ATTR_RO(id_code);
static DEVICE_ATTR_WO(clean);

static const struct attribute *wieg_attrs[] = {
    &dev_attr_status.attr,
    &dev_attr_bits.attr,
    &dev_attr_parity_enabled.attr,
    &dev_attr_parity_first_bits.attr,
    &dev_attr_parity_first_type.attr,
    &dev_attr_parity_last_bits.attr,
    &dev_attr_parity_last_type.attr,
    &dev_attr_facility_enabled.attr,
    &dev_attr_facility_bits.attr,
    &dev_attr_start_bit.attr,
    &dev_attr_facility_code.attr,
    &dev_attr_id_code.attr,
    &dev_attr_clean.attr,
    NULL, };

static const struct attribute_group wieg_attr_group = { .attrs = (struct attribute **) wieg_attrs, .name = "info" };

int wieg_setup_initializes(struct s_wieg_driver * wieg) {
    int err;

    pr_info(" wiegand setup initializes\n");

    // configuração da estrutura de dados do WIEGAND
    wieg->sysfs.status = _E_WS_NONE;
    wieg->sysfs.parity_enabled = 1;
    wieg->sysfs.parity_first_bits = 20;
    wieg->sysfs.parity_first_type = WIEG_PARITY_EVEN;
    wieg->sysfs.parity_last_bits = 40;
    wieg->sysfs.parity_last_type = WIEG_PARITY_ODD;
    wieg->sysfs.facility_enabled = 0;
    wieg->sysfs.facility_bits = 8;
    wieg->sysfs.start_bit = 0;
    wieg->sysfs.facility_code = 0;
    wieg->sysfs.id_code = 0;

    // criar o grupo de atributos para controle do driver
    err = sysfs_create_group(&wieg->dev->kobj, &wieg_attr_group);

    return err;
}

void wieg_setup_remove(struct s_wieg_driver * wieg) {
    sysfs_remove_group(&wieg->dev->kobj, &wieg_attr_group);
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    wieg_card_reader(wieg);
    len = sprintf(buf, "%d", wieg->sysfs.status);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->info.pulses);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t parity_enabled_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->sysfs.parity_enabled);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t parity_first_bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->sysfs.parity_first_bits);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t parity_first_type_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%s", (wieg->sysfs.parity_first_type == WIEG_PARITY_ODD) ? "odd" : "even");
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t parity_last_bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->sysfs.parity_last_bits);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t parity_last_type_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%s", (wieg->sysfs.parity_last_type == WIEG_PARITY_ODD) ? "odd" : "even");
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t facility_enabled_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->sysfs.facility_enabled);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t facility_bits_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->sysfs.facility_bits);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t start_bit_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->sysfs.start_bit);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t facility_code_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%d", wieg->sysfs.facility_code);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t id_code_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    len = sprintf(buf, "%llu", wieg->sysfs.id_code);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

static ssize_t parity_enabled_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf,
                                         size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value == 0 || value == 1) {
        if (!spin_trylock_irqsave(&wieg->spinlock, flags))
            return 0;
        wieg->sysfs.parity_enabled = value;
        spin_unlock_irqrestore(&wieg->spinlock, flags);
        status = size;
    }
    return status;
}

static ssize_t parity_first_bits_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value < WIEGREADER_MAX_BITS) {
        if (!spin_trylock_irqsave(&wieg->spinlock, flags))
            return 0;
        wieg->sysfs.parity_first_bits = value;
        spin_unlock_irqrestore(&wieg->spinlock, flags);
        status = size;
    }
    return status;
}

static ssize_t parity_first_type_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    if (sysfs_streq(buf, "even")) {
        wieg->sysfs.parity_first_type = 1;
        status = size;
    }
    else if (sysfs_streq(buf, "odd")) {
        wieg->sysfs.parity_first_type = 0;
        status = size;
    }
    spin_unlock_irqrestore(&wieg->spinlock, flags);

    return status;
}

static ssize_t parity_last_bits_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf,
                                            size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value < WIEGREADER_MAX_BITS) {
        if (!spin_trylock_irqsave(&wieg->spinlock, flags))
            return 0;
        wieg->sysfs.parity_last_bits = value;
        spin_unlock_irqrestore(&wieg->spinlock, flags);
        status = size;
    }
    return status;
}

static ssize_t parity_last_type_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf,
                                            size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    if (sysfs_streq(buf, "even")) {
        wieg->sysfs.parity_last_type = 1;
        status = size;
    }
    else if (sysfs_streq(buf, "odd")) {
        wieg->sysfs.parity_last_type = 0;
        status = size;
    }
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return status;
}

static ssize_t facility_enabled_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value == 0 || value == 1) {
        if (!spin_trylock_irqsave(&wieg->spinlock, flags))
            return 0;
        wieg->sysfs.facility_enabled = value;
        spin_unlock_irqrestore(&wieg->spinlock, flags);
        status = size;
    }

    return status;
}

static ssize_t facility_bits_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value < WIEGREADER_MAX_BITS) {
        if (!spin_trylock_irqsave(&wieg->spinlock, flags))
            return 0;
        wieg->sysfs.facility_bits = value;
        spin_unlock_irqrestore(&wieg->spinlock, flags);
        status = size;
    }
    return status;
}

static ssize_t start_bit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (value < WIEGREADER_MAX_BITS) {
        if (!spin_trylock_irqsave(&wieg->spinlock, flags))
            return 0;
        wieg->sysfs.start_bit = value;
        spin_unlock_irqrestore(&wieg->spinlock, flags);
        status = size;
    }
    return status;
}

static ssize_t facility_code_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    wieg->sysfs.facility_code = value;
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    status = size;
    return status;
}

static ssize_t clean_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    ssize_t status = -EINVAL;
    int value = 0;
    unsigned long flags;
    sscanf(buf, "%d", &value);
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    wieg->info.pulses = 0;
    wieg->info.in_progress = 0;
    wieg->sysfs.status = _E_WS_NONE;
    wieg->sysfs.id_code = 0;
    wieg->sysfs.facility_code = 0;
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    status = size;
    return status;
}

ssize_t card_show(struct device *dev, struct device_attribute *attr, char *buf) {
    s_wieg_driver_t * wieg = (s_wieg_driver_t *) dev_get_drvdata(dev);
    int i;
    ssize_t len = 0;
    unsigned long flags;
    if (!spin_trylock_irqsave(&wieg->spinlock, flags))
        return 0;
    wieg->sysfs.id_code = 0;
    if (systime_timeout(wieg->info.stamp, WIEG_TIMEOUT)) {
        for (i = (wieg->sysfs.parity_enabled) ? 1 : 0; i < wieg->info.pulses; i++) {
            if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, i))
                wieg->sysfs.id_code |= 0x1;
            wieg->sysfs.id_code <<= 1;
        }
    }

    if (wieg->sysfs.parity_enabled)
        wieg->sysfs.id_code >>= 2;

    len = sprintf(buf, "%llu", wieg->sysfs.id_code);
    spin_unlock_irqrestore(&wieg->spinlock, flags);
    return len;
}

void wieg_card_reader(struct s_wieg_driver * wieg) {
    if (wieg->sysfs.status == _E_WS_NONE && wieg->info.in_progress) {
        if (systime_timeout(wieg->info.stamp, WIEG_TIMEOUT)) {
            if (!wieg->info.pulses || wieg->info.pulses > WIEGREADER_MAX_BITS) {
                wieg->sysfs.status = _E_WS_INVALID_BITS;
            }
            else {
                int i;

                if (wieg->sysfs.parity_enabled) {
                    bool parity;

                    if ((wieg->sysfs.parity_first_bits > wieg->info.pulses)
                                    || (wieg->sysfs.parity_last_bits > wieg->info.pulses)) {
                        wieg->sysfs.status = _E_WS_INVALID_PARITY_SETUP;
                        return;
                    }

                    // copy the facility code
                    if (wieg->sysfs.facility_enabled) {
                        // pos = start_bit + one byte of parity
                        for (i = wieg->sysfs.start_bit + 1; i < wieg->sysfs.facility_bits; i++) {
                            if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, i))
                                wieg->sysfs.id_code |= 0x1;
                            wieg->sysfs.facility_code <<= 1;
                        }
                    }
                    // copy the card number, adjust pulses with parity
                    else if ((wieg->sysfs.start_bit + 2) < wieg->info.pulses) {
                        int pulses = wieg->info.pulses - wieg->sysfs.start_bit - 1;
                        wieg->sysfs.id_code = 0;
                        for (i = wieg->sysfs.start_bit + 1; i < pulses; i++) {
                            if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, i))
                                wieg->sysfs.id_code |= 0x1;
                            wieg->sysfs.id_code <<= 1;
                        }
                        wieg->sysfs.id_code >>= 1;
                    }

                    // check first parity
                    parity = (wieg->sysfs.parity_first_type == WIEG_PARITY_ODD) ? false : true;

                    for (i = 0; i < wieg->sysfs.parity_first_bits; i++) {
                        if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, i))
                            parity = !parity;
                    }

                    if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, 0) != parity) {
                        wieg->sysfs.status = _E_WS_LEFT_PARITY_FAILED;
                        return;
                    }

                    // check last parity
                    parity = (wieg->sysfs.parity_last_type == WIEG_PARITY_ODD) ? false : true;

                    // ajustar a paridade de acordo com a quantidade de bits para o lado esquerdo
                    for (i = wieg->info.pulses - wieg->sysfs.parity_last_bits; i < wieg->sysfs.parity_last_bits; i++) {
                        if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, i))
                            parity = !parity;
                    }

                    if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, (wieg->info.pulses - 1)) != parity) {
                        wieg->sysfs.status = _E_WS_RIGHT_PARITY_FAILED;
                        return;
                    }
                }
                else if (wieg->sysfs.start_bit) { // wieg->sysfs.id_code already shift by interrupt handler
                    // agora se tivermos start_bit devemos tratar um novo id
                    wieg->sysfs.id_code = 0;
                    for (i = wieg->sysfs.start_bit; i < wieg->info.pulses; i++) {
                        wieg->sysfs.id_code <<= 1;
                        if (INTERPRET_GET_BITS_BUFF_DATA(wieg->info.buffer, i))
                            wieg->sysfs.id_code |= 0x1;
                    }
                }

                wieg->sysfs.status = _E_WS_CARD_AVAILABLE;
            }
        }
    }
}

/**
 * @}
 */
