/**
 ******************************************************************************
 * @file    systime.h
 * @author  Dimitri Marques - dimitri@ddembedded.com.br
 * @version V0.0.0
 * @date    2016-04-26
 * @brief   Source to control systime
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

#ifndef SYSTIME_H_
#define SYSTIME_H_

#include <linux/jiffies.h>

/**
 * @defgroup SysTime_Module
 * @{
 */

//! Start timer routine
#define systime_start(timer)                (timer=jiffies_to_msecs(jiffies))
#define systime_start_us(timer)              (timer=jiffies_to_usecs(jiffies))
#define systime_start_ns(timer)              (timer=jiffies_to_nsecs(jiffies))

//! Timeout timer routine
#define systime_timeout(timer, timeout)   ((jiffies_to_msecs(jiffies)-timer) >= timeout)


/**
 * @}
 */

#endif /* SYSTIME_H_ */
