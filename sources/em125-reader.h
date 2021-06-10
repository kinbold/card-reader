/**
 ******************************************************************************
 * @file    em125-reader.h
 * @author  Vitor Gomes - vitor.gomes@csgd.com.br
 * @version V0.0.1
 * @date    2021-06-07
 * @brief   Em125 Reader Linux Kernel module using GPIO interrupts.
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

#ifndef EM125_READER_H_
#define EM125_READER_H_

/**
 * @defgroup Em125_Module
 * @{
 */

#include <linux/gpio.h>
#include <linux/platform_device.h>


/**
 * @addtogroup MagReader_Export_Defines Definições
 * @{
 */

//! Quantidade de bytes máximo para comandos
#define EM125READER_CMD_BUF                          16u

//! Quantidade máxima de pulsos para leitura do clock WIEGAND
#define EM125READER_MAX_BITS                         600u

//! Configurar o tamanho máximo do buffer
#define EM125READER_BUF_SIZE                         256u

//! Definição do ID único para controle da rotina IOCTL
#define EM125READER_IOCTL_APP_TYPE                   0x52

//! Timeout to detect a card number
#define EM125_TIMEOUT                                100 // 100 milliseconds


#define DATA_ACQUIS_MIN_SAMPLES_EM4100              200

/**
 * @}
 */

/**
 * @brief For the EM125 reader driver
 */
typedef struct s_em125_info {
    bool in_progress;
    unsigned int pulses;
    unsigned long stamp;
    struct timespec start_uptime; 
    char buffer[EM125READER_BUF_SIZE];
} s_em125_info_t;

/**
 * @brief Estrutura de controle do driver 
 */
struct s_em125_sysfs {
  int status;
  int start_bit;
  __u64 id_code;
};

/**
 * \brief Struct EM125 Driver control
 */
typedef struct s_em125_driver {
    const char * name;
    struct device * dev;
    int minor;
    int data_irq;
    spinlock_t spinlock;
    struct s_em125_info info;
    struct s_em125_sysfs sysfs;
    struct pwm_device *antena;
    struct gpio_desc *data_gpiod;
} s_em125_driver_t;

enum _e_em125_status {
    _E_ES_NONE,
    _E_ES_CARD_AVAILABLE,
    _E_ES_INVALID_BITS,
};

/**
 * @brief Create data of the device driver
 * @param pdev      : platform device driver pointer
 * @param drv_class : class driver
 * @return 0 - success, < 0 - failed
 */
int em125_reader_create(struct platform_device *pdev, struct class * drv_class);

/**
 * \brief Remove data of the device driver
 * @param pdev  : plataform device driver
 */
void em125_reader_destroy(struct platform_device *pdev);

/**
 * \brief Initializes SYSFS data Wiegand Reader
 * @param em125  : Wiegand Device Driver Created
 * @return < 0 --> failed, > 0 - success
 */
int em125_setup_initializes(struct s_em125_driver * em125);


/**
 * @brief remove SYSFS
 * @param em125  : Wiegand device driver created
 */
void em125_setup_remove(struct s_em125_driver * em125);


/**
 * @brief perform card reader
 * @param em125  : Wiegand device driver created
 */
void em125_card_reader(struct s_em125_driver * em125);

/**
 * @}
 */

#endif /* em125_READER_H_ */
