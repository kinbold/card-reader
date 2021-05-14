/**
 ******************************************************************************
 * @file    aba-reader.h
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

#ifndef ABA_READER_H_
#define ABA_READER_H_

/**
 * @defgroup MagReader_Module
 * @{
 */

#include <linux/gpio.h>
#include <linux/platform_device.h>

/**
 * @addtogroup MagReader_Export_Defines Definições
 * @{
 */

//! Quantidade de bytes máximo para comandos
#define ABAREADER_CMD_BUF                          16u

//! Quantidade máxima de pulsos para leitura do clock WIEGAND
#define ABAREADER_MAX_BITS                         600u

//! Configurar o tamanho máximo do buffer
#define ABAREADER_BUF_SIZE                         ((ABAREADER_MAX_BITS + 7) / 8)

//! Get a bit from buffer read
#define INTERPRET_GET_BITS_BUFF_DATA(buff, pos)     ((buff[(pos) / 8] & (1 << (7 - ((pos) % 8)))) != 0)

//! Inserir um bit no buffer de dados do Magnético WIEGAND
#define ABAREADER_INSERTBIT2BUF(buff, pos, data)   ((data == 1) ? \
                                                    (buff[pos/8] |= (1 << (7 - (pos % 8)))) : \
                                                    (buff[pos/8] &= ~(1 << (7 - (pos % 8)))))

//! Definição do ID único para controle da rotina IOCTL
#define ABAREADER_IOCTL_APP_TYPE                   0x52

//! Timeout to detect a card number
#define ABA_TIMEOUT                                100 // 100 milliseconds

/**
 * @}
 */

/**
 * @brief For the WIEGAND reader driver
 */
typedef struct s_aba_info {
    bool in_progress;
    unsigned int pulses;
    unsigned long stamp;
    char buffer[ABAREADER_BUF_SIZE];
} s_aba_info_t;


/**
 * @brief Estrutura de controle do driver magnético
 */
struct s_aba_sysfs {
  int status;
  int start_bit;
};

/**
 * \brief Struct Wiegand Driver control
 */
typedef struct s_aba_driver {
    const char * name;
    struct device * dev;
    int minor;
    int clock_irq;
    spinlock_t spinlock;
    struct s_aba_info info;
    struct s_aba_sysfs sysfs;
    struct gpio_desc *data_gpiod;
} s_aba_driver_t;

enum _e_aba_status {
    _E_AS_NONE,
    _E_AS_CARD_AVAILABLE,
    _E_AS_INVALID_BITS,
};

/**
 * @brief Create data of the device driver
 * @param pdev      : platform device driver pointer
 * @param drv_class : class driver
 * @return 0 - success, < 0 - failed
 */
int aba_reader_create(struct platform_device *pdev, struct class * drv_class);

/**
 * \brief Remove data of the device driver
 * @param pdev  : plataform device driver
 */
void aba_reader_destroy(struct platform_device *pdev);

/**
 * \brief Initializes SYSFS data Wiegand Reader
 * @param aba  : Wiegand Device Driver Created
 * @return < 0 --> failed, > 0 - success
 */
int aba_setup_initializes(struct s_aba_driver * aba);


/**
 * @brief remove SYSFS
 * @param aba  : Wiegand device driver created
 */
void aba_setup_remove(struct s_aba_driver * aba);


/**
 * @brief perform card reader
 * @param aba  : Wiegand device driver created
 */
void aba_card_reader(struct s_aba_driver * aba);

/**
 * @}
 */

#endif /* ABA_READER_H_ */
