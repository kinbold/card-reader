/**
 ******************************************************************************
 * @file    wiegand-reader.h
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

#ifndef WIEGAND_READER_H_
#define WIEGAND_READER_H_

/**
 * @defgroup MagReader_Module
 * @{
 */

#include <linux/platform_device.h>

/**
 * @addtogroup MagReader_Export_Defines Definições
 * @{
 */

//! Quantidade de sensores WIEGAND que podem ter neste driver
#define WIEGAND_READERS_TOTAL                       2

//! Quantidade de bytes máximo para comandos
#define WIEGREADER_CMD_BUF                          16u

//! Quantidade máxima de pulsos para leitura do clock WIEGAND
#define WIEGREADER_MAX_BITS                         128u

//! Configurar o tamanho máximo do buffer
#define WIEGREADER_BUF_SIZE                         ((WIEGREADER_MAX_BITS + 7) / 8)

//! Get a bit from buffer read
#define INTERPRET_GET_BITS_BUFF_DATA(buff, pos)     ((buff[(pos) / 8] & (1 << (7 - ((pos) % 8)))) != 0)

//! Inserir um bit no buffer de dados do Magnético WIEGAND
#define WIEGREADER_INSERTBIT2BUF(buff, pos, data)   ((data == 1) ? \
                                                     (buff[pos/8] |= (1 << (7 - (pos % 8)))) : \
                                                     (buff[pos/8] &= ~(1 << (7 - (pos % 8)))))

//! Definição do ID único para controle da rotina IOCTL
#define WIEGREADER_IOCTL_APP_TYPE                   0x52

//! Valor de paridade ímpar
#define WIEG_PARITY_ODD                             0

//! Valor de paridade par
#define WIEG_PARITY_EVEN                            1

//! Timeout to detect a card number
#define WIEG_TIMEOUT                                100 // 100 milliseconds

/**
 * @}
 */

/**
 * @brief For the WIEGAND reader driver
 */
typedef struct s_wieg_info {
    bool in_progress;
    unsigned int pulses;
    unsigned long stamp;
    char buffer[WIEGREADER_BUF_SIZE];
} s_wieg_info_t;


/**
 * @brief Estrutura de controle do driver magnético
 */
struct s_wieg_sysfs {
  int status;
  bool parity_enabled;
  int parity_first_bits;
  int parity_first_type; // 0 - ODD, 1 - EVEN
  int parity_last_bits;
  int parity_last_type; // 0 - ODD, 1 - EVEN
  bool facility_enabled;
  int facility_bits;
  int start_bit;
  int facility_code;
  __u64 id_code;
};

/**
 * \brief Struct Wiegand Driver control
 */
typedef struct s_wieg_driver {
    const char * name;
    struct device * dev;
    int minor;
    int data0_irq;
    int data1_irq;
    spinlock_t spinlock;
    struct s_wieg_info info;
    struct s_wieg_sysfs sysfs;
} s_wieg_driver_t;

enum _e_wieg_status {
    _E_WS_NONE,
    _E_WS_CARD_AVAILABLE,
    _E_WS_LEFT_PARITY_FAILED,
    _E_WS_RIGHT_PARITY_FAILED,
    _E_WS_INVALID_BITS,
    _E_WS_INVALID_PARITY_SETUP,
};

/**
 * @brief Create data of the device driver
 * @param pdev      : platform device driver pointer
 * @param drv_class : class driver
 * @return 0 - success, < 0 - failed
 */
int wiegand_reader_create(struct platform_device *pdev, struct class * drv_class);

/**
 * \brief Remove data of the device driver
 * @param pdev  : plataform device driver
 */
void wiegand_reader_destroy(struct platform_device *pdev);

/**
 * \brief Initializes SYSFS data Wiegand Reader
 * @param wieg  : Wiegand Device Driver Created
 * @return < 0 --> failed, > 0 - success
 */
int wieg_setup_initializes(struct s_wieg_driver * wieg);


/**
 * @brief remove SYSFS
 * @param wieg  : Wiegand device driver created
 */
void wieg_setup_remove(struct s_wieg_driver * wieg);


/**
 * @brief perform card reader
 * @param wieg  : Wiegand device driver created
 */
void wieg_card_reader(struct s_wieg_driver * wieg);

/**
 * @}
 */

#endif /* WIEGAND_READER_H_ */
