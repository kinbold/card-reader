/**
 ******************************************************************************
 * @file    wieg-readear_errors.h
 * @author  Dimitri Marques - dimitri.silva@tsi.com.br
 * @version V0.0.0
 * @date    12/03/2015
 * @brief
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

#ifndef CARD_READER_ERRORS_H_
#define CARD_READER_ERRORS_H_

//! Usar este define para retornar o erro no device driver
#define MAG_ERR(X)        -(X)

/**
 * \brief Código de erros do device driver do leitor magnético
 */
enum e_magreader_errors {
  /** 0000 */_E_WIEGERR_SUCCESS,              //!< Sucesso no comando
  /** 0001 */_E_WIEGERR_FAILED,               //!< Ocorreu alguma falha desconhecida
  /** 0002 */_E_WIEGERR_INVALID_SIZE,         //!< Tamanho de mensagem inválido
  /** 0003 */_E_WIEGERR_COPY_FROM_USER_FAILED,//!< Ocorreu falhas ao copiar dados do usuário para o driver
  /** 0004 */_E_WIEGERR_COPY_TO_USER_FAILED,  //!< Ocorreu falha ao copiar dados do driver para o usuário
  /** 0005 */_E_WIEGERR_INVALID_COMMAND,      //!< IOCTL Comando inválido
};

#endif /* CARD_READER_ERRORS_H_ */
