/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2016, 2017 Nucleron R&D LLC <main@nucleron.ru>
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbascii.h, v 1.8 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_ASCII_H
#define _MB_ASCII_H

#include <mb_common.h>
PR_BEGIN_EXTERN_C

#include <mb_types.h>

/* ----------------------- Defines ------------------------------------------*/
#define MB_ASCII_DEFAULT_CR           '\r'    /*!< Default CR character for Modbus ASCII. */
#define MB_ASCII_DEFAULT_LF           '\n'    /*!< Default LF character for Modbus ASCII. */
#define MB_ASCII_SER_PDU_SIZE_MIN     3       /*!< Minimum size of a Modbus ASCII frame. */
#define MB_ASCII_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus ASCII frame. */
#define MB_ASCII_SER_PDU_SIZE_LRC     1       /*!< Size of LRC field in PDU. */
#define MB_ASCII_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_ASCII_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

typedef enum
{
    MB_ASCII_RX_STATE_IDLE,              /*!< Receiver is in idle state. */
	MB_ASCII_RX_STATE_RCV,               /*!< Frame is beeing received. */
	MB_ASCII_RX_STATE_WAIT_EOF           /*!< Wait for End of Frame. */
} mb_ascii_rx_state_enum;

typedef enum
{
	MB_ASCII_TX_STATE_IDLE,              /*!< Transmitter is in idle state. */
	MB_ASCII_TX_STATE_START,             /*!< Starting transmission (':' sent). */
	MB_ASCII_TX_STATE_DATA,              /*!< Sending of data (Address, Data, LRC). */
	MB_ASCII_TX_STATE_END,               /*!< End of transmission. */
	MB_ASCII_TX_STATE_NOTIFY,             /*!< Notify sender that the frame has been sent. */
	MB_ASCII_TX_STATE_XFWR
} mb_ascii_tx_state_enum;

typedef enum
{
    BYTE_HIGH_NIBBLE,           /*!< Character for high nibble of byte. */
    BYTE_LOW_NIBBLE             /*!< Character for low nibble of byte. */
} mb_ascii_byte_pos_enum;


typedef struct
{
    mb_trans_base_struct            base;

	volatile mb_ascii_tx_state_enum snd_state;
	volatile mb_ascii_rx_state_enum rcv_state;

	volatile UCHAR                  pdu_buf[MB_ASCII_SER_PDU_SIZE_MAX];
    volatile USHORT                 snd_pdu_len;

	volatile UCHAR                  *snd_buf_cur;
	volatile USHORT                  snd_buf_cnt;

	volatile USHORT                  rcv_buf_pos;
	volatile mb_ascii_byte_pos_enum  byte_pos;

	volatile UCHAR                  mb_lf_char;
	BOOL                            frame_is_broadcast;
	BOOL                            is_master;
	volatile mb_tmr_mode_enum       cur_tmr_mode;
}mb_ascii_tr_struct;

extern const mb_tr_mtab mb_ascii_mtab;

mb_err_enum mb_ascii_init            (mb_ascii_tr_struct* inst, BOOL is_master, UCHAR slv_addr, ULONG baud, mb_port_ser_parity_enum parity);
void        mb_ascii_start           (mb_ascii_tr_struct* inst                                                                            );
void        mb_ascii_stop            (mb_ascii_tr_struct* inst                                                                            );
mb_err_enum mb_ascii_receive         (mb_ascii_tr_struct* inst, UCHAR * rcv_addr_buf, UCHAR ** frame_ptr_buf, USHORT *len_buf            );
mb_err_enum mb_ascii_send            (mb_ascii_tr_struct* inst, UCHAR slv_addr, const UCHAR *frame_ptr, USHORT len                       );
BOOL        mb_ascii_rcv_fsm         (mb_ascii_tr_struct* inst                                                                            );
BOOL        mb_ascii_snd_fsm         (mb_ascii_tr_struct* inst                                                                            );
BOOL        mb_ascii_tmr_1s_expired  (mb_ascii_tr_struct* inst                                                                            );
//master
void        mb_ascii_get_snd_buf     (mb_ascii_tr_struct* inst, UCHAR ** frame_ptr_buf                                                    );
USHORT      mb_ascii_get_snd_len     (mb_ascii_tr_struct* inst                                                                            );
void        mb_ascii_set_snd_len     (mb_ascii_tr_struct* inst, USHORT snd_pdu_len                                                        );
void        mb_ascii_set_cur_tmr_mode(mb_ascii_tr_struct* inst, mb_tmr_mode_enum tmr_mode                                                 );
BOOL        mb_ascii_rq_is_bcast     (mb_ascii_tr_struct* inst                                                                            );

PR_END_EXTERN_C
#endif
