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
 * File: $Id: mbrtu.h, v 1.9 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_RTU_H
#define _MB_RTU_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Defines ------------------------------------------*/
#define MB_RTU_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_RTU_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_RTU_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_RTU_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_RTU_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    MB_RTU_RX_STATE_INIT,              /*!< Receiver is in initial state. */
    MB_RTU_RX_STATE_IDLE,              /*!< Receiver is in idle state. */
    MB_RTU_RX_STATE_RCV,               /*!< Frame is beeing received. */
    MB_RTU_RX_STATE_ERROR              /*!< If the frame is invalid. */
} mb_rtu_rcv_state_enum;

typedef enum
{
    MB_RTU_TX_STATE_IDLE,              /*!< Transmitter is in idle state. */
    MB_RTU_TX_STATE_XMIT,              /*!< Transmitter is in transfer state. */
    MB_RTU_TX_STATE_XFWR
} mb_rtu_snd_state_enum;

typedef struct
{
    mb_trans_base_struct           base;
    volatile mb_rtu_snd_state_enum snd_state;
    volatile mb_rtu_rcv_state_enum rcv_state;

    volatile UCHAR                 pdu_buf[MB_RTU_SER_PDU_SIZE_MAX];
    volatile USHORT                snd_pdu_len;

    volatile UCHAR                 *snd_buf_cur;
    volatile USHORT                 snd_buf_cnt;

    volatile USHORT                 rcv_buf_pos;

    BOOL                           is_master;
    BOOL                           frame_is_broadcast;
    volatile mb_tmr_mode_enum      cur_tmr_mode;
} mb_rtu_tr_struct;

extern const mb_tr_mtab mb_rtu_mtab;

mb_err_enum mb_rtu_init            (mb_rtu_tr_struct* inst, BOOL is_master, UCHAR slv_addr, ULONG baud, mb_port_ser_parity_enum parity);
void        mb_rtu_start           (mb_rtu_tr_struct* inst                                                                            );
void        mb_rtu_stop            (mb_rtu_tr_struct* inst                                                                            );
mb_err_enum mb_rtu_receive         (mb_rtu_tr_struct* inst, UCHAR * rcv_addr_buf, UCHAR ** frame_ptr_buf, USHORT *len_buf            );
mb_err_enum mb_rtu_send            (mb_rtu_tr_struct* inst, UCHAR slv_addr, const UCHAR *frame_ptr, USHORT len                       );
BOOL        mb_rtu_rcv_fsm         (mb_rtu_tr_struct* inst                                                                            );
BOOL        mb_rtu_snd_fsm         (mb_rtu_tr_struct* inst                                                                            );
BOOL        mb_rtu_tmr_35_expired  (mb_rtu_tr_struct* inst                                                                            );
void        mb_rtu_get_snd_buf     (mb_rtu_tr_struct* inst, UCHAR ** frame_ptr_buf                                                    );
USHORT      mb_rtu_get_snd_len     (mb_rtu_tr_struct* inst                                                                            );
void        mb_rtu_set_snd_len     (mb_rtu_tr_struct* inst, USHORT snd_pdu_len                                                        );
void        mb_rtu_set_cur_tmr_mode(mb_rtu_tr_struct* inst, mb_tmr_mode_enum tmr_mode                                                 );
BOOL        mb_rtu_rq_is_bcast     (mb_rtu_tr_struct* inst                                                                            );
#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
