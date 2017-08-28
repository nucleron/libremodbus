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
 * File: $Id: mbport.h, v 1.19 2010/06/06 13:54:40 wolti Exp $
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#include <mb_common.h>
PR_BEGIN_EXTERN_C

#include <mbconfig.h>

#if MB_TCP_ENABLED
typedef struct _mb_port_tcp mb_port_tcp_struct; //!< TCP port, child of #mb_port_base_struct
#endif

#if MB_RTU_ENABLED || MB_ASCII_ENABLED
typedef struct _mb_port_ser mb_port_ser_struct; //!< Serial port, child of #mb_port_base_struct
#endif

typedef BOOL (*mb_port_cb_fp)(void *arg);

typedef struct
{
    mb_port_cb_fp byte_rcvd;   //!<pxMBFrameCBByteReceived;
    mb_port_cb_fp tx_empty;    //!<pxMBFrameCBTransmitterEmpty;
    mb_port_cb_fp tmr_expired; //!<pxMBPortCBTimerExpired;
}
mb_port_cb_struct;                 //!<port callback table

typedef struct //_mb_port_base
{
    mb_port_cb_struct  *cb; //!<Port callbacks.
    void               *arg; //!<CB arg pointer.
}mb_port_base_struct; //!< Port base type

typedef struct //_mb_trans_base
{
    mb_port_base_struct * port_obj;
}mb_trans_base_struct; //!< Transport base type

#include <mbframe.h>
typedef struct
{
    mb_frm_close_fp     frm_close;//!<pvMBFrameCloseCur;
    mp_port_evt_post_fp evt_post; //!<pvPortEventPostCur;
    mb_port_evt_get_fp  evt_get;  //!<pvPortEventGetCur;
}
mb_port_mtab_struct; //!< Port methods tab;

typedef enum
{
    MB_PAR_NONE,                /*!< No parity. */
    MB_PAR_ODD,                 /*!< Odd parity. */
    MB_PAR_EVEN                 /*!< Even parity. */
} mb_port_ser_parity_enum;

/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
*/
/* ----------------------- Serial Supporting functions -----------------------------*/
BOOL mb_port_ser_evt_init(mb_port_ser_struct* inst                                     );
BOOL mb_port_ser_evt_post(mb_port_ser_struct* inst, mb_event_enum event                );
BOOL mb_port_ser_evt_get (mb_port_ser_struct* inst, void* caller, mb_event_enum * event); //FIXME
/* ----------------------- TCP Supporting functions -----------------------------*/
#if MB_TCP_ENABLED
BOOL mb_port_tcp_evt_init(mb_port_tcp_struct *inst                                     );
BOOL mb_port_tcp_evt_post(mb_port_tcp_struct *inst, mb_event_enum event                );
BOOL mb_port_tcp_evt_get (mb_port_tcp_struct *inst, void* caller, mb_event_enum * event); //FIXME
#endif
/* ----------------------- Serial port functions ----------------------------*/
BOOL mb_port_ser_init    (mb_port_ser_struct* inst, ULONG baud, UCHAR data_bits, mb_port_ser_parity_enum parity);
void mb_port_ser_close   (mb_port_ser_struct* inst                                                             );
void mb_port_ser_enable  (mb_port_ser_struct* inst, BOOL rx_enable, BOOL tx_enable                             );
BOOL mb_port_ser_get_byte(mb_port_ser_struct* inst, CHAR * byte_buf                                            );
BOOL mb_port_ser_put_byte(mb_port_ser_struct* inst, CHAR byte_va0l                                              );
/* ----------------------- Timers functions ---------------------------------*/
BOOL mb_port_ser_tmr_init   (mb_port_ser_struct* inst, USHORT timeout_50us);
void mb_port_ser_tmr_close  (mb_port_ser_struct* inst                     );
void mb_port_ser_tmr_enable (mb_port_ser_struct* inst                     );
void mb_port_ser_tmr_disable(mb_port_ser_struct* inst                     );
void mb_port_ser_tmr_delay  (mb_port_ser_struct* inst, USHORT timeout_ms  );

/*-------------------------TCP Timers functions ---------------------------------*/
#if MB_TCP_ENABLED > 0
BOOL mb_port_tcp_tmr_init   (mb_port_tcp_struct *inst, USHORT timeout_50us);
void mb_port_tcp_tmr_close  (mb_port_tcp_struct *inst                     );
void mb_port_tcp_tmr_enable (mb_port_tcp_struct *inst                     );
void mb_port_tcp_tmr_disable(mb_port_tcp_struct *inst                     );
void mb_port_tcp_tmr_delay  (mb_port_tcp_struct *inst, USHORT timeout_ms  );
#endif
/* ----------------------- Callback for the protocol stack ------------------*/

/*!
 * \brief Callback function for the porting layer when a new byte is
 *   available.
 *
 * Depending upon the mode this callback function is used by the RTU or
 * ASCII transmission layers. In any case a call to mb_port_ser_get_byte()
 * must immediately return a new character.
 *
 * \return <code>TRUE</code> if a event was posted to the queue because
 *   a new byte was received. The port implementation should wake up the
 *   tasks which are currently blocked on the eventqueue.
 */
/* ----------------------- TCP port functions -------------------------------*/
#if MB_TCP_ENABLED > 0
BOOL mb_port_tcp_init        (mb_port_tcp_struct *inst, USHORT tcp_port_num, SOCKADDR_IN hostaddr, BOOL is_master);
void mb_port_tcp_close       (mb_port_tcp_struct *inst                                                           );
void mb_port_tcp_disable     (mb_port_tcp_struct *inst                                                           );
BOOL mb_port_tcp_get_rq      (mb_port_tcp_struct *inst, UCHAR **frame_ptr_buf, USHORT *len_buf                  );
BOOL mb_port_tcp_snd_response(mb_port_tcp_struct *inst, const UCHAR *frame_ptr, USHORT len                       );
#endif
#if MB_MASTER >0
INLINE void mb_port_ser_tmr_convert_delay_enable  (mb_port_ser_struct* inst);
INLINE void mb_port_ser_tmr_respond_timeout_enable(mb_port_ser_struct* inst);
#endif

PR_END_EXTERN_C
#endif
