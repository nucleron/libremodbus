/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
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
 * File: $Id: mbtcp.c,v 1.3 2006/12/07 22:10:34 wolti Exp $
 */

#include <mb.h>

///* ----------------------- System includes ----------------------------------*/
//#include "stdlib.h"
//#include "string.h"
//
///* ----------------------- Platform includes --------------------------------*/
//#include "tcp_port.h"
//
///* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
//#include "mbconfig.h"
//#include "mbtcp.h"
//#include "mbframe.h"
//#include "mbport.h"
//
//#include "tcp_multiport.h"

#if MB_TCP_ENABLED > 0

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- MBAP Header --------------------------------------*/
/*
 *
 * <------------------------ MODBUS TCP/IP ADU(1) ------------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+------------------------------------------+
 *  | TID | PID | Length | UID  |Code | Data                               |
 *  +-----------+---------------+------------------------------------------+
 *  |     |     |        |      |
 * (2)   (3)   (4)      (5)    (6)
 *
 * (2)  ... MB_TCP_TID          = 0 (Transaction Identifier - 2 Byte)
 * (3)  ... MB_TCP_PID          = 2 (Protocol Identifier - 2 Byte)
 * (4)  ... MB_TCP_LEN          = 4 (Number of bytes - 2 Byte)
 * (5)  ... MB_TCP_UID          = 6 (Unit Identifier - 1 Byte)
 * (6)  ... MB_TCP_FUNC         = 7 (Modbus Function Code)
 *
 * (1)  ... Modbus TCP/IP Application Data Unit
 * (1') ... Modbus Protocol Data Unit
 */

#define MB_TCP_TID          0
#define MB_TCP_PID          2
#define MB_TCP_LEN          4
#define MB_TCP_UID          6
#define MB_TCP_FUNC         7

#define MB_TCP_PROTOCOL_ID  0   /* 0 = Modbus Protocol */

#if MB_MULTIPORT > 0
#define usSendPDULength inst->usSendPDULength
#else

#endif


const mb_tr_mtab mb_tcp_mtab =
{
    .frm_start   = (mb_frm_start_fp)  mb_tcp_start,
    .frm_stop    = (mb_frm_stop_fp)   mb_tcp_stop,
    .frm_send    = (mb_frm_snd_fp)   mb_tcp_send,
    .frm_rcv     = (mb_frm_rcv_fp)mb_tcp_receive,

    .get_rx_frm      = (mb_get_rx_frm_fp)mb_tcp_get_rcv_buf,
    .get_tx_frm      = (mb_get_tx_frm_fp)mb_tcp_get_snd_buf
#   if MB_MASTER > 0
    , .rq_is_broadcast = (mb_mstr_rq_is_bcast_fp)mb_tcp_rq_is_bcast
#   endif //master
};
/* ----------------------- Start implementation -----------------------------*/
mb_err_enum
mb_tcp_init(mb_tcp_tr* inst, USHORT tcp_port_num, SOCKADDR_IN hostaddr, BOOL is_master)
{
    mb_err_enum    status = MB_ENOERR;

    if (xMBTCPPortInit(inst->base.port_obj, tcp_port_num,hostaddr, is_master) == FALSE)
    {
        status = MB_EPORTERR;
    }
    return status;
}

void
mb_tcp_start(mb_tcp_tr* inst)
{
}

void
mb_tcp_stop(mb_tcp_tr* inst)
{
    /* Make sure that no more clients are connected. */
    vMBTCPPortDisable(inst->base.port_obj);
}

mb_err_enum
mb_tcp_receive(mb_tcp_tr* inst, UCHAR * rcv_addr_buf, UCHAR ** frame_ptr_buf, USHORT * len_buf)
{
    mb_err_enum    status = MB_EIO;
    UCHAR          *pucMBTCPFrame;
    USHORT          len;
    USHORT          usPID;

    if (xMBTCPPortGetRequest(inst->base.port_obj, &pucMBTCPFrame, &len) != FALSE)
    {
        usPID = pucMBTCPFrame[MB_TCP_PID] << 8U;
        usPID |= pucMBTCPFrame[MB_TCP_PID + 1];

        if (usPID == MB_TCP_PROTOCOL_ID)
        {
            *frame_ptr_buf = &pucMBTCPFrame[MB_TCP_FUNC];
            *len_buf = len - MB_TCP_FUNC;
            status = MB_ENOERR;

            /* Modbus TCP does not use any addresses. Fake the source address such
             * that the processing part deals with this frame.
             */
            *rcv_addr_buf = MB_TCP_PSEUDO_ADDRESS;
        }
    }
    else
    {
        status = MB_EIO;
    }
    return status;
}

mb_err_enum
mb_tcp_send(mb_tcp_tr* inst, UCHAR _unused, const UCHAR * frame_ptr, USHORT len)
{
    mb_err_enum    status = MB_ENOERR;
    UCHAR          *pucMBTCPFrame = (UCHAR *) frame_ptr - MB_TCP_FUNC;
    USHORT          usTCPLength = len + MB_TCP_FUNC;

    /* The MBAP header is already initialized because the caller calls this
     * function with the buffer returned by the previous call. Therefore we
     * only have to update the length in the header. Note that the length
     * header includes the size of the Modbus PDU and the UID Byte. Therefore
     * the length is len plus one.
     */
    pucMBTCPFrame[MB_TCP_LEN] = (len + 1) >> 8U;
    pucMBTCPFrame[MB_TCP_LEN + 1] = (len + 1) & 0xFF;
    if (xMBTCPPortSendResponse(inst->base.port_obj, pucMBTCPFrame, usTCPLength) == FALSE)
    {
        status = MB_EIO;
    }
    return status;
}

#if MB_MASTER > 0

void vMBTCPMasterSetPDUSndLength( mb_tcp_tr* inst, USHORT snd_pdu_len)
{
    usSendPDULength = snd_pdu_len;
}

/* Get Modbus Master send PDU's buffer length.*/
USHORT usMBTCPMasterGetPDUSndLength( mb_tcp_tr* inst)
{
    return usSendPDULength;
}

void mb_tcp_get_snd_buf(mb_tcp_tr* inst, UCHAR ** frame_ptr_buf)
{
    *frame_ptr_buf = inst->tcp_port.aucTCPSndBuf+MB_TCP_FUNC;
}

void mb_tcp_get_rcv_buf(mb_tcp_tr* inst, UCHAR ** frame_ptr_buf)
{
    *frame_ptr_buf = inst->tcp_port.aucTCPRcvBuf+MB_TCP_FUNC;
}

BOOL mb_tcp_rq_is_bcast(mb_tcp_tr* inst)
{
    return FALSE; //no broadcasts on tcp
}
#endif

#endif
