/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
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
 * File: $Id: mbfuncinput_m.c, v 1.60 2013/10/12 14:23:40 Armink Add Master Functions  Exp $
 */

#include <mb.h>
/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_REQ_READ_ADDR_OFF            (MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_READ_REGCNT_OFF          (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_READ_SIZE                (4)
#define MB_PDU_FUNC_READ_BYTECNT_OFF        (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_VALUES_OFF         (MB_PDU_DATA_OFF + 1)
#define MB_PDU_FUNC_READ_SIZE_MIN           (1)

#define MB_PDU_FUNC_READ_RSP_BYTECNT_OFF    (MB_PDU_DATA_OFF)

/* ----------------------- Static functions ---------------------------------*/
mb_exception_enum    prveMBError2Exception(mb_err_enum eErrorCode);

/* ----------------------- Start implementation -----------------------------*/
#if (MB_RTU_ENABLED > 0 || MB_ASCII_ENABLED > 0) && MB_MASTER >0
#if MB_FUNC_READ_INPUT_ENABLED > 0

/**
 * This function will request read input register.
 *
 * @param ucSndAddr salve address
 * @param usRegAddr register start address
 * @param reg_num register total number
 * @param lTimeOut timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum
eMBMasterReqReadInputRegister(mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr, USHORT reg_num, LONG lTimeOut)
{
    UCHAR                 *ucMBFrame;
//    mb_err_enum    eErrStatus = MB_ENOERR;

    if (ucSndAddr > MB_ADDRESS_MAX)
    {
        return MB_EINVAL;
    }
    if (inst->master_is_busy)
    {
        return MB_EBUSY;
    }
    //else if (xMBMasterRunResTake(lTimeOut) == FALSE) eErrStatus = MB_EBUSY; //FIXME
    inst->trmt->get_tx_frm(inst-> transport, &ucMBFrame);
    inst->master_dst_addr = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_READ_INPUT_REGISTER;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]       = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]   = usRegAddr;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF]     = reg_num >> 8;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF + 1] = reg_num;
    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE); ///WTF?????

    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    //eErrStatus = eMBMasterWaitRequestFinish();
    return MB_EX_NONE;
}

mb_exception_enum
eMBMasterFuncReadInputRegister(mb_instance* inst, UCHAR * frame_ptr, USHORT * usLen)
{
    UCHAR          *ucMBFrame;
    USHORT          usRegAddress;
    USHORT          usRegCount;

    mb_exception_enum    eStatus = MB_EX_NONE;
    mb_err_enum    eRegStatus;

    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    if (inst->trmt->rq_is_broadcast(inst->transport))
    {
        eStatus = MB_EX_NONE;
    }
    else if (*usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN)
    {
        inst->trmt->get_tx_frm(inst->transport, &ucMBFrame);
        usRegAddress = (USHORT)(ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF] << 8);
        usRegAddress |= (USHORT)(ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]);
        usRegAddress++;

        usRegCount = (USHORT)(ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF] << 8);
        usRegCount |= (USHORT)(ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF + 1]);

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception.
         */
        if ((usRegCount >= 1) && (2 * usRegCount == frame_ptr[MB_PDU_FUNC_READ_BYTECNT_OFF]))
        {
            /* Make callback to fill the buffer. */
            eRegStatus = eMBMasterRegInputCB(inst, &frame_ptr[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usRegCount);
            /* If an error occured convert it into a Modbus exception. */
            if (eRegStatus != MB_ENOERR)
            {
                eStatus = prveMBError2Exception(eRegStatus);
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif
#endif
