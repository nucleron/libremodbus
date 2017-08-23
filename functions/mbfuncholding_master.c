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
 * File: $Id: mbfuncholding_m.c, v 1.60 2013/09/02 14:13:40 Armink Add Master Functions  Exp $
 */

#include <mb.h>
/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_REQ_READ_ADDR_OFF                (MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_READ_REGCNT_OFF              (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_READ_SIZE                    (4)
#define MB_PDU_FUNC_READ_REGCNT_MAX             (0x007D)
#define MB_PDU_FUNC_READ_BYTECNT_OFF            (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_VALUES_OFF             (MB_PDU_DATA_OFF + 1)
#define MB_PDU_FUNC_READ_SIZE_MIN               (1)

#define MB_PDU_REQ_WRITE_ADDR_OFF               (MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_WRITE_VALUE_OFF              (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_WRITE_SIZE                   (4)
#define MB_PDU_FUNC_WRITE_ADDR_OFF              (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_VALUE_OFF             (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_SIZE                  (4)

#define MB_PDU_REQ_WRITE_MUL_ADDR_OFF           (MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_WRITE_MUL_REGCNT_OFF         (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF        (MB_PDU_DATA_OFF + 4)
#define MB_PDU_REQ_WRITE_MUL_VALUES_OFF         (MB_PDU_DATA_OFF + 5)
#define MB_PDU_REQ_WRITE_MUL_SIZE_MIN           (5)
#define MB_PDU_REQ_WRITE_MUL_REGCNT_MAX         (0x0078)
#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF          (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF        (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_MUL_SIZE              (4)

#define MB_PDU_REQ_READWRITE_READ_ADDR_OFF      (MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_READWRITE_READ_REGCNT_OFF    (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF     (MB_PDU_DATA_OFF + 4)
#define MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF   (MB_PDU_DATA_OFF + 6)
#define MB_PDU_REQ_READWRITE_WRITE_BYTECNT_OFF  (MB_PDU_DATA_OFF + 8)
#define MB_PDU_REQ_READWRITE_WRITE_VALUES_OFF   (MB_PDU_DATA_OFF + 9)
#define MB_PDU_REQ_READWRITE_SIZE_MIN           (9)
#define MB_PDU_FUNC_READWRITE_READ_BYTECNT_OFF  (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READWRITE_READ_VALUES_OFF   (MB_PDU_DATA_OFF + 1)
#define MB_PDU_FUNC_READWRITE_SIZE_MIN          (1)

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception(mb_err_enum eErrorCode);

/* ----------------------- Start implementation -----------------------------*/
#if MB_MASTER > 0
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0

/**
 * This function will request write holding register.
 *
 * @param ucSndAddr salve address
 * @param usRegAddr register start address
 * @param usRegData register data to be written
 * @param lTimeOut timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum
eMBMasterReqWriteHoldingRegister(mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr, USHORT usRegData, LONG lTimeOut)
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
    //else if (xMBMasterRunResTake(lTimeOut) == FALSE) eErrStatus = MB_EBUSY; //FIXME too
    inst->trmt->get_tx_frm(inst-> transport, &ucMBFrame);
    inst->master_dst_addr = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_WRITE_REGISTER;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF]      = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF + 1]  = usRegAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF]     = usRegData >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF + 1] = usRegData ;
    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_SIZE);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    //eErrStatus = eMBMasterWaitRequestFinish();
    return MB_EX_NONE;
}

eMBException
eMBMasterFuncWriteHoldingRegister(mb_instance* inst,  UCHAR * pucFrame, USHORT * usLen)
{
//    USHORT          usRegAddress;
    eMBException    eStatus = MB_EX_NONE;
//    mb_err_enum    eRegStatus;

    if (*usLen == (MB_PDU_SIZE_MIN + MB_PDU_FUNC_WRITE_SIZE))
    {
//        usRegAddress = (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
//        usRegAddress |= (USHORT)(pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);
//        usRegAddress++;
//
//        /* Make callback to update the value. */
//        eRegStatus = eMBMasterRegHoldingCB(inst, &pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF],
//                                      usRegAddress, 1, MB_REG_WRITE);
//
//        /* If an error occured convert it into a Modbus exception. */
//        if (eRegStatus != MB_ENOERR)
//        {
//            eStatus = prveMBError2Exception(eRegStatus);
//        }
        eStatus = MB_EX_NONE;
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0

/**
 * This function will request write multiple holding register.
 *
 * @param ucSndAddr salve address
 * @param usRegAddr register start address
 * @param reg_num register total number
 * @param pusDataBuffer data to be written
 * @param lTimeOut timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum
eMBMasterReqWriteMultipleHoldingRegister(mb_instance* inst, UCHAR ucSndAddr,
        USHORT usRegAddr, USHORT reg_num, USHORT * pusDataBuffer, LONG lTimeOut)
{
    UCHAR                 *ucMBFrame;
    USHORT                 usRegIndex = 0;
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
    ucMBFrame[MB_PDU_FUNC_OFF]                     = MB_FUNC_WRITE_MULTIPLE_REGISTERS;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF]       = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1]   = usRegAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF]     = reg_num >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF + 1] = reg_num ;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF]    = reg_num * 2;
    ucMBFrame += MB_PDU_REQ_WRITE_MUL_VALUES_OFF;
    while(reg_num > usRegIndex)
    {
        *ucMBFrame++ = pusDataBuffer[usRegIndex] >> 8;
        *ucMBFrame++ = pusDataBuffer[usRegIndex++] ;
    }
    *inst->pdu_snd_len = (MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_MUL_SIZE_MIN + 2*reg_num);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    //eErrStatus = eMBMasterWaitRequestFinish();
    return MB_EX_NONE;
}

eMBException
eMBMasterFuncWriteMultipleHoldingRegister(mb_instance* inst,  UCHAR * pucFrame, USHORT * usLen)
{
//    UCHAR          *ucMBFrame;
//    USHORT          usRegAddress;
//    USHORT          usRegCount;
//    UCHAR           ucRegByteCount;

    eMBException    eStatus = MB_EX_NONE;
//    mb_err_enum    eRegStatus;

    /* If this request is broadcast, the *usLen is not need check. */
    if ((*usLen == MB_PDU_SIZE_MIN + MB_PDU_FUNC_WRITE_MUL_SIZE) || inst->trmt->rq_is_broadcast(inst->transport))
    {
//		  inst->trmt->get_tx_frm(inst->transport, &ucMBFrame);
//        usRegAddress = (USHORT)(ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF] << 8);
//        usRegAddress |= (USHORT)(ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1]);
//        usRegAddress++;
//
//        usRegCount = (USHORT)(ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF] << 8);
//        usRegCount |= (USHORT)(ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF + 1]);
//
//        ucRegByteCount = ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF];
//
//        if (ucRegByteCount == 2 * usRegCount)
//        {
//            /* Make callback to update the register values. */
//            eRegStatus =
//                eMBMasterRegHoldingCB(inst, &ucMBFrame[MB_PDU_REQ_WRITE_MUL_VALUES_OFF],
//                                 usRegAddress, usRegCount, MB_REG_WRITE);
//
//            /* If an error occured convert it into a Modbus exception. */
//            if (eRegStatus != MB_ENOERR)
//            {
//                eStatus = prveMBError2Exception(eRegStatus);
//            }
//        }
//        else
//        {
//            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
//        }
        eStatus = MB_EX_NONE;
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0

/**
 * This function will request read holding register.
 *
 * @param ucSndAddr salve address
 * @param usRegAddr register start address
 * @param reg_num register total number
 * @param lTimeOut timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum
eMBMasterReqReadHoldingRegister(mb_instance* inst,  UCHAR ucSndAddr, USHORT usRegAddr, USHORT reg_num, LONG lTimeOut)
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
    inst->trmt->get_tx_frm(inst->transport, &ucMBFrame);
    inst->master_dst_addr = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_READ_HOLDING_REGISTER;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]       = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]   = usRegAddr;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF]     = reg_num >> 8;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF + 1] = reg_num;

    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE);

    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    //eErrStatus = eMBMasterWaitRequestFinish();
    return MB_EX_NONE;
}

eMBException
eMBMasterFuncReadHoldingRegister(mb_instance* inst, UCHAR * pucFrame, USHORT * usLen)
{
    UCHAR          *ucMBFrame;
    USHORT          usRegAddress;
    USHORT          usRegCount;

    eMBException    eStatus = MB_EX_NONE;
    mb_err_enum    eRegStatus;

    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    BOOL isBroadcast = FALSE;

    if (inst->cur_mode == MB_RTU)
    {
#if MB_TCP_ENABLED
        isBroadcast = inst->trmt->rq_is_broadcast(inst->transport);
#endif // MB_TCP_ENABLED
    }
    else if (inst->cur_mode == MB_ASCII)
    {
#if MB_ASCII_ENABLED
        isBroadcast = xMBASCIIMasterRequestIsBroadcast((mb_ascii_tr *)inst->transport);
#endif // MB_ASCII_ENABLED
    }
    else //TCP
    {
        isBroadcast = FALSE; //No broadcasts in TCP mode
    }

    if (isBroadcast == TRUE)
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
        if ((usRegCount >= 1) && (2 * usRegCount == pucFrame[MB_PDU_FUNC_READ_BYTECNT_OFF]))
        {
            /* Make callback to fill the buffer. */
            eRegStatus = eMBMasterRegHoldingCB(inst, &pucFrame[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usRegCount);
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

#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0

/**
 * This function will request read and write holding register.
 *
 * @param ucSndAddr salve address
 * @param usReadRegAddr read register start address
 * @param usNReadRegs read register total number
 * @param pusDataBuffer data to be written
 * @param usWriteRegAddr write register start address
 * @param usNWriteRegs write register total number
 * @param lTimeOut timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum
eMBMasterReqReadWriteMultipleHoldingRegister(mb_instance* inst, UCHAR ucSndAddr,
        USHORT usReadRegAddr, USHORT usNReadRegs, USHORT * pusDataBuffer,
        USHORT usWriteRegAddr, USHORT usNWriteRegs, LONG lTimeOut)
{
    UCHAR                 *ucMBFrame;
    USHORT                 usRegIndex = 0;
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
    inst->trmt->get_tx_frm(inst->transport, &ucMBFrame);
    inst->master_dst_addr = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                           = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF]        = usReadRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF + 1]    = usReadRegAddr;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF]      = usNReadRegs >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF + 1]  = usNReadRegs ;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF]       = usWriteRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF + 1]   = usWriteRegAddr;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF]     = usNWriteRegs >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF + 1] = usNWriteRegs ;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_BYTECNT_OFF]    = usNWriteRegs * 2;
    ucMBFrame += MB_PDU_REQ_READWRITE_WRITE_VALUES_OFF;
    while(usNWriteRegs > usRegIndex)
    {
        *ucMBFrame++ = pusDataBuffer[usRegIndex] >> 8;
        *ucMBFrame++ = pusDataBuffer[usRegIndex++] ;
    }
    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_READWRITE_SIZE_MIN + 2*usNWriteRegs);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    //eErrStatus = eMBMasterWaitRequestFinish();
    return MB_EX_NONE;
}

eMBException
eMBMasterFuncReadWriteMultipleHoldingRegister(mb_instance* inst, UCHAR * pucFrame, USHORT * usLen)
{
    USHORT          usRegReadAddress;
    USHORT          usRegReadCount;
//    USHORT          usRegWriteAddress;
//    USHORT          usRegWriteCount;
    UCHAR          *ucMBFrame;

    eMBException    eStatus = MB_EX_NONE;
    mb_err_enum    eRegStatus;

    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    if (inst->trmt->rq_is_broadcast(inst->transport))
    {
        eStatus = MB_EX_NONE;
    }
    else if (*usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READWRITE_SIZE_MIN)
    {
        inst->trmt->get_tx_frm(inst->transport, &ucMBFrame);
        usRegReadAddress = (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF] << 8U);
        usRegReadAddress |= (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF + 1]);
        usRegReadAddress++;

        usRegReadCount = (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF] << 8U);
        usRegReadCount |= (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF + 1]);

//        usRegWriteAddress = (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF] << 8U);
//        usRegWriteAddress |= (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF + 1]);
//        usRegWriteAddress++;
//
//        usRegWriteCount = (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF] << 8U);
//        usRegWriteCount |= (USHORT)(ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF + 1]);

        if ((2 * usRegReadCount) == pucFrame[MB_PDU_FUNC_READWRITE_READ_BYTECNT_OFF])
        {
//            /* Make callback to update the register values. */
//            eRegStatus = eMBMasterRegHoldingCB(inst, &ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_VALUES_OFF],
//                                           usRegWriteAddress, usRegWriteCount, MB_REG_WRITE);
//
//            if (eRegStatus == MB_ENOERR)
//            {
//                /* Make the read callback. */
//				eRegStatus = eMBMasterRegHoldingCB(inst, &pucFrame[MB_PDU_FUNC_READWRITE_READ_VALUES_OFF],
//						                      usRegReadAddress, usRegReadCount, MB_REG_READ);
//            }
            eRegStatus = eMBMasterRegHoldingCB(inst, &pucFrame[MB_PDU_FUNC_READWRITE_READ_VALUES_OFF],
                                               usRegReadAddress, usRegReadCount);
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
    return eStatus;
}

#endif
#endif

