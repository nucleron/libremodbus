/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2016, 2017 Nucleron R&D LLC <main@nucleron.ru>
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
mb_exception_enum    mb_error_to_exception(mb_err_enum error_code);

/* ----------------------- Start implementation -----------------------------*/
#if MB_MASTER > 0
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0

/**
 * This function will request write holding register.
 *
 * @param snd_addr salve address
 * @param reg_addr register start address
 * @param reg_data register data to be written
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum  mb_mstr_rq_write_holding_reg(mb_inst_struct *inst, UCHAR snd_addr, USHORT reg_addr, USHORT reg_data, LONG timeout)
{
    UCHAR                 *mb_frame_ptr;

    if (snd_addr > MB_ADDRESS_MAX)
    {
        return MB_EINVAL;
    }
    if (inst->master_is_busy)
    {
        return MB_EBUSY;
    }
    //else if (xMBMasterRunResTake(timeout) == FALSE) eErrStatus = MB_EBUSY; //FIXME too
    inst->trmt->get_tx_frm(inst-> transport, &mb_frame_ptr);
    inst->master_dst_addr = snd_addr;
    mb_frame_ptr[MB_PDU_FUNC_OFF]                = MB_FUNC_WRITE_REGISTER;
    mb_frame_ptr[MB_PDU_REQ_WRITE_ADDR_OFF]      = reg_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_ADDR_OFF + 1]  = reg_addr;
    mb_frame_ptr[MB_PDU_REQ_WRITE_VALUE_OFF]     = reg_data >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_VALUE_OFF + 1] = reg_data ;
    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_SIZE);

    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    return MB_EX_NONE;
}

mb_exception_enum  mb_mstr_fn_write_holding_reg(mb_inst_struct *inst,  UCHAR *frame_ptr, USHORT *len_buf)
{
    mb_exception_enum    status = MB_EX_NONE;

    (void)inst;
    (void)frame_ptr;

    if (*len_buf == (MB_PDU_SIZE_MIN + MB_PDU_FUNC_WRITE_SIZE))
    {
        status = MB_EX_NONE;
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        status = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return status;
}
#endif

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0

/**
 * This function will request write multiple holding register.
 *
 * @param snd_addr salve address
 * @param reg_addr register start address
 * @param reg_num register total number
 * @param data_ptr data to be written
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum  mb_mstr_rq_write_multi_holding_reg(mb_inst_struct *inst, UCHAR snd_addr, USHORT reg_addr, USHORT reg_num, USHORT * data_ptr, LONG timeout)
{
    UCHAR                 *mb_frame_ptr;
    USHORT                 reg_idx = 0;

    if (snd_addr > MB_ADDRESS_MAX)
    {
        return MB_EINVAL;
    }
    if (inst->master_is_busy)
    {
        return MB_EBUSY;
    }
    inst->trmt->get_tx_frm(inst-> transport, &mb_frame_ptr);
    inst->master_dst_addr = snd_addr;
    mb_frame_ptr[MB_PDU_FUNC_OFF]                     = MB_FUNC_WRITE_MULTIPLE_REGISTERS;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_ADDR_OFF]       = reg_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1]   = reg_addr;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF]     = reg_num >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF + 1] = reg_num ;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF]    = reg_num * 2;
    mb_frame_ptr += MB_PDU_REQ_WRITE_MUL_VALUES_OFF;
    while(reg_num > reg_idx)
    {
        *mb_frame_ptr++ = data_ptr[reg_idx] >> 8;
        *mb_frame_ptr++ = data_ptr[reg_idx++] ;
    }
    *inst->pdu_snd_len = (MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_MUL_SIZE_MIN + 2*reg_num);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    return MB_EX_NONE;
}

mb_exception_enum  mb_mstr_fn_write_multi_holding_reg(mb_inst_struct *inst,  UCHAR *frame_ptr, USHORT *len_buf)
{

    mb_exception_enum    status = MB_EX_NONE;

    (void)frame_ptr;

    /* If this request is broadcast, the *len_buf is not need check. */
    if ((*len_buf == MB_PDU_SIZE_MIN + MB_PDU_FUNC_WRITE_MUL_SIZE) || inst->trmt->rq_is_broadcast(inst->transport))
    {
        status = MB_EX_NONE;
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        status = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return status;
}
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0

/**
 * This function will request read holding register.
 *
 * @param snd_addr salve address
 * @param reg_addr register start address
 * @param reg_num register total number
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum  mb_mstr_rq_read_holding_reg(mb_inst_struct *inst,  UCHAR snd_addr, USHORT reg_addr, USHORT reg_num, LONG timeout)
{
    UCHAR                 *mb_frame_ptr;

    if (snd_addr > MB_ADDRESS_MAX)
    {
        return MB_EINVAL;
    }
    if (inst->master_is_busy)
    {
        return MB_EBUSY;
    }
    inst->trmt->get_tx_frm(inst->transport, &mb_frame_ptr);
    inst->master_dst_addr = snd_addr;
    mb_frame_ptr[MB_PDU_FUNC_OFF]                = MB_FUNC_READ_HOLDING_REGISTER;
    mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF]       = reg_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF + 1]   = reg_addr;
    mb_frame_ptr[MB_PDU_REQ_READ_REGCNT_OFF]     = reg_num >> 8;
    mb_frame_ptr[MB_PDU_REQ_READ_REGCNT_OFF + 1] = reg_num;

    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE);

    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    //eErrStatus = eMBMasterWaitRequestFinish();
    return MB_EX_NONE;
}

mb_exception_enum  mb_mstr_fn_read_holding_reg(mb_inst_struct *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    UCHAR          *mb_frame_ptr;
    USHORT          reg_addr;
    USHORT          reg_cnt;

    mb_exception_enum    status = MB_EX_NONE;
    mb_err_enum    reg_status;

    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    BOOL isBroadcast = FALSE;

    isBroadcast = inst->trmt->rq_is_broadcast(inst->transport);

    if (isBroadcast == TRUE)
    {
        status = MB_EX_NONE;
    }
    else if (*len_buf >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN)
    {
        inst->trmt->get_tx_frm(inst->transport, &mb_frame_ptr);

        reg_addr = (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF] << 8);
        reg_addr |= (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF + 1]);
        reg_addr++;

        reg_cnt = (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_REGCNT_OFF] << 8);
        reg_cnt |= (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_REGCNT_OFF + 1]);


        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception.
         */
        if ((reg_cnt >= 1) && (2 * reg_cnt == frame_ptr[MB_PDU_FUNC_READ_BYTECNT_OFF]))
        {
            /* Make callback to fill the buffer. */
            reg_status = mb_mstr_reg_holding_cb(inst, &frame_ptr[MB_PDU_FUNC_READ_VALUES_OFF], reg_addr, reg_cnt);
            /* If an error occured convert it into a Modbus exception. */
            if (reg_status != MB_ENOERR)
            {
                status = mb_error_to_exception(reg_status);
            }
        }
        else
        {
            status = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        status = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return status;
}

#endif

#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0

/**
 * This function will request read and write holding register.
 *
 * @param snd_addr salve address
 * @param rd_reg_addr read register start address
 * @param rd_reg_num read register total number
 * @param data_ptr data to be written
 * @param wr_reg_addr write register start address
 * @param wr_reg_num write register total number
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum  mb_mstr_rq_rw_multi_holding_reg(mb_inst_struct *inst, UCHAR snd_addr, USHORT rd_reg_addr, USHORT rd_reg_num, USHORT * data_ptr, USHORT wr_reg_addr, USHORT wr_reg_num, LONG timeout)
{
    UCHAR                 *mb_frame_ptr;
    USHORT                 reg_idx = 0;
//    mb_err_enum    eErrStatus = MB_ENOERR;

    if (snd_addr > MB_ADDRESS_MAX)
    {
        return MB_EINVAL;
    }
    if (inst->master_is_busy)
    {
        return MB_EBUSY;
    }
    //else if (xMBMasterRunResTake(timeout) == FALSE) eErrStatus = MB_EBUSY; //FIXME
    inst->trmt->get_tx_frm(inst->transport, &mb_frame_ptr);
    inst->master_dst_addr = snd_addr;
    mb_frame_ptr[MB_PDU_FUNC_OFF]                           = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_ADDR_OFF]        = rd_reg_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_ADDR_OFF + 1]    = rd_reg_addr;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF]      = rd_reg_num >> 8;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF + 1]  = rd_reg_num ;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF]       = wr_reg_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF + 1]   = wr_reg_addr;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF]     = wr_reg_num >> 8;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF + 1] = wr_reg_num ;
    mb_frame_ptr[MB_PDU_REQ_READWRITE_WRITE_BYTECNT_OFF]    = wr_reg_num * 2;
    mb_frame_ptr += MB_PDU_REQ_READWRITE_WRITE_VALUES_OFF;
    while(wr_reg_num > reg_idx)
    {
        *mb_frame_ptr++ = data_ptr[reg_idx] >> 8;
        *mb_frame_ptr++ = data_ptr[reg_idx++] ;
    }
    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_READWRITE_SIZE_MIN + 2*wr_reg_num);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);
    return MB_EX_NONE;
}

mb_exception_enum  mb_mstr_fn_rw_multi_holding_regs(mb_inst_struct *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    USHORT          reg_rd_addr;
    USHORT          reg_rd_cnt;
    UCHAR          *mb_frame_ptr;

    mb_exception_enum    status = MB_EX_NONE;
    mb_err_enum    reg_status;

    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    if (inst->trmt->rq_is_broadcast(inst->transport))
    {
        status = MB_EX_NONE;
    }
    else if (*len_buf >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READWRITE_SIZE_MIN)
    {
        inst->trmt->get_tx_frm(inst->transport, &mb_frame_ptr);
        reg_rd_addr = (USHORT)(mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_ADDR_OFF] << 8U);
        reg_rd_addr |= (USHORT)(mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_ADDR_OFF + 1]);
        reg_rd_addr++;

        reg_rd_cnt = (USHORT)(mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF] << 8U);
        reg_rd_cnt |= (USHORT)(mb_frame_ptr[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF + 1]);

        if ((2 * reg_rd_cnt) == frame_ptr[MB_PDU_FUNC_READWRITE_READ_BYTECNT_OFF])
        {
            reg_status = mb_mstr_reg_holding_cb(inst, &frame_ptr[MB_PDU_FUNC_READWRITE_READ_VALUES_OFF], reg_rd_addr, reg_rd_cnt);
            if (reg_status != MB_ENOERR)
            {
                status = mb_error_to_exception(reg_status);
            }
        }
        else
        {
            status = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    return status;
}

#endif
#endif

