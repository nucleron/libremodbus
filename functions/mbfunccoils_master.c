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
 * File: $Id: mbfunccoils_m.c, v 1.60 2013/10/12 15:10:12 Armink Add Master Functions
 */
#include <mb.h>

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_REQ_READ_ADDR_OFF            (MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_READ_COILCNT_OFF         (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_READ_SIZE                (4)
#define MB_PDU_FUNC_READ_COILCNT_OFF        (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_VALUES_OFF         (MB_PDU_DATA_OFF + 1)
#define MB_PDU_FUNC_READ_SIZE_MIN           (1)

#define MB_PDU_REQ_WRITE_ADDR_OFF           (MB_PDU_DATA_OFF)
#define MB_PDU_REQ_WRITE_VALUE_OFF          (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_WRITE_SIZE               (4)
#define MB_PDU_FUNC_WRITE_ADDR_OFF          (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_WRITE_VALUE_OFF         (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_SIZE              (4)

#define MB_PDU_REQ_WRITE_MUL_ADDR_OFF       (MB_PDU_DATA_OFF)
#define MB_PDU_REQ_WRITE_MUL_COILCNT_OFF    (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF    (MB_PDU_DATA_OFF + 4)
#define MB_PDU_REQ_WRITE_MUL_VALUES_OFF     (MB_PDU_DATA_OFF + 5)
#define MB_PDU_REQ_WRITE_MUL_SIZE_MIN       (5)
#define MB_PDU_REQ_WRITE_MUL_COILCNT_MAX    (0x07B0)
#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF      (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF   (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_MUL_SIZE          (5)

/* ----------------------- Static functions ---------------------------------*/
mb_exception_enum  mb_error_to_exception(mb_err_enum error_code);

/* ----------------------- Start implementation -----------------------------*/
#if MB_MASTER > 0
#if MB_FUNC_READ_COILS_ENABLED > 0

/**
 * This function will request read coil.
 *
 * @param snd_addr salve address
 * @param coil_addr coil start address
 * @param coil_num coil total number
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum  mb_mstr_rq_read_coils(mb_inst_struct *inst, UCHAR snd_addr, USHORT coil_addr, USHORT coil_num)
{
    UCHAR                 *mb_frame_ptr;

    if (snd_addr > MB_ADDRESS_MAX)
    {
        return  MB_EINVAL;
    }
    if (inst->master_is_busy)
    {
        return MB_EBUSY;
    }
    inst->trmt->get_tx_frm(inst->transport, &mb_frame_ptr);
    inst->master_dst_addr = snd_addr;

    mb_frame_ptr[MB_PDU_FUNC_OFF]                 = MB_FUNC_READ_COILS;

    inst->master_el_addr                          = coil_addr;
    mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF]        = coil_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF + 1]    = coil_addr;

    inst->master_el_cnt                           = coil_num;
    mb_frame_ptr[MB_PDU_REQ_READ_COILCNT_OFF ]    = coil_num >> 8;
    mb_frame_ptr[MB_PDU_REQ_READ_COILCNT_OFF + 1] = coil_num;

    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE);

    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);

    return MB_ENOERR;
}

mb_exception_enum  mb_mstr_fn_read_coils(mb_inst_struct *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    //UCHAR          *mb_frame_ptr;
    //USHORT          reg_addr;
    //USHORT          coil_cnt;
    UCHAR           byte_cnt;

    mb_exception_enum    status = MB_EX_NONE;
    mb_err_enum    reg_status;

    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    if (inst->trmt->rq_is_broadcast(inst->transport))
    {
        status = MB_EX_NONE;
    }
    else if (*len_buf >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN)
    {
        USHORT coil_cnt;
        coil_cnt = inst->master_el_cnt;
//        inst->trmt->get_tx_frm(inst->transport, &mb_frame_ptr);
//        reg_addr = (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF] << 8);
//        reg_addr |= (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF + 1]);
//        reg_addr++;
//
//        coil_cnt = (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_COILCNT_OFF] << 8);
//        coil_cnt |= (USHORT)(mb_frame_ptr[MB_PDU_REQ_READ_COILCNT_OFF + 1]);


        /* Test if the quantity of coils is a multiple of 8. If not last
         * byte is only partially field with unused coils set to zero. */
        if ((coil_cnt & 0x0007) != 0)
        {
            byte_cnt = (UCHAR)(coil_cnt / 8 + 1);
        }
        else
        {
            byte_cnt = (UCHAR)(coil_cnt / 8);
        }

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception.
         */
        if ((coil_cnt >= 1) && (byte_cnt == frame_ptr[MB_PDU_FUNC_READ_COILCNT_OFF]))
        {
            /* Make callback to fill the buffer. */
            reg_status = mb_mstr_reg_coils_cb(inst, &frame_ptr[MB_PDU_FUNC_READ_VALUES_OFF], inst->master_el_addr + 1, coil_cnt);

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
        /* Can't be a valid read coil register request because the length
         * is incorrect. */
        status = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return status;
}
#endif

#if MB_FUNC_WRITE_COIL_ENABLED > 0

/**
 * This function will request write one coil.
 *
 * @param snd_addr salve address
 * @param coil_addr coil start address
 * @param coil_data data to be written
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 *
 * @see mb_mstr_rq_write_multi_coils
 */
mb_err_enum  mb_mstr_rq_write_coil(mb_inst_struct *inst, UCHAR snd_addr, USHORT coil_addr, USHORT coil_data)
{
    UCHAR *mb_frame_ptr;

    if (snd_addr > MB_ADDRESS_MAX)
    {
        return MB_EINVAL;
    }
    if ((coil_data != 0xFF00) && (coil_data != 0x0000))
    {
        return MB_EINVAL;
    }
    if (inst->master_is_busy)
    {
        return MB_EBUSY;
    }
    inst->trmt->get_tx_frm(inst->transport, &mb_frame_ptr);
    inst->master_dst_addr = snd_addr;

    mb_frame_ptr[MB_PDU_FUNC_OFF]                = MB_FUNC_WRITE_SINGLE_COIL;

    inst->master_el_addr                         = coil_addr;
    mb_frame_ptr[MB_PDU_REQ_WRITE_ADDR_OFF]      = coil_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_ADDR_OFF + 1]  = coil_addr;

    inst->master_el_cnt                          = 1;

    mb_frame_ptr[MB_PDU_REQ_WRITE_VALUE_OFF ]    = coil_data >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_VALUE_OFF + 1] = coil_data;

    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_SIZE);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);

    return MB_ENOERR;
}

mb_exception_enum  mb_mstr_fn_write_coil(mb_inst_struct *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    mb_exception_enum    status = MB_EX_NONE;
    (void)inst;
    (void)frame_ptr;

    if (*len_buf == (MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN))
    {
        status = MB_EX_NONE;
    }
    else
    {
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        status = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return status;
}

#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0

/**
 * This function will request write multiple coils.
 *
 * @param snd_addr salve address
 * @param coil_addr coil start address
 * @param coil_num coil total number
 * @param coil_data data to be written
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 *
 * @see mb_mstr_rq_write_coil
 */
mb_err_enum  mb_mstr_rq_write_multi_coils(mb_inst_struct *inst, UCHAR snd_addr, USHORT coil_addr, USHORT coil_num, UCHAR * data_ptr)
{
    UCHAR                 *mb_frame_ptr;
    USHORT                 reg_idx = 0;
    UCHAR                  byte_cnt;

    if (snd_addr > MB_ADDRESS_MAX)
    {
        return MB_EINVAL;
    }
    if (coil_num > MB_PDU_REQ_WRITE_MUL_COILCNT_MAX)
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
    mb_frame_ptr[MB_PDU_FUNC_OFF]                      = MB_FUNC_WRITE_MULTIPLE_COILS;

    inst->master_el_addr                               = coil_addr;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_ADDR_OFF]        = coil_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1]    = coil_addr;

    inst->master_el_cnt                                = coil_num;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_COILCNT_OFF]     = coil_num >> 8;
    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_COILCNT_OFF + 1] = coil_num ;

    if((coil_num & 0x0007) != 0)
    {
        byte_cnt = (UCHAR)(coil_num / 8 + 1);
    }
    else
    {
        byte_cnt = (UCHAR)(coil_num / 8);
    }

    mb_frame_ptr[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF]     = byte_cnt;
    mb_frame_ptr += MB_PDU_REQ_WRITE_MUL_VALUES_OFF;

    while(byte_cnt > reg_idx)
    {
        *mb_frame_ptr++ = data_ptr[reg_idx++];
    }

    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_MUL_SIZE_MIN + byte_cnt);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);

    return MB_ENOERR;
}

mb_exception_enum mb_mstr_fn_write_multi_coils(mb_inst_struct *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    mb_exception_enum    status = MB_EX_NONE;
    (void)frame_ptr;
    /* If this request is broadcast, the *len_buf is not need check. */
    if ((*len_buf == MB_PDU_FUNC_WRITE_MUL_SIZE) || inst->trmt->rq_is_broadcast(inst->transport))
    {
        status = MB_EX_NONE;
    }
    else
    {
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        status = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return status;
}

#endif
#endif
