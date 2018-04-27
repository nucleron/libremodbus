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
 * File: $Id: mbfuncdisc_m.c, v 1.60 2013/10/15 8:48:20 Armink Add Master Functions  Exp $
 */
#include <mb.h>
/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_REQ_READ_ADDR_OFF            (MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_READ_DISCCNT_OFF         (MB_PDU_DATA_OFF + 2)
#define MB_PDU_REQ_READ_SIZE                (4)
#define MB_PDU_FUNC_READ_DISCCNT_OFF        (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_VALUES_OFF         (MB_PDU_DATA_OFF + 1)
#define MB_PDU_FUNC_READ_SIZE_MIN           (1)

/* ----------------------- Static functions ---------------------------------*/
mb_exception_enum    mb_error_to_exception(mb_err_enum error_code);

/* ----------------------- Start implementation -----------------------------*/
#if MB_MASTER > 0
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0

/**
 * This function will request read discrete inputs.
 *
 * @param snd_addr salve address
 * @param discrete_addr discrete start address
 * @param discrete_num discrete total number
 * @param timeout timeout (-1 will waiting forever)
 *
 * @return error code
 */
mb_err_enum  mb_mstr_rq_read_discrete_inputs(mb_inst_struct *inst, UCHAR snd_addr, USHORT discrete_addr, USHORT discrete_num)
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
    inst->trmt->get_tx_frm(inst-> transport, &mb_frame_ptr);
    inst->master_dst_addr = snd_addr;
    mb_frame_ptr[MB_PDU_FUNC_OFF]                 = MB_FUNC_READ_DISCRETE_INPUTS;

    inst->master_el_addr                          = discrete_addr;
    mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF]        = discrete_addr >> 8;
    mb_frame_ptr[MB_PDU_REQ_READ_ADDR_OFF + 1]    = discrete_addr;

    inst->master_el_cnt                           = discrete_num;
    mb_frame_ptr[MB_PDU_REQ_READ_DISCCNT_OFF ]    = discrete_num >> 8;
    mb_frame_ptr[MB_PDU_REQ_READ_DISCCNT_OFF + 1] = discrete_num;

    *(inst->pdu_snd_len) = (MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE);
    (void)inst->pmt->evt_post(inst->port, EV_FRAME_SENT);

    return MB_ENOERR;
}

mb_exception_enum  mb_mstr_fn_read_discrete_inputs(mb_inst_struct *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    UCHAR           byte_num;
    //UCHAR          *mb_frame_ptr;

    mb_exception_enum    status = MB_EX_NONE;
    mb_err_enum    reg_status;
    if (inst->trmt->rq_is_broadcast(inst->transport))
    {
        status = MB_EX_NONE;
    }
    else if (*len_buf >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN)
    {
        USHORT discrete_cnt;
        discrete_cnt = inst->master_el_cnt;
        /* Test if the quantity of coils is a multiple of 8. If not last
         * byte is only partially field with unused coils set to zero. */
        if ((discrete_cnt & 0x0007) != 0)
        {
            byte_num = (UCHAR)(discrete_cnt / 8 + 1);
        }
        else
        {
            byte_num = (UCHAR)(discrete_cnt / 8);
        }

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception.
         */
        if ((discrete_cnt >= 1) && byte_num == frame_ptr[MB_PDU_FUNC_READ_DISCCNT_OFF])
        {
            /* Make callback to fill the buffer. */
            reg_status = mb_mstr_reg_discrete_cb(inst, &frame_ptr[MB_PDU_FUNC_READ_VALUES_OFF], inst->master_el_addr + 1, discrete_cnt);

            /* If an error occured convert it into a Modbus exception. */
            if(reg_status != MB_ENOERR)
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
#endif
