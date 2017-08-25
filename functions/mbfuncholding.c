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
 * File: $Id: mbfuncholding.c, v 1.12 2007/02/18 23:48:22 wolti Exp $
 */
#include <mb.h>
/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF               (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READ_REGCNT_OFF             (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READ_SIZE                   (4)
#define MB_PDU_FUNC_READ_REGCNT_MAX             (0x007D)

#define MB_PDU_FUNC_WRITE_ADDR_OFF              (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_VALUE_OFF             (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_SIZE                  (4)

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF          (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF        (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF       (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF        (MB_PDU_DATA_OFF + 5)
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN          (5)
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX        (0x0078)

#define MB_PDU_FUNC_READWRITE_READ_ADDR_OFF     (MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF   (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF    (MB_PDU_DATA_OFF + 4)
#define MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF  (MB_PDU_DATA_OFF + 6)
#define MB_PDU_FUNC_READWRITE_BYTECNT_OFF       (MB_PDU_DATA_OFF + 8)
#define MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF  (MB_PDU_DATA_OFF + 9)
#define MB_PDU_FUNC_READWRITE_SIZE_MIN          (9)

/* ----------------------- Static functions ---------------------------------*/
mb_exception_enum    mb_error_to_exception(mb_err_enum error_code);

/* ----------------------- Start implementation -----------------------------*/

#if MB_FUNC_WRITE_HOLDING_ENABLED > 0

mb_exception_enum
mb_fn_write_holding_reg(mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    USHORT            reg_addr;
    mb_exception_enum status = MB_EX_NONE;
    mb_err_enum       reg_status;

    if (*len_buf == (MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN))
    {
        reg_addr = (USHORT)(frame_ptr[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
        reg_addr |= (USHORT)(frame_ptr[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);
        reg_addr++;

        /* Make callback to update the value. */
        reg_status = mb_reg_holding_cb(&frame_ptr[MB_PDU_FUNC_WRITE_VALUE_OFF],
                                      reg_addr, 1, MB_REG_WRITE);

        /* If an error occured convert it into a Modbus exception. */
        if (reg_status != MB_ENOERR)
        {
            status = mb_error_to_exception(reg_status);
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

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
mb_exception_enum
mb_fn_write_multi_holding_reg(mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    USHORT          reg_addr;
    USHORT          reg_cnt;
    UCHAR           reg_byte_cnt;

    mb_exception_enum    status = MB_EX_NONE;
    mb_err_enum    reg_status;

    if (*len_buf >= (MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN))
    {
        reg_addr = (USHORT)(frame_ptr[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8);
        reg_addr |= (USHORT)(frame_ptr[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1]);
        reg_addr++;

        reg_cnt = (USHORT)(frame_ptr[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF] << 8);
        reg_cnt |= (USHORT)(frame_ptr[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF + 1]);

        reg_byte_cnt = frame_ptr[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        if ((reg_cnt >= 1) &&
            (reg_cnt <= MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX) &&
            (reg_byte_cnt == (UCHAR) (2 * reg_cnt)))
        {
            /* Make callback to update the register values. */
            reg_status =
                mb_reg_holding_cb(&frame_ptr[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                                 reg_addr, reg_cnt, MB_REG_WRITE);

            /* If an error occured convert it into a Modbus exception. */
            if (reg_status != MB_ENOERR)
            {
                status = mb_error_to_exception(reg_status);
            }
            else
            {
                /* The response contains the function code, the starting
                 * address and the quantity of registers. We reuse the
                 * old values in the buffer because they are still valid.
                 */
                *len_buf = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
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

#if MB_FUNC_READ_HOLDING_ENABLED > 0

mb_exception_enum
mb_fn_read_holding_reg(mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    USHORT          reg_addr;
    USHORT          reg_cnt;
    UCHAR          *frame_cur;

    mb_exception_enum    status = MB_EX_NONE;
    mb_err_enum    reg_status;

    if (*len_buf == (MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN))
    {
        reg_addr = (USHORT)(frame_ptr[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
        reg_addr |= (USHORT)(frame_ptr[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
        reg_addr++;

        reg_cnt = (USHORT)(frame_ptr[MB_PDU_FUNC_READ_REGCNT_OFF] << 8);
        reg_cnt = (USHORT)(frame_ptr[MB_PDU_FUNC_READ_REGCNT_OFF + 1]);

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception.
         */
        if ((reg_cnt >= 1) && (reg_cnt <= MB_PDU_FUNC_READ_REGCNT_MAX))
        {
            /* Set the current PDU data pointer to the beginning. */
            frame_cur = &frame_ptr[MB_PDU_FUNC_OFF];
            *len_buf = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *frame_cur++ = MB_FUNC_READ_HOLDING_REGISTER;
            *len_buf += 1;

            /* Second byte in the response contain the number of bytes. */
            *frame_cur++ = (UCHAR) (reg_cnt * 2);
            *len_buf += 1;

            /* Make callback to fill the buffer. */
            reg_status = mb_reg_holding_cb(frame_cur, reg_addr, reg_cnt, MB_REG_READ);
            /* If an error occured convert it into a Modbus exception. */
            if (reg_status != MB_ENOERR)
            {
                status = mb_error_to_exception(reg_status);
            }
            else
            {
                *len_buf += reg_cnt * 2;
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

mb_exception_enum
mb_fn_rw_multi_holding_reg(mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf)
{
    USHORT          reg_rd_addr;
    USHORT          reg_rd_cnt;
    USHORT          reg_wr_addr;
    USHORT          reg_wr_cnt;
    UCHAR           reg_wr_byte_cnt;
    UCHAR          *frame_cur;

    mb_exception_enum    status = MB_EX_NONE;
    mb_err_enum    reg_status;

    if (*len_buf >= (MB_PDU_FUNC_READWRITE_SIZE_MIN + MB_PDU_SIZE_MIN))
    {
        reg_rd_addr = (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF] << 8U);
        reg_rd_addr |= (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF + 1]);
        reg_rd_addr++;

        reg_rd_cnt = (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF] << 8U);
        reg_rd_cnt |= (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF + 1]);

        reg_wr_addr = (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF] << 8U);
        reg_wr_addr |= (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF + 1]);
        reg_wr_addr++;

        reg_wr_cnt = (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF] << 8U);
        reg_wr_cnt |= (USHORT)(frame_ptr[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF + 1]);

        reg_wr_byte_cnt = frame_ptr[MB_PDU_FUNC_READWRITE_BYTECNT_OFF];

        if ((reg_rd_cnt >= 1) && (reg_rd_cnt <= 0x7D) &&
            (reg_wr_cnt >= 1) && (reg_wr_cnt <= 0x79) &&
            ((2 * reg_wr_cnt) == reg_wr_byte_cnt))
        {
            /* Make callback to update the register values. */
            reg_status = mb_reg_holding_cb(&frame_ptr[MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF],
                                          reg_wr_addr, reg_wr_cnt, MB_REG_WRITE);

            if (reg_status == MB_ENOERR)
            {
                /* Set the current PDU data pointer to the beginning. */
                frame_cur = &frame_ptr[MB_PDU_FUNC_OFF];
                *len_buf = MB_PDU_FUNC_OFF;

                /* First byte contains the function code. */
                *frame_cur++ = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
                *len_buf += 1;

                /* Second byte in the response contain the number of bytes. */
                *frame_cur++ = (UCHAR) (reg_rd_cnt * 2);
                *len_buf += 1;

                /* Make the read callback. */
                reg_status =
                    mb_reg_holding_cb(frame_cur, reg_rd_addr, reg_rd_cnt, MB_REG_READ);
                if (reg_status == MB_ENOERR)
                {
                    *len_buf += 2 * reg_rd_cnt;
                }
            }
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
