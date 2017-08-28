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
 * File: $Id: mbutils.c, v 1.6 2007/02/18 23:49:07 wolti Exp $
 */
#include <mb.h>
/* ----------------------- Defines ------------------------------------------*/
#define BITS_UCHAR      8U

/* ----------------------- Start implementation -----------------------------*/
void
mb_util_set_bits(UCHAR * byte_buf, USHORT bit_offset, UCHAR but_num,
                UCHAR ucValue)
{
    USHORT          word_buf;
    USHORT          msk;
    USHORT          byte_offset;
    USHORT          pre_bits_num;
    USHORT          usValue = ucValue;

    assert(but_num <= 8);
    assert((size_t)BITS_UCHAR == sizeof(UCHAR) * 8);

    /* Calculate byte offset for first byte containing the bit values starting
     * at bit_offset. */
    byte_offset = (USHORT)((bit_offset) / BITS_UCHAR);

    /* How many bits precede our bits to set. */
    pre_bits_num = (USHORT)(bit_offset - byte_offset * BITS_UCHAR);

    /* Move bit field into position over bits to set */
    usValue <<= pre_bits_num;

    /* Prepare a mask for setting the new bits. */
    msk = (USHORT)((1 << (USHORT) but_num) - 1);
    msk <<= bit_offset - byte_offset * BITS_UCHAR;

    /* copy bits into temporary storage. */
    word_buf = byte_buf[byte_offset];
    word_buf |= byte_buf[byte_offset + 1] << BITS_UCHAR;

    /* Zero out bit field bits and then or value bits into them. */
    word_buf = (USHORT)((word_buf & (~msk)) | usValue);

    /* move bits back into storage */
    byte_buf[byte_offset] = (UCHAR)(word_buf & 0xFF);
    byte_buf[byte_offset + 1] = (UCHAR)(word_buf >> BITS_UCHAR);
}

UCHAR
mb_util_get_bits(UCHAR * byte_buf, USHORT bit_offset, UCHAR but_num)
{
    USHORT          word_buf;
    USHORT          msk;
    USHORT          byte_offset;
    USHORT          pre_bits_num;

    /* Calculate byte offset for first byte containing the bit values starting
     * at bit_offset. */
    byte_offset = (USHORT)((bit_offset) / BITS_UCHAR);

    /* How many bits precede our bits to set. */
    pre_bits_num = (USHORT)(bit_offset - byte_offset * BITS_UCHAR);

    /* Prepare a mask for setting the new bits. */
    msk = (USHORT)((1 << (USHORT) but_num) - 1);

    /* copy bits into temporary storage. */
    word_buf = byte_buf[byte_offset];
    word_buf |= byte_buf[byte_offset + 1] << BITS_UCHAR;

    /* throw away unneeded bits. */
    word_buf >>= pre_bits_num;

    /* mask away bits above the requested bitfield. */
    word_buf &= msk;

    return (UCHAR) word_buf;
}

mb_exception_enum
mb_error_to_exception(mb_err_enum error_code)
{
    mb_exception_enum    status;

    switch (error_code)
    {
        case MB_ENOERR:
            status = MB_EX_NONE;
            break;

        case MB_ENOREG:
            status = MB_EX_ILLEGAL_DATA_ADDRESS;
            break;

        case MB_ETIMEDOUT:
            status = MB_EX_SLAVE_BUSY;
            break;

        default:
            status = MB_EX_SLAVE_DEVICE_FAILURE;
            break;
    }

    return status;
}
