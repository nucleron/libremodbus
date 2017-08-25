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
 * File: $Id: mbutils.c, v 1.6 2007/02/18 23:49:07 wolti Exp $
 */
#include <mb.h>

/* ----------------------- Defines ------------------------------------------*/
#define BITS_UCHAR      8U

/* ----------------------- Start implementation -----------------------------*/
void
xMBUtilSetBits(UCHAR * byte_buf, USHORT bit_offset, UCHAR but_num,
                UCHAR ucValue)
{
    USHORT          usWordBuf;
    USHORT          usMask;
    USHORT          usByteOffset;
    USHORT          usNPreBits;
    USHORT          usValue = ucValue;

    assert(but_num <= 8);
    assert((size_t)BITS_UCHAR == sizeof(UCHAR) * 8);

    /* Calculate byte offset for first byte containing the bit values starting
     * at bit_offset. */
    usByteOffset = (USHORT)((bit_offset) / BITS_UCHAR);

    /* How many bits precede our bits to set. */
    usNPreBits = (USHORT)(bit_offset - usByteOffset * BITS_UCHAR);

    /* Move bit field into position over bits to set */
    usValue <<= usNPreBits;

    /* Prepare a mask for setting the new bits. */
    usMask = (USHORT)((1 << (USHORT) but_num) - 1);
    usMask <<= bit_offset - usByteOffset * BITS_UCHAR;

    /* copy bits into temporary storage. */
    usWordBuf = byte_buf[usByteOffset];
    usWordBuf |= byte_buf[usByteOffset + 1] << BITS_UCHAR;

    /* Zero out bit field bits and then or value bits into them. */
    usWordBuf = (USHORT)((usWordBuf & (~usMask)) | usValue);

    /* move bits back into storage */
    byte_buf[usByteOffset] = (UCHAR)(usWordBuf & 0xFF);
    byte_buf[usByteOffset + 1] = (UCHAR)(usWordBuf >> BITS_UCHAR);
}

UCHAR
mb_util_get_bits(UCHAR * byte_buf, USHORT bit_offset, UCHAR but_num)
{
    USHORT          usWordBuf;
    USHORT          usMask;
    USHORT          usByteOffset;
    USHORT          usNPreBits;

    /* Calculate byte offset for first byte containing the bit values starting
     * at bit_offset. */
    usByteOffset = (USHORT)((bit_offset) / BITS_UCHAR);

    /* How many bits precede our bits to set. */
    usNPreBits = (USHORT)(bit_offset - usByteOffset * BITS_UCHAR);

    /* Prepare a mask for setting the new bits. */
    usMask = (USHORT)((1 << (USHORT) but_num) - 1);

    /* copy bits into temporary storage. */
    usWordBuf = byte_buf[usByteOffset];
    usWordBuf |= byte_buf[usByteOffset + 1] << BITS_UCHAR;

    /* throw away unneeded bits. */
    usWordBuf >>= usNPreBits;

    /* mask away bits above the requested bitfield. */
    usWordBuf &= usMask;

    return (UCHAR) usWordBuf;
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
