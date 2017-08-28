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
 * File: $Id: mbutils.h, v 1.5 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_UTILS_H
#define _MB_UTILS_H

#include <mb_common.h>
PR_BEGIN_EXTERN_C

/*! \defgroup modbus_utils Utilities
 *
 * This module contains some utility functions which can be used by
 * the application. It includes some special functions for working with
 * bitfields backed by a character array buffer.
 *
 */
/*! \addtogroup modbus_utils
 *  @{
 */
/*! \brief Function to set bits in a byte buffer.
 *
 * This function allows the efficient use of an array to implement bitfields.
 * The array used for storing the bits must always be a multiple of two
 * bytes. Up to eight bits can be set or cleared in one operation.
 *
 * \param byte_buf A buffer where the bit values are stored. Must be a
 *   multiple of 2 bytes. No length checking is performed and if
 *   bit_offset / 8 is greater than the size of the buffer memory contents
 *   is overwritten.
 * \param bit_offset The starting address of the bits to set. The first
 *   bit has the offset 0.
 * \param but_num Number of bits to modify. The value must always be smaller
 *   than 8.
 * \param ucValues Thew new values for the bits. The value for the first bit
 *   starting at <code>bit_offset</code> is the LSB of the value
 *   <code>ucValues</code>
 *
 * \code
 * ucBits[2] = {0, 0};
 *
 * // Set bit 4 to 1 (read: set 1 bit starting at bit offset 4 to value 1)
 * mb_util_set_bits(ucBits, 4, 1, 1);
 *
 * // Set bit 7 to 1 and bit 8 to 0.
 * mb_util_set_bits(ucBits, 7, 2, 0x01);
 *
 * // Set bits 8 - 11 to 0x05 and bits 12 - 15 to 0x0A;
 * mb_util_set_bits(ucBits, 8, 8, 0x5A);
 * \endcode
 */
void mb_util_set_bits(UCHAR * byte_buf, USHORT bit_offset, UCHAR but_num, UCHAR ucValues);

/*! \brief Function to read bits in a byte buffer.
 *
 * This function is used to extract up bit values from an array. Up to eight
 * bit values can be extracted in one step.
 *
 * \param byte_buf A buffer where the bit values are stored.
 * \param bit_offset The starting address of the bits to set. The first
 *   bit has the offset 0.
 * \param but_num Number of bits to modify. The value must always be smaller
 *   than 8.
 *
 * \code
 * UCHAR ucBits[2] = {0, 0};
 * UCHAR ucResult;
 *
 * // Extract the bits 3 - 10.
 * ucResult = mb_util_get_bits(ucBits, 3, 8);
 * \endcode
 */
UCHAR mb_util_get_bits(UCHAR * byte_buf, USHORT bit_offset, UCHAR but_num);

/*! @} */
PR_END_EXTERN_C
#endif
