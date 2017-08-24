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
 * File: $Id: mbfunc.h, v 1.12 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_FUNC_H
#define _MB_FUNC_H

#include <mb_common.h>
PR_BEGIN_EXTERN_C

#if MB_FUNC_OTHER_REP_SLAVEID_BUF > 0
    mb_exception_enum mb_fn_report_slv_id(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_READ_INPUT_ENABLED > 0
mb_exception_enum    mb_fn_read_input_reg(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0
mb_exception_enum    mb_fn_read_holding_reg(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
mb_exception_enum    mb_fn_write_holding_reg(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
mb_exception_enum    mb_fn_write_multi_holding_reg(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_READ_COILS_ENABLED > 0
mb_exception_enum    mb_fn_read_coils(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_WRITE_COIL_ENABLED > 0
mb_exception_enum    mb_fn_write_coil(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
mb_exception_enum    mb_fn_write_multi_coils(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
mb_exception_enum    mb_fn_read_discrete_inp(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
mb_exception_enum    mb_fn_rw_multi_holding_reg(mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
#endif

PR_END_EXTERN_C
#endif
