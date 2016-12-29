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
 * File: $Id: mbascii.h,v 1.8 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_ASCII_H
#define _MB_ASCII_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#include "ascii_multiport.h"

#if MB_ASCII_ENABLED > 0
eMBErrorCode    eMBASCIIInit(ASCII_ARG UCHAR slaveAddress, UCHAR ucPort,
                              ULONG ulBaudRate, eMBParity eParity );
void            eMBASCIIStart(ASCII_ARG_VOID);
void            eMBASCIIStop(ASCII_ARG_VOID);

eMBErrorCode    eMBASCIIReceive(ASCII_ARG UCHAR * pucRcvAddress, UCHAR ** pucFrame,
                                 USHORT * pusLength );
eMBErrorCode    eMBASCIISend(ASCII_ARG UCHAR slaveAddress, const UCHAR * pucFrame,
                              USHORT usLength );
BOOL            xMBASCIIReceiveFSM(ASCII_ARG_VOID);
BOOL            xMBASCIITransmitFSM(ASCII_ARG_VOID);
BOOL            xMBASCIITimerT1SExpired(ASCII_ARG_VOID);
#endif

//master
void vMBASCIIMasterGetPDUSndBuf(ASCII_ARG UCHAR ** pucFrame );
USHORT usMBASCIIMasterGetPDUSndLength( ASCII_ARG_VOID );
void vMBASCIIMasterSetPDUSndLength(ASCII_ARG USHORT SendPDULength );
void vMBASCIIMasterSetCurTimerMode(ASCII_ARG eMBMasterTimerMode eMBTimerMode );
BOOL xMBASCIIMasterRequestIsBroadcast( ASCII_ARG_VOID );
eMBMasterErrorEventType eMBASCIIMasterGetErrorType( ASCII_ARG_VOID );
eMBMasterReqErrCode eMBASCIIMasterWaitRequestFinish( void );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
