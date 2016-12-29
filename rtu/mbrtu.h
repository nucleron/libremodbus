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
 * File: $Id: mbrtu.h,v 1.9 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_RTU_H
#define _MB_RTU_H

#include "rtu_multiport.h"

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif
    eMBErrorCode eMBRTUInit(RTU_ARG UCHAR slaveAddress, UCHAR ucPort, ULONG ulBaudRate,
                             eMBParity eParity );
void            eMBRTUStart(RTU_ARG_VOID);
void            eMBRTUStop(RTU_ARG_VOID);
eMBErrorCode    eMBRTUReceive(RTU_ARG UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength );
eMBErrorCode    eMBRTUSend(RTU_ARG UCHAR slaveAddress, const UCHAR * pucFrame, USHORT usLength );
BOOL            xMBRTUReceiveFSM(RTU_ARG_VOID);
BOOL            xMBRTUTransmitFSM(RTU_ARG_VOID);
BOOL            xMBRTUTimerT15Expired(RTU_ARG_VOID);
BOOL            xMBRTUTimerT35Expired(RTU_ARG_VOID);

//master
void vMBMasterGetPDUSndBuf(RTU_ARG UCHAR ** pucFrame );
USHORT usMBMasterGetPDUSndLength( RTU_ARG_VOID );
void vMBMasterSetPDUSndLength(RTU_ARG USHORT SendPDULength );
void vMBMasterSetCurTimerMode(RTU_ARG eMBMasterTimerMode eMBTimerMode );
BOOL xMBMasterRequestIsBroadcast( RTU_ARG_VOID );
eMBMasterErrorEventType eMBMasterGetErrorType( RTU_ARG_VOID );
eMBMasterReqErrCode eMBMasterWaitRequestFinish( void );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
