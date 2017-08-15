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
 * File: $Id: mbport.h, v 1.19 2010/06/06 13:54:40 wolti Exp $
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#include <mb_common.h>
PR_BEGIN_EXTERN_C

#include <mbconfig.h>

#if MB_TCP_ENABLED
typedef struct _mp_port_tcp mp_port_tcp; //!< TCP port, child of #mb_port_base
#endif

#if MB_RTU_ENABLED || MB_ASCII_ENABLED
typedef struct _mb_port_ser mb_port_ser; //!< Serial port, child of #mb_port_base
#endif

typedef BOOL (*mb_fp_bool)(void *arg);

typedef struct
{
    mb_fp_bool byte_rcvd;   //!<pxMBFrameCBByteReceived;
    mb_fp_bool tx_empty;    //!<pxMBFrameCBTransmitterEmpty;
    mb_fp_bool tmr_expired; //!<pxMBPortCBTimerExpired;
}
mb_port_cb;                 //!<port callback table

typedef struct //_mb_port_base
{
    mb_port_cb  *cb; //!<Port callbacks.
    void       *arg; //!<CB arg pointer.
}mb_port_base; //!< Port base type

typedef struct //_mb_trans_base
{
    mb_port_base * port_obj;
}mb_trans_base; //!< Transport base type

#include <mbframe.h>
typedef struct
{
    pvMBFrameClose  frm_close;//!<pvMBFrameCloseCur;
    pvPortEventPost evt_post; //!<pvPortEventPostCur;
    pvPortEventGet  evt_get;  //!<pvPortEventGetCur;
}
mb_port_mtab; //!< Port methods tab;

typedef enum
{
    MB_PAR_NONE,                /*!< No parity. */
    MB_PAR_ODD,                 /*!< Odd parity. */
    MB_PAR_EVEN                 /*!< Even parity. */
} eMBParity;

/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
*/
/* ----------------------- Serial Supporting functions -----------------------------*/
BOOL xMBPortEventInit(mb_port_ser* inst                                    );
BOOL xMBPortEventPost(mb_port_ser* inst, eMBEventType eEvent               );
BOOL xMBPortEventGet (mb_port_ser* inst, void* caller, eMBEventType * eEvent); //FIXME
/* ----------------------- TCP Supporting functions -----------------------------*/
#if MB_TCP_ENABLED
BOOL xMBTCPPortEventInit(MULTIPORT_TCP_ARG_void                              );
BOOL xMBTCPPortEventPost(MULTIPORT_TCP_ARG eMBEventType eEvent               );
BOOL xMBTCPPortEventGet (MULTIPORT_TCP_ARG void* caller, eMBEventType * eEvent); //FIXME
#endif
/* ----------------------- Serial port functions ----------------------------*/
BOOL xMBPortSerialInit   (mb_port_ser* inst, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity);
void vMBPortClose        (mb_port_ser* inst                                                      );
void xMBPortSerialClose  (mb_port_ser* inst                                                      );
void vMBPortSerialEnable (mb_port_ser* inst,  BOOL xRxEnable, BOOL xTxEnable                     );
BOOL xMBPortSerialGetByte(mb_port_ser* inst,  CHAR * pucByte                                     );
BOOL xMBPortSerialPutByte(mb_port_ser* inst, CHAR ucByte                                         );
/* ----------------------- Timers functions ---------------------------------*/
BOOL xMBPortTimersInit   (mb_port_ser* inst, USHORT usTimeOut50us);
void xMBPortTimersClose  (mb_port_ser* inst                     );
void vMBPortTimersEnable (mb_port_ser* inst                     );
void vMBPortTimersDisable(mb_port_ser* inst                     );
void vMBPortTimersDelay  (mb_port_ser* inst, USHORT usTimeOutMS );

/*-------------------------TCP Timers functions ---------------------------------*/
#if MB_TCP_ENABLED > 0
BOOL xMBTCPPortTimersInit   (MULTIPORT_TCP_ARG USHORT usTimeOut50us);
void xMBTCPPortTimersClose  (MULTIPORT_TCP_ARG_VOID               );
void vMBTCPPortTimersEnable (MULTIPORT_TCP_ARG_VOID               );
void vMBTCPPortTimersDisable(MULTIPORT_TCP_ARG_VOID               );
void vMBTCPPortTimersDelay  (MULTIPORT_TCP_ARG USHORT usTimeOutMS );
#endif
/* ----------------------- Callback for the protocol stack ------------------*/

/*!
 * \brief Callback function for the porting layer when a new byte is
 *   available.
 *
 * Depending upon the mode this callback function is used by the RTU or
 * ASCII transmission layers. In any case a call to xMBPortSerialGetByte()
 * must immediately return a new character.
 *
 * \return <code>TRUE</code> if a event was posted to the queue because
 *   a new byte was received. The port implementation should wake up the
 *   tasks which are currently blocked on the eventqueue.
 */
//extern BOOL(*pxMBFrameCBByteReceived)    (void* transport);
//extern BOOL(*pxMBFrameCBTransmitterEmpty)(void* transport);
//extern BOOL(*pxMBPortCBTimerExpired)     (void* transport);
/* ----------------------- TCP port functions -------------------------------*/
#if MB_TCP_ENABLED > 0
BOOL xMBTCPPortInit        (MULTIPORT_TCP_ARG USHORT usTCPPort, SOCKADDR_IN hostaddr, BOOL bMaster);
void vMBTCPPortClose       (MULTIPORT_TCP_ARG_VOID                                               );
void vMBTCPPortDisable     (MULTIPORT_TCP_ARG_VOID                                               );
BOOL xMBTCPPortGetRequest  (MULTIPORT_TCP_ARG UCHAR **ppucMBTCPFrame, USHORT * usTCPLength       );
BOOL xMBTCPPortSendResponse(MULTIPORT_TCP_ARG const UCHAR *pucMBTCPFrame, USHORT usTCPLength     );
#endif
#if MB_MASTER >0
INLINE void vMBPortTimersConvertDelayEnable  (mb_port_ser* inst);
INLINE void vMBPortTimersRespondTimeoutEnable(mb_port_ser* inst);
#endif

PR_END_EXTERN_C
#endif
