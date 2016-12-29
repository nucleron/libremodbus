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
 * File: $Id: mbport.h,v 1.19 2010/06/06 13:54:40 wolti Exp $
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#include "mbconfig.h"

#if MB_TCP_ENABLED
	#include "tcp_multi.h"
#else
//#error "watafuck"
#endif

#if MB_RTU_ENABLED || MB_ASCII_ENABLED
#include "serial_multi.h"

#endif

typedef enum
	{
		MB_PAR_NONE,                /*!< No parity. */
		MB_PAR_ODD,                 /*!< Odd parity. */
		MB_PAR_EVEN                 /*!< Even parity. */
	} eMBParity;


#ifdef __cplusplus
extern "C" {
#endif

//typedef enum
//{
//    EV_MASTER_READY                    = 1<<0,  /*!< Startup finished. */
//    EV_MASTER_FRAME_RECEIVED           = 1<<1,  /*!< Frame received. */
//    EV_MASTER_EXECUTE                  = 1<<2,  /*!< Execute function. */
//    EV_MASTER_FRAME_SENT               = 1<<3,  /*!< Frame sent. */
 //   EV_MASTER_ERROR_PROCESS            = 1<<4,  /*!< Frame error process. */
//    EV_MASTER_PROCESS_SUCESS           = 1<<5,  /*!< Request process success. */
//    EV_MASTER_ERROR_RESPOND_TIMEOUT    = 1<<6,  /*!< Request respond timeout. */
//    EV_MASTER_ERROR_RECEIVE_DATA       = 1<<7,  /*!< Request receive data error. */
//    EV_MASTER_ERROR_EXECUTE_FUNCTION   = 1<<8,  /*!< Request execute function error. */
//} eMBMasterEventType;

typedef enum
{
    ERR_EV_ERROR_RESPOND_TIMEOUT,         /*!< Slave respond timeout. */
    ERR_EV_ERROR_RECEIVE_DATA,            /*!< Receive frame data erroe. */
    ERR_EV_ERROR_EXECUTE_FUNCTION,        /*!< Execute function error. */
} eMBMasterErrorEventType;

/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
*/

/* ----------------------- Serial Supporting functions -----------------------------*/
BOOL            xMBPortEventInit( MULTIPORT_SERIAL_ARG_VOID  );

BOOL            xMBPortEventPost( MULTIPORT_SERIAL_ARG eMBEventType eEvent );

BOOL            xMBPortEventGet(  MULTIPORT_SERIAL_ARG void* caller,/*@out@ */ eMBEventType * eEvent ); //FIXME

/* ----------------------- TCP Supporting functions -----------------------------*/
#if MB_TCP_ENABLED
BOOL            xMBTCPPortEventInit( MULTIPORT_TCP_ARG_VOID  );

BOOL            xMBTCPPortEventPost( MULTIPORT_TCP_ARG eMBEventType eEvent );

BOOL            xMBTCPPortEventGet(  MULTIPORT_TCP_ARG void* caller,/*@out@ */ eMBEventType * eEvent ); //FIXME
#endif
/* ----------------------- Serial port functions ----------------------------*/

BOOL            xMBPortSerialInit(MULTIPORT_SERIAL_ARG  UCHAR ucPort, ULONG ulBaudRate,
                                   UCHAR ucDataBits, eMBParity eParity );

void            vMBPortClose(MULTIPORT_SERIAL_ARG_VOID );

void            xMBPortSerialClose(MULTIPORT_SERIAL_ARG_VOID );

void            vMBPortSerialEnable(MULTIPORT_SERIAL_ARG  BOOL xRxEnable, BOOL xTxEnable );

BOOL            xMBPortSerialGetByte(MULTIPORT_SERIAL_ARG  CHAR * pucByte );

BOOL            xMBPortSerialPutByte(MULTIPORT_SERIAL_ARG CHAR ucByte );

/* ----------------------- Timers functions ---------------------------------*/
BOOL            xMBPortTimersInit(MULTIPORT_SERIAL_ARG USHORT usTimeOut50us );

void            xMBPortTimersClose( MULTIPORT_SERIAL_ARG_VOID );

void            vMBPortTimersEnable( MULTIPORT_SERIAL_ARG_VOID );

void            vMBPortTimersDisable( MULTIPORT_SERIAL_ARG_VOID );

void            vMBPortTimersDelay(MULTIPORT_SERIAL_ARG USHORT usTimeOutMS );

/*-------------------------TCP Timers functions ---------------------------------*/
#if MB_TCP_ENABLED
BOOL            xMBTCPPortTimersInit(MULTIPORT_TCP_ARG USHORT usTimeOut50us );

void            xMBTCPPortTimersClose( MULTIPORT_TCP_ARG_VOID );

void            vMBTCPPortTimersEnable( MULTIPORT_TCP_ARG_VOID );

void            vMBTCPPortTimersDisable( MULTIPORT_TCP_ARG_VOID );

void            vMBTCPPortTimersDelay(MULTIPORT_TCP_ARG USHORT usTimeOutMS );
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
extern          BOOL( *pxMBFrameCBByteReceived ) ( CALLBACK_ARG );

extern          BOOL( *pxMBFrameCBTransmitterEmpty ) ( CALLBACK_ARG );

extern          BOOL( *pxMBPortCBTimerExpired ) ( CALLBACK_ARG );

/* ----------------------- TCP port functions -------------------------------*/
#if MB_TCP_ENABLED
BOOL            xMBTCPPortInit(MULTIPORT_TCP_ARG USHORT usTCPPort, SOCKADDR_IN hostaddr, BOOL bMaster );

void            vMBTCPPortClose(MULTIPORT_TCP_ARG_VOID);

void            vMBTCPPortDisable(MULTIPORT_TCP_ARG_VOID);

BOOL            xMBTCPPortGetRequest(MULTIPORT_TCP_ARG UCHAR **ppucMBTCPFrame, USHORT * usTCPLength );

BOOL            xMBTCPPortSendResponse(MULTIPORT_TCP_ARG const UCHAR *pucMBTCPFrame, USHORT usTCPLength );
#endif


#ifdef MB_MASTER
INLINE void     vMBPortTimersConvertDelayEnable( MULTIPORT_SERIAL_ARG_VOID );

INLINE void     vMBPortTimersRespondTimeoutEnable( MULTIPORT_SERIAL_ARG_VOID );
#endif

#ifdef __cplusplus
}
#endif
#endif
