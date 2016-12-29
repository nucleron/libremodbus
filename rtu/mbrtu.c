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
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "serial_port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

#include "rtu_multiport.h"
#include "mb_master.h"




#ifndef MB_MULTIPORT
/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;


/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;

volatile UCHAR  ucRTUBuf[MB_SER_PDU_SIZE_MAX];

static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

static volatile USHORT usRcvBufferPos;

#else

	#define STATE_RX_INIT RTU_STATE_RX_INIT
	#define STATE_RX_IDLE RTU_STATE_RX_IDLE
	#define STATE_RX_RCV  RTU_STATE_RX_RCV
	#define STATE_RX_ERROR RTU_STATE_RX_ERROR

	#define STATE_TX_IDLE RTU_STATE_TX_IDLE
	#define	STATE_TX_XMIT RTU_STATE_TX_XMIT
	#define	STATE_TX_XFWR RTU_STATE_TX_XFWR

	#define SERIAL_ARG &(inst->serial_port),
	#define SERIAL_ARG_VOID &(inst->serial_port)

	#define eSndState inst->eSndState
	#define eRcvState inst->eRcvState

	#define ucRTURcvBuf inst->ucRTURcvBuf
	#define ucRTUSndBuf inst->ucRTUSndBuf

	#define xFrameIsBroadcast inst->xFrameIsBroadcast
	#define eCurTimerMode inst->eCurTimerMode

	#define rtuMaster inst->rtuMaster

	#define pucSndBufferCur inst->pucSndBufferCur
	#define usSndBufferCount inst->usSndBufferCount

	#define txFrame inst->txFrame
	#define rxFrame inst->rxFrame

	#define usRcvBufferPos inst->usRcvBufferPos
	#define usSendPDULength inst->usSendPDULength

#endif /*ndef RTU_MULTIPORT*/


/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBRTUInit(RTU_ARG UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    ULONG           usTimerT35_50us;

    ( void )ucSlaveAddress;
    ENTER_CRITICAL_SECTION(  );

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( SERIAL_ARG ucPort, ulBaudRate, 8, eParity ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
            usTimerT35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
        }
        if( xMBPortTimersInit(SERIAL_ARG  ( USHORT ) usTimerT35_50us ) != TRUE )
        {
            eStatus = MB_EPORTERR;
        }
	#ifdef MB_MULTIPORT
        eSndState = STATE_TX_IDLE;
        eRcvState = STATE_RX_INIT;

        inst->serial_port.parent = (void*)(inst);
    #endif
    }
    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

void
eMBRTUStart(RTU_ARG_VOID)
{
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable(SERIAL_ARG TRUE, FALSE );
    vMBPortTimersEnable(SERIAL_ARG_VOID );

    EXIT_CRITICAL_SECTION(  );
}

void
eMBRTUStop(RTU_ARG_VOID)
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable(SERIAL_ARG FALSE, FALSE );
    vMBPortTimersDisable( SERIAL_ARG_VOID  );
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBRTUReceive(RTU_ARG UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
    BOOL            xFrameReceived = FALSE;
    eMBErrorCode    eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( UCHAR * ) ucRTURcvBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTURcvBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucRTURcvBuf[MB_SER_PDU_PDU_OFF];

        xFrameReceived = TRUE;
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBRTUSend(RTU_ARG UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;

    ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if( eRcvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        ucRTUSndBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        ucRTUSndBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /* Activate the transmitter. */
        eSndState = STATE_TX_XMIT;
        vMBPortSerialEnable(SERIAL_ARG FALSE, TRUE );
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
xMBRTUReceiveFSM(RTU_ARG_VOID)
{
    BOOL            xTaskNeedSwitch = FALSE;
    UCHAR           ucByte;

    assert(( eSndState == STATE_TX_IDLE ) || ( eSndState == STATE_TX_XFWR ));

    /* Always read the character. */
    ( void )xMBPortSerialGetByte(SERIAL_ARG ( CHAR * ) & ucByte );

    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
        vMBPortTimersEnable( SERIAL_ARG_VOID  );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        vMBPortTimersEnable( SERIAL_ARG_VOID  );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:

    	if(rtuMaster)
    	{
    		vMBPortTimersDisable( SERIAL_ARG_VOID );
			eSndState = STATE_TX_IDLE;
    	}
        usRcvBufferPos = 0;
        ucRTURcvBuf[usRcvBufferPos++] = ucByte;
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        vMBPortTimersEnable( SERIAL_ARG_VOID  );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTURcvBuf[usRcvBufferPos++] = ucByte;
        }
        else
        {
            eRcvState = STATE_RX_ERROR;
        }
        vMBPortTimersEnable( SERIAL_ARG_VOID );
        break;
    }
    return xTaskNeedSwitch;
}

BOOL
xMBRTUTransmitFSM( RTU_ARG_VOID )
{
    BOOL            xNeedPoll = FALSE;

    assert( eRcvState == STATE_RX_IDLE );

    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable(SERIAL_ARG  TRUE, FALSE );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutByte(SERIAL_ARG ( CHAR )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
        }
        else
        {
        	if(rtuMaster==TRUE)
        	{
				#if MB_MASTER >0
        		xFrameIsBroadcast = ( ucRTUSndBuf[MB_SER_PDU_ADDR_OFF] == MB_ADDRESS_BROADCAST ) ? TRUE : FALSE;
				/* Disable transmitter. This prevents another transmit buffer
				 * empty interrupt. */
				vMBPortSerialEnable(SERIAL_ARG TRUE, FALSE );
				eSndState = STATE_TX_XFWR;
				/* If the frame is broadcast ,master will enable timer of convert delay,
				 * else master will enable timer of respond timeout. */
				if ( xFrameIsBroadcast == TRUE )
				{
					vMBPortTimersConvertDelayEnable( SERIAL_ARG_VOID );
				}
				else
				{
					vMBPortTimersRespondTimeoutEnable( SERIAL_ARG_VOID );
				}
				#endif
        	}
        	else
        	{
				xNeedPoll = xMBPortEventPost(SERIAL_ARG EV_FRAME_SENT );
				/* Disable transmitter. This prevents another transmit buffer
				 * empty interrupt. */
				vMBPortSerialEnable(SERIAL_ARG TRUE, FALSE );
				eSndState = STATE_TX_IDLE;
        	}
        }
        break;
    }

    return xNeedPoll;
}



BOOL
xMBRTUTimerT35Expired( RTU_ARG_VOID )
{
    BOOL            xNeedPoll = FALSE;

    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost(&(inst->serial_port), EV_READY );//fixme
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost(&(inst->serial_port), EV_FRAME_RECEIVED ); //fixme too
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        assert( ( eRcvState == STATE_RX_INIT ) ||
                ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
    }

    vMBPortTimersDisable( SERIAL_ARG_VOID  );
    eRcvState = STATE_RX_IDLE;
#if MB_MASTER >0
    if(rtuMaster == TRUE)
    {
    	switch (eSndState)
    	{
    		/* A frame was send finish and convert delay or respond timeout expired.
    		 * If the frame is broadcast,The master will idle,and if the frame is not
    		 * broadcast.Notify the listener process error.*/
    	case STATE_TX_XFWR:
    		if ( xFrameIsBroadcast == FALSE ) {
    			((MBInstance*)(inst->parent))->eMBMasterCurErrorType = ERR_EV_ERROR_RESPOND_TIMEOUT;
    			//vMBSetErrorType(ERR_EV_ERROR_RESPOND_TIMEOUT); //FIXME pass reference to instance
    			xNeedPoll = xMBPortEventPost(SERIAL_ARG EV_ERROR_PROCESS);
    		}
    		break;
    		/* Function called in an illegal state. */
    	default:
    		assert(
    				( eSndState == STATE_TX_XFWR ) || ( eSndState == STATE_TX_IDLE ));
    		break;
    	}
    	eSndState = STATE_TX_IDLE;

    	vMBPortTimersDisable( SERIAL_ARG_VOID );
    	/* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
    	if (eCurTimerMode == MB_TMODE_CONVERT_DELAY)
    	{
    		xNeedPoll = xMBPortEventPost(SERIAL_ARG EV_EXECUTE );
    	}
    }
#endif

    return xNeedPoll;
}


#if MB_MASTER > 0

/* Get Modbus Master send RTU's buffer address pointer.*/
void vMBMasterGetRTUSndBuf( RTU_ARG UCHAR ** pucFrame )
{
	*pucFrame = ( UCHAR * ) ucRTUSndBuf;
}

/* Get Modbus Master send PDU's buffer address pointer.*/
void vMBMasterGetPDUSndBuf( RTU_ARG UCHAR ** pucFrame )
{
	*pucFrame = ( UCHAR * ) &ucRTUSndBuf[MB_SER_PDU_PDU_OFF];
}

/* Set Modbus Master send PDU's buffer length.*/
void vMBMasterSetPDUSndLength(  RTU_ARG USHORT SendPDULength )
{
	usSendPDULength = SendPDULength;
}

/* Get Modbus Master send PDU's buffer length.*/
USHORT usMBMasterGetPDUSndLength(  RTU_ARG_VOID )
{
	return usSendPDULength;
}

/* Set Modbus Master current timer mode.*/
void vMBMasterSetCurTimerMode( RTU_ARG eMBMasterTimerMode eMBTimerMode )
{
	eCurTimerMode = eMBTimerMode;
}

/* The master request is broadcast? */
BOOL xMBMasterRequestIsBroadcast( RTU_ARG_VOID )
{
	return xFrameIsBroadcast;
}

#endif
