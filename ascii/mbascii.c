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
 * File: $Id: mbascii.c,v 1.17 2010/06/06 13:47:07 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "serial_port.h"
#include "ascii_multiport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbascii.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

#if MB_ASCII_ENABLED > 0


/* ----------------------- Defines ------------------------------------------*/
#define MB_ASCII_DEFAULT_CR     '\r'    /*!< Default CR character for Modbus ASCII. */
#define MB_ASCII_DEFAULT_LF     '\n'    /*!< Default LF character for Modbus ASCII. */
#define MB_SER_PDU_SIZE_MIN     3       /*!< Minimum size of a Modbus ASCII frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus ASCII frame. */
#define MB_SER_PDU_SIZE_LRC     1       /*!< Size of LRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

#ifndef ASCII_MULTIPORT

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_WAIT_EOF           /*!< Wait for End of Frame. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_START,             /*!< Starting transmission (':' sent). */
    STATE_TX_DATA,              /*!< Sending of data (Address, Data, LRC). */
    STATE_TX_END,               /*!< End of transmission. */
    STATE_TX_NOTIFY             /*!< Notify sender that the frame has been sent. */
} eMBSndState;

typedef enum
{
    BYTE_HIGH_NIBBLE,           /*!< Character for high nibble of byte. */
    BYTE_LOW_NIBBLE             /*!< Character for low nibble of byte. */
} eMBBytePos;

#else

#define STATE_RX_IDLE ASCII_STATE_RX_IDLE
#define STATE_RX_RCV ASCII_STATE_RX_RCV
#define STATE_RX_WAIT_EOF ASCII_STATE_RX_WAIT_EOF

#define STATE_TX_IDLE ASCII_STATE_TX_IDLE
#define STATE_TX_START ASCII_STATE_TX_START
#define STATE_TX_DATA ASCII_STATE_TX_DATA
#define STATE_TX_END ASCII_STATE_TX_END
#define STATE_TX_NOTIFY ASCII_STATE_TX_NOTIFY

#define SERIAL_ARG &(inst->serial_port),
#define SERIAL_ARG_VOID &(inst->serial_port)

#define eSndState inst->eSndState
#define eRcvState inst->eRcvState

#define ucASCIIRcvBuf inst->ucASCIIRcvBuf
#define ucASCIISndBuf inst->ucASCIISndBuf

#define usRcvBufferPos inst->usRcvBufferPos
#define eBytePos inst->eBytePos
#define xFrameIsBroadcast inst->xFrameIsBroadcast
#define eCurTimerMode inst->eCurTimerMode
#define usSendPDULength inst->usSendPDULength
#define pucSndBufferCur inst->pucSndBufferCur
#define usSndBufferCount inst->usSndBufferCount
//#define ucLRC inst->ucLRC
#define asciiMaster inst->asciiMaster
#define ucMBLFCharacter inst->ucMBLFCharacter

#endif

/* ----------------------- Static functions ---------------------------------*/
static UCHAR    prvucMBCHAR2BIN( UCHAR ucCharacter );

static UCHAR    prvucMBBIN2CHAR( UCHAR ucByte );

static UCHAR    prvucMBLRC( UCHAR * pucFrame, USHORT usLen );

#ifndef ASCII_MULTIPORT
/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;

/* We reuse the Modbus RTU buffer because only one buffer is needed and the
 * RTU buffer is bigger. */
extern volatile UCHAR ucRTUBuf[];
static volatile UCHAR *ucASCIIRcvBuf = ucRTUBuf;

static volatile UCHAR *ucASCIISndBuf = ucRTUBuf;

static volatile USHORT usRcvBufferPos;
static volatile eMBBytePos eBytePos;

static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

static volatile UCHAR ucLRC;
static volatile UCHAR ucMBLFCharacter;

#endif

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBASCIIInit(ASCII_ARG UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    ( void )ucSlaveAddress;
    usSndBufferCount=0;

    ENTER_CRITICAL_SECTION(  );
    ucMBLFCharacter = MB_ASCII_DEFAULT_LF;

    if( xMBPortSerialInit(SERIAL_ARG ucPort, ulBaudRate, 7, eParity ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }
    else if( xMBPortTimersInit(SERIAL_ARG MB_ASCII_TIMEOUT_SEC * 20000UL ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }

    #ifdef MB_MULTIPORT
    eRcvState =  STATE_RX_IDLE;
    eSndState = STATE_TX_IDLE;
    inst->serial_port.parent = (void*)(inst);
	#endif


    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

void
eMBASCIIStart(ASCII_ARG_VOID )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable(SERIAL_ARG TRUE, FALSE );
    eRcvState = STATE_RX_IDLE;
    EXIT_CRITICAL_SECTION(  );

    /* No special startup required for ASCII. */
    ( void )xMBPortEventPost(SERIAL_ARG EV_READY );
}

void
eMBASCIIStop( ASCII_ARG_VOID )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable(SERIAL_ARG FALSE, FALSE );
    vMBPortTimersDisable(SERIAL_ARG_VOID);
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBASCIIReceive(ASCII_ARG  UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( prvucMBLRC( ( UCHAR * ) ucASCIIRcvBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucASCIIRcvBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_LRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucASCIIRcvBuf[MB_SER_PDU_PDU_OFF];
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBASCIISend(ASCII_ARG  UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    UCHAR           usLRC;

    ENTER_CRITICAL_SECTION(  );
    /* Check if the receiver is still in idle state. If not we where too
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

        /* Calculate LRC checksum for Modbus-Serial-Line-PDU. */
        usLRC = prvucMBLRC( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        ucASCIISndBuf[usSndBufferCount++] = usLRC;

        /* Activate the transmitter. */
        eSndState = STATE_TX_START;
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
xMBASCIIReceiveFSM(ASCII_ARG_VOID)
{
    BOOL            xNeedPoll = FALSE;
    UCHAR           ucByte;
    UCHAR           ucResult;

    assert(( eSndState == STATE_TX_IDLE )|| ( eSndState == ASCII_STATE_TX_XFWR ));

    ( void )xMBPortSerialGetByte(SERIAL_ARG ( CHAR * ) & ucByte );
    switch ( eRcvState )
    {
        /* A new character is received. If the character is a ':' the input
         * buffer is cleared. A CR-character signals the end of the data
         * block. Other characters are part of the data block and their
         * ASCII value is converted back to a binary representation.
         */
    case STATE_RX_RCV:
        /* Enable timer for character timeout. */
        vMBPortTimersEnable( SERIAL_ARG_VOID );
        if( ucByte == ':' )
        {
            /* Empty receive buffer. */
            eBytePos = BYTE_HIGH_NIBBLE;
            usRcvBufferPos = 0;
        }
        else if( ucByte == MB_ASCII_DEFAULT_CR )
        {
            eRcvState = STATE_RX_WAIT_EOF;
        }
        else
        {
            ucResult = prvucMBCHAR2BIN( ucByte );
            switch ( eBytePos )
            {
                /* High nibble of the byte comes first. We check for
                 * a buffer overflow here. */
            case BYTE_HIGH_NIBBLE:
                if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
                {
                    ucASCIIRcvBuf[usRcvBufferPos] = ( UCHAR )( ucResult << 4 );
                    eBytePos = BYTE_LOW_NIBBLE;
                    break;
                }
                else
                {
                    /* not handled in Modbus specification but seems
                     * a resonable implementation. */
                    eRcvState = STATE_RX_IDLE;
                    /* Disable previously activated timer because of error state. */
                    vMBPortTimersDisable(SERIAL_ARG_VOID);
                }
                break;

            case BYTE_LOW_NIBBLE:
                ucASCIIRcvBuf[usRcvBufferPos] |= ucResult;
                usRcvBufferPos++;
                eBytePos = BYTE_HIGH_NIBBLE;
                break;
            }
        }
        break;

    case STATE_RX_WAIT_EOF:
        if( ucByte == ucMBLFCharacter )
        {
            /* Disable character timeout timer because all characters are
             * received. */
            vMBPortTimersDisable(SERIAL_ARG_VOID  );
            /* Receiver is again in idle state. */
            eRcvState = STATE_RX_IDLE;

            /* Notify the caller of eMBASCIIReceive that a new frame
             * was received. */
            xNeedPoll = xMBPortEventPost(SERIAL_ARG EV_FRAME_RECEIVED );
        }
        else if( ucByte == ':' )
        {
            /* Empty receive buffer and back to receive state. */
            eBytePos = BYTE_HIGH_NIBBLE;
            usRcvBufferPos = 0;
            eRcvState = STATE_RX_RCV;

            /* Enable timer for character timeout. */
            vMBPortTimersEnable(SERIAL_ARG_VOID );
        }
        else
        {
            /* Frame is not okay. Delete entire frame. */
            eRcvState = STATE_RX_IDLE;
        }
        break;

    case STATE_RX_IDLE:
        if( ucByte == ':' )
        {
            #if MB_MASTER > 0
        	if(asciiMaster == TRUE)
        	{
				vMBPortTimersDisable( SERIAL_ARG_VOID );
				eSndState = ASCII_STATE_TX_IDLE;
        	}
        	#endif
            /* Enable timer for character timeout. */
            vMBPortTimersEnable(SERIAL_ARG_VOID );
            /* Reset the input buffers to store the frame. */
            usRcvBufferPos = 0;;
            eBytePos = BYTE_HIGH_NIBBLE;
            eRcvState = STATE_RX_RCV;
        }
        break;
    }

    return xNeedPoll;
}

BOOL
xMBASCIITransmitFSM( ASCII_ARG_VOID )
{
    BOOL            xNeedPoll = FALSE;
    UCHAR           ucByte;

    assert( eRcvState == STATE_RX_IDLE );
    switch ( eSndState )
    {
        /* Start of transmission. The start of a frame is defined by sending
         * the character ':'. */
    case STATE_TX_START:
        ucByte = ':';
        xMBPortSerialPutByte(SERIAL_ARG ( CHAR )ucByte );
        eSndState = STATE_TX_DATA;
        eBytePos = BYTE_HIGH_NIBBLE;
        break;

        /* Send the data block. Each data byte is encoded as a character hex
         * stream with the high nibble sent first and the low nibble sent
         * last. If all data bytes are exhausted we send a '\r' character
         * to end the transmission. */
    case STATE_TX_DATA:
        if( usSndBufferCount > 0 )
        {
            switch ( eBytePos )
            {
            case BYTE_HIGH_NIBBLE:
                ucByte = prvucMBBIN2CHAR( ( UCHAR )( *pucSndBufferCur >> 4 ) );
                xMBPortSerialPutByte(SERIAL_ARG ( CHAR ) ucByte );
                eBytePos = BYTE_LOW_NIBBLE;
                break;

            case BYTE_LOW_NIBBLE:
                ucByte = prvucMBBIN2CHAR( ( UCHAR )( *pucSndBufferCur & 0x0F ) );
                xMBPortSerialPutByte(SERIAL_ARG ( CHAR )ucByte );
                pucSndBufferCur++;
                eBytePos = BYTE_HIGH_NIBBLE;
                usSndBufferCount--;
                break;
            }
        }
        else
        {
            xMBPortSerialPutByte(SERIAL_ARG MB_ASCII_DEFAULT_CR );
            eSndState = STATE_TX_END;
        }
        break;

        /* Finish the frame by sending a LF character. */
    case STATE_TX_END:
        xMBPortSerialPutByte(SERIAL_ARG ( CHAR )ucMBLFCharacter );
        /* We need another state to make sure that the CR character has
         * been sent. */
        eSndState = STATE_TX_NOTIFY;
        break;

        /* Notify the task which called eMBASCIISend that the frame has
         * been sent. */
    case STATE_TX_NOTIFY:
        #if MB_MASTER >0
    	if(asciiMaster==TRUE)
		{

			xFrameIsBroadcast = ( ucASCIISndBuf[MB_SER_PDU_ADDR_OFF] == MB_ADDRESS_BROADCAST ) ? TRUE : FALSE;
			/* Disable transmitter. This prevents another transmit buffer
			 * empty interrupt. */
			vMBPortSerialEnable(SERIAL_ARG TRUE, FALSE );
			eSndState = ASCII_STATE_TX_XFWR;
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

		}
    	else
        #endif
    	{
			eSndState = STATE_TX_IDLE;
			xNeedPoll = xMBPortEventPost(SERIAL_ARG EV_FRAME_SENT );

			/* Disable transmitter. This prevents another transmit buffer
			 * empty interrupt. */
			vMBPortSerialEnable(SERIAL_ARG TRUE, FALSE );
			eSndState = STATE_TX_IDLE;
    	}
        break;

        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable(SERIAL_ARG TRUE, FALSE );
        break;
    }

    return xNeedPoll;
}

BOOL
xMBASCIITimerT1SExpired( ASCII_ARG_VOID )
{
    switch ( eRcvState )
    {
        /* If we have a timeout we go back to the idle state and wait for
         * the next frame.
         */
    case STATE_RX_RCV:
    case STATE_RX_WAIT_EOF:
        eRcvState = STATE_RX_IDLE;
        break;

    default:
        assert( ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_WAIT_EOF ) );
        break;
    }
    vMBPortTimersDisable( SERIAL_ARG_VOID );

    /* no context switch required. */
    return FALSE;
}


static          UCHAR
prvucMBCHAR2BIN( UCHAR ucCharacter )
{
    if( ( ucCharacter >= '0' ) && ( ucCharacter <= '9' ) )
    {
        return ( UCHAR )( ucCharacter - '0' );
    }
    else if( ( ucCharacter >= 'A' ) && ( ucCharacter <= 'F' ) )
    {
        return ( UCHAR )( ucCharacter - 'A' + 0x0A );
    }
    else
    {
        return 0xFF;
    }
}

static          UCHAR
prvucMBBIN2CHAR( UCHAR ucByte )
{
    if( ucByte <= 0x09 )
    {
        return ( UCHAR )( '0' + ucByte );
    }
    else if( ( ucByte >= 0x0A ) && ( ucByte <= 0x0F ) )
    {
        return ( UCHAR )( ucByte - 0x0A + 'A' );
    }
    else
    {
        /* Programming error. */
        assert( 0 );
    }
    return '0';
}


static          UCHAR
prvucMBLRC( UCHAR * pucFrame, USHORT usLen )
{
    UCHAR ucLRC = 0;  /* LRC char initialized */

    while( usLen-- )
    {
        ucLRC += *pucFrame++;   /* Add buffer byte without carry */
    }

    /* Return twos complement */
    ucLRC = ( UCHAR ) ( -( ( CHAR ) ucLRC ) );
    return ucLRC;
}

/* Get Modbus send PDU's buffer address pointer.*/
void vMBASCIIMasterGetPDUSndBuf( ASCII_ARG UCHAR ** pucFrame )
{
	*pucFrame = ( UCHAR * ) &ucASCIISndBuf[MB_SER_PDU_PDU_OFF];
}

#endif



#ifdef ASCII_MULTIPORT

/* Get Modbus Master send RTU's buffer address pointer.*/
void vMBASCIIMasterGetRTUSndBuf( ASCII_ARG UCHAR ** pucFrame )
{
	*pucFrame = ( UCHAR * ) ucASCIISndBuf;
}



/* Set Modbus Master send PDU's buffer length.*/
void vMBASCIIMasterSetPDUSndLength(  ASCII_ARG USHORT SendPDULength )
{
	usSendPDULength = SendPDULength;
}

/* Get Modbus Master send PDU's buffer length.*/
USHORT usMBASCIIMasterGetPDUSndLength(  ASCII_ARG_VOID )
{
	return usSendPDULength;
}

/* Set Modbus Master current timer mode.*/
void vMBASCIIMasterSetCurTimerMode( ASCII_ARG eMBMasterTimerMode eMBTimerMode )
{
	eCurTimerMode = eMBTimerMode;
}

/* The master request is broadcast? */
BOOL xMBASCIIMasterRequestIsBroadcast( ASCII_ARG_VOID )
{
	return xFrameIsBroadcast;
}

#endif
