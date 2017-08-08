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
//#include <stdlib.h>
//#include <string.h>
/* ----------------------- Modbus includes ----------------------------------*/
#include <mb.h>

//#include <mbconfig.h>
//#include <mb_types.h>
//
//#include <serial_port.h>
//
//#include <mbport.h>
//#include <mbframe.h>
//#include <mbascii.h>
//#include <mb.h>
//#include <mbcrc.h>

#if MB_ASCII_ENABLED > 0

#define eSndState         inst->snd_state
#define eRcvState         inst->rcv_state

#define ucASCIIRcvBuf     inst->rcv_buf
#define ucASCIISndBuf     inst->snd_buf

#define usRcvBufferPos    inst->rcv_buf_pos
#define eBytePos          inst->byte_pos
#define xFrameIsBroadcast inst->frame_is_broadcast
#define eCurTimerMode     inst->cur_tmr_mode
#define usSendPDULength   inst->snd_pdu_len
#define pucSndBufferCur   inst->snd_buf_cur
#define usSndBufferCount  inst->snd_buf_cnt

#define asciiMaster       inst->is_master
#define ucMBLFCharacter   inst->mb_lf_char
/* ----------------------- Static functions ---------------------------------*/
static UCHAR    prvucMBCHAR2BIN(UCHAR ucCharacter             );
static UCHAR    prvucMBBIN2CHAR(UCHAR ucByte                  );
static UCHAR    prvucMBLRC     (UCHAR * pucFrame, USHORT usLen);
/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode eMBASCIIInit(MBASCIIInstance* inst, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    ( void )ucSlaveAddress;
    usSndBufferCount=0;

    ENTER_CRITICAL_SECTION(  );
    ucMBLFCharacter = MB_ASCII_DEFAULT_LF;

    if( xMBPortSerialInit(&(inst->serial_port), ucPort, ulBaudRate, 7, eParity ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }
    else if( xMBPortTimersInit(&(inst->serial_port), MB_ASCII_TIMEOUT_SEC * 20000UL ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }

    eRcvState =  MB_ASCII_RX_STATE_IDLE;
    eSndState = MB_ASCII_TX_STATE_IDLE;
    //inst->serial_port.parent = (void*)(inst);

    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

void eMBASCIIStart(MBASCIIInstance* inst)
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable(&(inst->serial_port), TRUE, FALSE );
    eRcvState = MB_ASCII_RX_STATE_IDLE;
    EXIT_CRITICAL_SECTION(  );

    /* No special startup required for ASCII. */
    ( void )xMBPortEventPost(&(inst->serial_port), EV_READY );
}

void eMBASCIIStop(MBASCIIInstance* inst)
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable(&(inst->serial_port), FALSE, FALSE );
    vMBPortTimersDisable(&(inst->serial_port));
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode eMBASCIIReceive(MBASCIIInstance* inst,  UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength)
{
    eMBErrorCode    eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_ASCII_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_ASCII_SER_PDU_SIZE_MIN )
            && ( prvucMBLRC( ( UCHAR * ) ucASCIIRcvBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucASCIIRcvBuf[MB_ASCII_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_ASCII_SER_PDU_PDU_OFF - MB_ASCII_SER_PDU_SIZE_LRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucASCIIRcvBuf[MB_ASCII_SER_PDU_PDU_OFF];
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode eMBASCIISend(MBASCIIInstance* inst,  UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    UCHAR           usLRC;

    ENTER_CRITICAL_SECTION(  );
    /* Check if the receiver is still in idle state. If not we where too
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if( eRcvState == MB_ASCII_RX_STATE_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_ASCII_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate LRC checksum for Modbus-Serial-Line-PDU. */
        usLRC = prvucMBLRC( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        ucASCIISndBuf[usSndBufferCount++] = usLRC;

        /* Activate the transmitter. */
        eSndState = MB_ASCII_TX_STATE_START;
        vMBPortSerialEnable(&(inst->serial_port), FALSE, TRUE );
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL xMBASCIIReceiveFSM(MBASCIIInstance* inst)
{
    BOOL            xNeedPoll = FALSE;
    UCHAR           ucByte;
    UCHAR           ucResult;

    assert(( eSndState == MB_ASCII_TX_STATE_IDLE )|| ( eSndState == MB_ASCII_TX_STATE_XFWR ));

    ( void )xMBPortSerialGetByte(&(inst->serial_port), ( CHAR * ) & ucByte );
    switch ( eRcvState )
    {
        /* A new character is received. If the character is a ':' the input
         * buffer is cleared. A CR-character signals the end of the data
         * block. Other characters are part of the data block and their
         * ASCII value is converted back to a binary representation.
         */
    case MB_ASCII_RX_STATE_RCV:
        /* Enable timer for character timeout. */
        vMBPortTimersEnable( &(inst->serial_port) );
        if( ucByte == ':' )
        {
            /* Empty receive buffer. */
            eBytePos = BYTE_HIGH_NIBBLE;
            usRcvBufferPos = 0;
        }
        else if( ucByte == MB_ASCII_DEFAULT_CR )
        {
            eRcvState = MB_ASCII_RX_STATE_WAIT_EOF;
        }
        else
        {
            ucResult = prvucMBCHAR2BIN( ucByte );
            switch ( eBytePos )
            {
                /* High nibble of the byte comes first. We check for
                 * a buffer overflow here. */
            case BYTE_HIGH_NIBBLE:
                if( usRcvBufferPos < MB_ASCII_SER_PDU_SIZE_MAX )
                {
                    ucASCIIRcvBuf[usRcvBufferPos] = ( UCHAR )( ucResult << 4 );
                    eBytePos = BYTE_LOW_NIBBLE;
                    break;
                }
                else
                {
                    /* not handled in Modbus specification but seems
                     * a resonable implementation. */
                    eRcvState = MB_ASCII_RX_STATE_IDLE;
                    /* Disable previously activated timer because of error state. */
                    vMBPortTimersDisable(&(inst->serial_port));
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

    case MB_ASCII_RX_STATE_WAIT_EOF:
        if( ucByte == ucMBLFCharacter )
        {
            /* Disable character timeout timer because all characters are
             * received. */
            vMBPortTimersDisable(&(inst->serial_port)  );
            /* Receiver is again in idle state. */
            eRcvState = MB_ASCII_RX_STATE_IDLE;

            /* Notify the caller of eMBASCIIReceive that a new frame
             * was received. */
            xNeedPoll = xMBPortEventPost(&(inst->serial_port), EV_FRAME_RECEIVED );
        }
        else if( ucByte == ':' )
        {
            /* Empty receive buffer and back to receive state. */
            eBytePos = BYTE_HIGH_NIBBLE;
            usRcvBufferPos = 0;
            eRcvState = MB_ASCII_RX_STATE_RCV;

            /* Enable timer for character timeout. */
            vMBPortTimersEnable(&(inst->serial_port) );
        }
        else
        {
            /* Frame is not okay. Delete entire frame. */
            eRcvState = MB_ASCII_RX_STATE_IDLE;
        }
        break;

    case MB_ASCII_RX_STATE_IDLE:
        if( ucByte == ':' )
        {
#if MB_MASTER > 0
            if(asciiMaster == TRUE)
            {
                vMBPortTimersDisable( &(inst->serial_port) );
                eSndState = MB_ASCII_TX_STATE_IDLE;
            }
#endif
            /* Enable timer for character timeout. */
            vMBPortTimersEnable(&(inst->serial_port) );
            /* Reset the input buffers to store the frame. */
            usRcvBufferPos = 0;;
            eBytePos = BYTE_HIGH_NIBBLE;
            eRcvState = MB_ASCII_RX_STATE_RCV;
        }
        break;
    }

    return xNeedPoll;
}

BOOL xMBASCIITransmitFSM(MBASCIIInstance* inst)
{
    BOOL            xNeedPoll = FALSE;
    UCHAR           ucByte;

    assert( eRcvState == MB_ASCII_RX_STATE_IDLE );
    switch ( eSndState )
    {
        /* Start of transmission. The start of a frame is defined by sending
         * the character ':'. */
    case MB_ASCII_TX_STATE_START:
        ucByte = ':';
        xMBPortSerialPutByte(&(inst->serial_port), ( CHAR )ucByte );
        eSndState = MB_ASCII_TX_STATE_DATA;
        eBytePos = BYTE_HIGH_NIBBLE;
        break;

        /* Send the data block. Each data byte is encoded as a character hex
         * stream with the high nibble sent first and the low nibble sent
         * last. If all data bytes are exhausted we send a '\r' character
         * to end the transmission. */
    case MB_ASCII_TX_STATE_DATA:
        if( usSndBufferCount > 0 )
        {
            switch ( eBytePos )
            {
            case BYTE_HIGH_NIBBLE:
                ucByte = prvucMBBIN2CHAR( ( UCHAR )( *pucSndBufferCur >> 4 ) );
                xMBPortSerialPutByte(&(inst->serial_port), ( CHAR ) ucByte );
                eBytePos = BYTE_LOW_NIBBLE;
                break;

            case BYTE_LOW_NIBBLE:
                ucByte = prvucMBBIN2CHAR( ( UCHAR )( *pucSndBufferCur & 0x0F ) );
                xMBPortSerialPutByte(&(inst->serial_port), ( CHAR )ucByte );
                pucSndBufferCur++;
                eBytePos = BYTE_HIGH_NIBBLE;
                usSndBufferCount--;
                break;
            }
        }
        else
        {
            xMBPortSerialPutByte(&(inst->serial_port), MB_ASCII_DEFAULT_CR );
            eSndState = MB_ASCII_TX_STATE_END;
        }
        break;

        /* Finish the frame by sending a LF character. */
    case MB_ASCII_TX_STATE_END:
        xMBPortSerialPutByte(&(inst->serial_port), ( CHAR )ucMBLFCharacter );
        /* We need another state to make sure that the CR character has
         * been sent. */
        eSndState = MB_ASCII_TX_STATE_NOTIFY;
        break;

        /* Notify the task which called eMBASCIISend that the frame has
         * been sent. */
    case MB_ASCII_TX_STATE_NOTIFY:
#if MB_MASTER >0
        if(asciiMaster==TRUE)
        {

            xFrameIsBroadcast = ( ucASCIISndBuf[MB_ASCII_SER_PDU_ADDR_OFF] == MB_ADDRESS_BROADCAST ) ? TRUE : FALSE;
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable(&(inst->serial_port), TRUE, FALSE );
            eSndState = MB_ASCII_TX_STATE_XFWR;
            /* If the frame is broadcast ,master will enable timer of convert delay,
             * else master will enable timer of respond timeout. */
            if ( xFrameIsBroadcast == TRUE )
            {
                vMBPortTimersConvertDelayEnable( &(inst->serial_port) );
            }
            else
            {
                vMBPortTimersRespondTimeoutEnable( &(inst->serial_port) );
            }

        }
        else
#endif
        {
            eSndState = MB_ASCII_TX_STATE_IDLE;
            xNeedPoll = xMBPortEventPost(&(inst->serial_port), EV_FRAME_SENT );

            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable(&(inst->serial_port), TRUE, FALSE );
            eSndState = MB_ASCII_TX_STATE_IDLE;
        }
        break;

        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case MB_ASCII_TX_STATE_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable(&(inst->serial_port), TRUE, FALSE );
        break;
    }

    return xNeedPoll;
}

BOOL xMBASCIITimerT1SExpired(MBASCIIInstance* inst)
{
    switch ( eRcvState )
    {
        /* If we have a timeout we go back to the idle state and wait for
         * the next frame.
         */
    case MB_ASCII_RX_STATE_RCV:
    case MB_ASCII_RX_STATE_WAIT_EOF:
        eRcvState = MB_ASCII_RX_STATE_IDLE;
        break;

    default:
        assert( ( eRcvState == MB_ASCII_RX_STATE_RCV ) || ( eRcvState == MB_ASCII_RX_STATE_WAIT_EOF ) );
        break;
    }
    vMBPortTimersDisable( &(inst->serial_port) );

    /* no context switch required. */
    return FALSE;
}


static UCHAR prvucMBCHAR2BIN(UCHAR ucCharacter)
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

static UCHAR prvucMBBIN2CHAR(UCHAR ucByte)
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


static UCHAR prvucMBLRC( UCHAR * pucFrame, USHORT usLen )
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
void vMBASCIIMasterGetPDUSndBuf( MBASCIIInstance* inst, UCHAR ** pucFrame )
{
    *pucFrame = ( UCHAR * ) &ucASCIISndBuf[MB_ASCII_SER_PDU_PDU_OFF];
}



/* Get Modbus Master send RTU's buffer address pointer.*/
void vMBASCIIMasterGetRTUSndBuf( MBASCIIInstance* inst, UCHAR ** pucFrame )
{
    *pucFrame = ( UCHAR * ) ucASCIISndBuf;
}

/* Set Modbus Master send PDU's buffer length.*/
void vMBASCIIMasterSetPDUSndLength(  MBASCIIInstance* inst, USHORT SendPDULength )
{
    usSendPDULength = SendPDULength;
}

/* Get Modbus Master send PDU's buffer length.*/
USHORT usMBASCIIMasterGetPDUSndLength(  MBASCIIInstance* inst )
{
    return usSendPDULength;
}

/* Set Modbus Master current timer mode.*/
void vMBASCIIMasterSetCurTimerMode( MBASCIIInstance* inst, eMBMasterTimerMode eMBTimerMode )
{
    eCurTimerMode = eMBTimerMode;
}

/* The master request is broadcast? */
BOOL xMBASCIIMasterRequestIsBroadcast( MBASCIIInstance* inst )
{
    return xFrameIsBroadcast;
}

#endif
