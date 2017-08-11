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
#include <mb.h>
#include <mbcrc.h>

#define eSndState         inst->snd_state
#define eRcvState         inst->rcv_state

#define ucRTURcvBuf       inst->rcv_buf
#define ucRTUSndBuf       inst->snd_buf

#define xFrameIsBroadcast inst->frame_is_broadcast
#define eCurTimerMode     inst->cur_tmr_mode

#define rtuMaster         inst->is_master

#define pucSndBufferCur   inst->snd_buf_cur
#define usSndBufferCount  inst->snd_buff_cnt

#define txFrame           inst->txFrame
#define rxFrame           inst->rxFrame

#define usRcvBufferPos    inst->rcv_buf_pos
#define usSendPDULength   inst->snd_pdu_len

const mb_tr_mtab mb_rtu_mtab =
{
    .frm_start   = (pvMBFrameStart)  eMBRTUStart,
    .frm_stop    = (pvMBFrameStop)   eMBRTUStop,
    .frm_send    = (peMBFrameSend)   eMBRTUSend,
    .frm_rcv     = (peMBFrameReceive)eMBRTUReceive,

    .get_rx_frm      = NULL,
    .get_tx_frm      = (pvGetTxFrame)vMBRTUMasterGetPDUSndBuf
#   if MB_MASTER > 0
    , .rq_is_broadcast = (pbMBMasterRequestIsBroadcast)xMBRTUMasterRequestIsBroadcast
#   endif //master
};

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBRTUInit(MBRTUInstance* inst, BOOL is_master, UCHAR ucSlaveAddress, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    ULONG           usTimerT35_50us;

    static const mb_port_cb mb_rtu_cb =
    {
        .byte_rcvd   = (mb_fp_bool)xMBRTUReceiveFSM,
        .tx_empty    = (mb_fp_bool)xMBRTUTransmitFSM,
        .tmr_expired = (mb_fp_bool)xMBRTUTimerT35Expired
    };

    (void)ucSlaveAddress;

    inst->base.port_obj->cb  = (mb_port_cb *)&mb_rtu_cb;
    inst->base.port_obj->arg = inst;
    inst->is_master          = is_master;

    ENTER_CRITICAL_SECTION(  );

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( (mb_port_ser *)inst->base.port_obj, ulBaudRate, 8, eParity ) != TRUE )
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
        if( xMBPortTimersInit((mb_port_ser *)inst->base.port_obj,  ( USHORT ) usTimerT35_50us ) != TRUE )
        {
            eStatus = MB_EPORTERR;
        }
        eSndState = MB_RTU_TX_STATE_IDLE;
        eRcvState = MB_RTU_RX_STATE_INIT;

        //inst->serial_port.parent = (void*)(inst);
    }
    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

void
eMBRTUStart(MBRTUInstance* inst)
{
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state MB_RTU_RX_STATE_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to MB_RTU_RX_STATE_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = MB_RTU_RX_STATE_INIT;
    vMBPortSerialEnable((mb_port_ser *)inst->base.port_obj, TRUE, FALSE );
    vMBPortTimersEnable((mb_port_ser *)inst->base.port_obj );

    EXIT_CRITICAL_SECTION(  );
}

void
eMBRTUStop(MBRTUInstance* inst)
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable((mb_port_ser *)inst->base.port_obj, FALSE, FALSE );
    vMBPortTimersDisable((mb_port_ser *)inst->base.port_obj  );
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBRTUReceive(MBRTUInstance* inst, UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
    //BOOL            xFrameReceived = FALSE;
    eMBErrorCode    eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_RTU_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_RTU_SER_PDU_SIZE_MIN )
            && ( usMBCRC16( ( UCHAR * ) ucRTURcvBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTURcvBuf[MB_RTU_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_RTU_SER_PDU_PDU_OFF - MB_RTU_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucRTURcvBuf[MB_RTU_SER_PDU_PDU_OFF];

        //xFrameReceived = TRUE;
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBRTUSend(MBRTUInstance* inst, UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;

    ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if( eRcvState == MB_RTU_RX_STATE_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_RTU_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        ucRTUSndBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        ucRTUSndBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /* Activate the transmitter. */
        eSndState = MB_RTU_TX_STATE_XMIT;
        vMBPortSerialEnable((mb_port_ser *)inst->base.port_obj, FALSE, TRUE );
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
xMBRTUReceiveFSM(MBRTUInstance* inst)
{
    BOOL            xTaskNeedSwitch = FALSE;
    UCHAR           ucByte;

    assert(( eSndState == MB_RTU_TX_STATE_IDLE ) || ( eSndState == MB_RTU_TX_STATE_XFWR ));

    /* Always read the character. */
    ( void )xMBPortSerialGetByte((mb_port_ser *)inst->base.port_obj, ( CHAR * ) & ucByte );

    switch ( eRcvState )
    {
    /* If we have received a character in the init state we have to
     * wait until the frame is finished.
     */
    case MB_RTU_RX_STATE_INIT:
        vMBPortTimersEnable((mb_port_ser *)inst->base.port_obj  );
        break;

    /* In the error state we wait until all characters in the
     * damaged frame are transmitted.
     */
    case MB_RTU_RX_STATE_ERROR:
        vMBPortTimersEnable((mb_port_ser *)inst->base.port_obj  );
        break;

    /* In the idle state we wait for a new character. If a character
     * is received the t1.5 and t3.5 timers are started and the
     * receiver is in the state MB_RTU_RX_STATE_RECEIVCE.
     */
    case MB_RTU_RX_STATE_IDLE:
#if MB_MASTER > 0
        if(rtuMaster)
        {
            vMBPortTimersDisable((mb_port_ser *)inst->base.port_obj );
            eSndState = MB_RTU_TX_STATE_IDLE;
        }
#endif
        usRcvBufferPos = 0;
        ucRTURcvBuf[usRcvBufferPos++] = ucByte;
        eRcvState = MB_RTU_RX_STATE_RCV;

        /* Enable t3.5 timers. */
        vMBPortTimersEnable((mb_port_ser *)inst->base.port_obj  );
        break;

    /* We are currently receiving a frame. Reset the timer after
     * every character received. If more than the maximum possible
     * number of bytes in a modbus frame is received the frame is
     * ignored.
     */
    case MB_RTU_RX_STATE_RCV:
        if( usRcvBufferPos < MB_RTU_SER_PDU_SIZE_MAX )
        {
            ucRTURcvBuf[usRcvBufferPos++] = ucByte;
        }
        else
        {
            eRcvState = MB_RTU_RX_STATE_ERROR;
        }
        vMBPortTimersEnable((mb_port_ser *)inst->base.port_obj );
        break;
    }
    return xTaskNeedSwitch;
}

BOOL
xMBRTUTransmitFSM( MBRTUInstance* inst )
{
    BOOL            xNeedPoll = FALSE;

    assert( eRcvState == MB_RTU_RX_STATE_IDLE );

    switch ( eSndState )
    {
    /* We should not get a transmitter event if the transmitter is in
     * idle state.  */
    case MB_RTU_TX_STATE_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable((mb_port_ser *)inst->base.port_obj,  TRUE, FALSE );
        break;

    case MB_RTU_TX_STATE_XMIT:
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutByte((mb_port_ser *)inst->base.port_obj, ( CHAR )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
        }
        else
        {
#if MB_MASTER >0
            if(rtuMaster==TRUE)
            {

                xFrameIsBroadcast = ( ucRTUSndBuf[MB_RTU_SER_PDU_ADDR_OFF] == MB_ADDRESS_BROADCAST ) ? TRUE : FALSE;
                /* Disable transmitter. This prevents another transmit buffer
                 * empty interrupt. */
                vMBPortSerialEnable((mb_port_ser *)inst->base.port_obj, TRUE, FALSE );
                eSndState = MB_RTU_TX_STATE_XFWR;
                /* If the frame is broadcast ,master will enable timer of convert delay,
                 * else master will enable timer of respond timeout. */
                if ( xFrameIsBroadcast == TRUE )
                {
                    vMBPortTimersConvertDelayEnable((mb_port_ser *)inst->base.port_obj );
                }
                else
                {
                    vMBPortTimersRespondTimeoutEnable((mb_port_ser *)inst->base.port_obj );
                }

            }
            else
#endif
            {
                xNeedPoll = xMBPortEventPost((mb_port_ser *)inst->base.port_obj, EV_FRAME_SENT );
                /* Disable transmitter. This prevents another transmit buffer
                 * empty interrupt. */
                vMBPortSerialEnable((mb_port_ser *)inst->base.port_obj, TRUE, FALSE );
                eSndState = MB_RTU_TX_STATE_IDLE;
            }
        }
        break;
    default:
        break;
    }

    return xNeedPoll;
}



BOOL
xMBRTUTimerT35Expired( MBRTUInstance* inst )
{
    BOOL            xNeedPoll = FALSE;

    switch ( eRcvState )
    {
    /* Timer t35 expired. Startup phase is finished. */
    case MB_RTU_RX_STATE_INIT:
        xNeedPoll = xMBPortEventPost((mb_port_ser *)inst->base.port_obj, EV_READY );
        break;

    /* A frame was received and t35 expired. Notify the listener that
     * a new frame was received. */
    case MB_RTU_RX_STATE_RCV:
        xNeedPoll = xMBPortEventPost((mb_port_ser *)inst->base.port_obj, EV_FRAME_RECEIVED );
        break;

    /* An error occured while receiving the frame. */
    case MB_RTU_RX_STATE_ERROR:
        break;

    /* Function called in an illegal state. */
    default:
        assert( ( eRcvState == MB_RTU_RX_STATE_INIT ) ||
                ( eRcvState == MB_RTU_RX_STATE_RCV ) || ( eRcvState == MB_RTU_RX_STATE_ERROR ) );
    }

    vMBPortTimersDisable((mb_port_ser *)inst->base.port_obj  );
    eRcvState = MB_RTU_RX_STATE_IDLE;
#if MB_MASTER >0
    if(rtuMaster == TRUE)
    {
        switch (eSndState)
        {
        /* A frame was send finish and convert delay or respond timeout expired.
         * If the frame is broadcast,The master will idle,and if the frame is not
         * broadcast.Notify the listener process error.*/
        case MB_RTU_TX_STATE_XFWR:
            if ( xFrameIsBroadcast == FALSE )
            {
                ((MBInstance*)(inst->parent))->master_err_cur = ERR_EV_ERROR_RESPOND_TIMEOUT;
                //vMBSetErrorType(ERR_EV_ERROR_RESPOND_TIMEOUT); //FIXME pass reference to instance
                xNeedPoll = xMBPortEventPost((mb_port_ser *)inst->base.port_obj, EV_ERROR_PROCESS);
            }
            break;
        /* Function called in an illegal state. */
        default:
            assert(
                ( eSndState == MB_RTU_TX_STATE_XFWR ) || ( eSndState == MB_RTU_TX_STATE_IDLE ));
            break;
        }
        eSndState = MB_RTU_TX_STATE_IDLE;

        vMBPortTimersDisable((mb_port_ser *)inst->base.port_obj );
        /* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
        if (eCurTimerMode == MB_TMODE_CONVERT_DELAY)
        {
            xNeedPoll = xMBPortEventPost((mb_port_ser *)inst->base.port_obj, EV_EXECUTE );
        }
    }
#endif

    return xNeedPoll;
}

/* Get Modbus Master send PDU's buffer address pointer.*/
void vMBRTUMasterGetPDUSndBuf( MBRTUInstance* inst, UCHAR ** pucFrame )
{
    *pucFrame = ( UCHAR * ) &ucRTUSndBuf[MB_RTU_SER_PDU_PDU_OFF];
}

#if MB_MASTER > 0
/* Get Modbus send RTU's buffer address pointer.*/
void vMBMasterGetRTUSndBuf( MBRTUInstance* inst, UCHAR ** pucFrame )
{
    *pucFrame = ( UCHAR * ) ucRTUSndBuf;
}


/* Set Modbus Master send PDU's buffer length.*/
void vMBRTUMasterSetPDUSndLength(  MBRTUInstance* inst, USHORT SendPDULength )
{
    usSendPDULength = SendPDULength;
}

/* Get Modbus Master send PDU's buffer length.*/
USHORT usMBRTUMasterGetPDUSndLength(  MBRTUInstance* inst )
{
    return usSendPDULength;
}

/* Set Modbus Master current timer mode.*/
void vMBRTUMasterSetCurTimerMode( MBRTUInstance* inst, eMBMasterTimerMode eMBTimerMode )
{
    eCurTimerMode = eMBTimerMode;
}

/* The master request is broadcast? */
BOOL xMBRTUMasterRequestIsBroadcast( MBRTUInstance* inst )
{
    return xFrameIsBroadcast;
}
#endif
