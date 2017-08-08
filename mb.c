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
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */
#include <mb.h>

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

#define ucMBAddress inst->address
#define eMBCurrentMode inst->cur_mode
#define eMBCurrentState inst->cur_state

#define pvMBFrameStartCur               inst->trmt->frm_start
#define peMBFrameSendCur                inst->trmt->frm_send
#define pvMBFrameStopCur                inst->trmt->frm_stop
#define peMBFrameReceiveCur             inst->trmt->frm_rcv

#define pvMBGetRxFrame                  inst->trmt->get_rx_frm
#define pvMBGetTxFrame                  inst->trmt->get_tx_frm

#define pbMBMasterRequestIsBroadcastCur inst->trmt->rq_is_broadcast

#define xFuncHandlers inst->xFuncHandlers

#define usLength inst->len
#define PDUSndLength inst->pdu_snd_len
#define rxFrame inst->rx_frame
#define txFrame inst->tx_frame

#define pvMBFrameCloseCur  (inst->pmt->frm_close)
#define pvPortEventPostCur (inst->pmt->evt_post)
#define pvPortEventGetCur  (inst->pmt->evt_get)

//master variables
#define ucMBMasterDestAddress inst->master_dst_addr
#define xMBRunInMasterMode    inst->master_mode_run
#define eMBMasterCurErrorType inst->master_err_cur

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler defaultFuncHandlers[MB_FUNC_HANDLERS_MAX] =
{
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, (void*)eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, (void*)eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, (void*)eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, (void*)eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, (void*)eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, (void*)eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, (void*)eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, (void*)eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, (void*)eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, (void*)eMBFuncReadDiscreteInputs},
#endif
};

#if MB_MASTER >0
static xMBFunctionHandler xMasterFuncHandlers[MB_FUNC_HANDLERS_MAX] =
{
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    //TODO Add Master function define
    {MB_FUNC_OTHER_REPORT_SLAVEID, (void*)eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, (void*)eMBMasterFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, (void*)eMBMasterFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, (void*)eMBMasterFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, (void*)eMBMasterFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, (void*)eMBMasterFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, (void*)eMBMasterFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, (void*)eMBMasterFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, (void*)eMBMasterFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, (void*)eMBMasterFuncReadDiscreteInputs},
#endif
};
#endif

/* ----------------------- Start implementation -----------------------------*/
#if MB_RTU_ENABLED > 0
eMBErrorCode
eMBInitRTU(MBInstance* inst, MBRTUInstance* transport, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    return eMBInit(inst,(void*)transport, MB_RTU, ucSlaveAddress, ucPort,ulBaudRate, eParity );
}
#endif

#if MB_ASCII_ENABLED > 0
eMBErrorCode
eMBInitASCII(MBInstance* inst, MBASCIIInstance* transport, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{

    return eMBInit(inst,(void*)transport, MB_ASCII, ucSlaveAddress, ucPort,ulBaudRate, eParity );
}

#endif


#if MB_MASTER >0

#if MB_RTU_ENABLED > 0
eMBErrorCode
eMBMasterInitRTU(MBInstance* inst, MBRTUInstance* transport, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    transport->is_master = TRUE;
    return eMBInit(inst,(void*)transport, MB_RTU, 0, ucPort,ulBaudRate, eParity );
}
#endif

#if MB_ASCII_ENABLED > 0
eMBErrorCode
eMBMasterInitASCII(MBInstance* inst, MBASCIIInstance* transport, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    transport->asciiMaster = TRUE;
    return eMBInit(inst,(void*)transport, MB_ASCII, 0, ucPort,ulBaudRate, eParity );
}

#endif

#if MB_TCP_ENABLED > 0
eMBErrorCode eMBMasterInitTCP(MBInstance* inst, MBTCPInstance* transport, USHORT ucTCPPort, SOCKADDR_IN hostaddr )
{
    transport->tcpMaster = TRUE;
    return eMBTCPInit(inst,transport,ucTCPPort,hostaddr,TRUE);
}

#endif

#endif //MASTER

#if MB_RTU_ENABLED || MB_ASCII_ENABLED
eMBErrorCode
eMBInit(MBInstance *inst, void* transport, eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity)
{
    inst->transport = transport;
    eMBCurrentState = STATE_NOT_INITIALIZED;
    eMBErrorCode    eStatus = MB_ENOERR;
    BOOL isMaster = FALSE;

    int i;

    switch ( eMode )
    {
#if MB_RTU_ENABLED > 0
    case MB_RTU:
    {
        //TODO: move to RTU init
        static const mb_port_cb mb_rtu_cb =
        {
            .byte_rcvd   = (mb_fp_bool)xMBRTUReceiveFSM,
            .tx_empty    = (mb_fp_bool)xMBRTUTransmitFSM,
            .tmr_expired = (mb_fp_bool)xMBRTUTimerT35Expired
        };
        ((MBRTUInstance *)transport)->serial_port.base.cb  = &mb_rtu_cb;
        ((MBRTUInstance *)transport)->serial_port.base.arg = transport;

        //TODO: place const tu mb_rtu.c
        static const mb_tr_mtab mb_rtu_mtab =
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
        inst->trmt = (mb_tr_mtab *)&mb_rtu_mtab;

        inst->port = &(((MBRTUInstance*)transport)->serial_port);
        PDUSndLength = &(((MBRTUInstance*)transport)->snd_pdu_len);
        isMaster = ((MBRTUInstance*)transport)->is_master;


        eStatus = eMBRTUInit((MBRTUInstance*)transport, ucMBAddress, ucPort, ulBaudRate, eParity );
        ((MBRTUInstance*)(transport))->parent = (void*)(inst);
        break;
    }
#endif //RTU
#if MB_ASCII_ENABLED > 0
    case MB_ASCII:
    {
        //TODO: move to ASCII init
        static const mb_port_cb mb_ascii_cb =
        {
            .byte_rcvd   = (mb_fp_bool)xMBASCIIReceiveFSM,
            .tx_empty    = (mb_fp_bool)xMBASCIITransmitFSM,
            .tmr_expired = (mb_fp_bool)xMBASCIITimerT1SExpired
        };
        ((MBASCIIInstance *)transport)->serial_port.base.cb  = &mb_ascii_cb;
        ((MBASCIIInstance *)transport)->serial_port.base.arg = transport;

        //TODO: place const tu mb_ascii.c
        static const mb_tr_mtab mb_ascii_mtab =
        {
            .frm_start   = (pvMBFrameStart)  eMBASCIIStart,
            .frm_stop    = (pvMBFrameStop)   eMBASCIIStop,
            .frm_send    = (peMBFrameSend)   eMBASCIISend,
            .frm_rcv     = (peMBFrameReceive)eMBASCIIReceive,

            .get_rx_frm      = NULL,
            .get_tx_frm      = (pvGetTxFrame)vMBASCIIMasterGetPDUSndBuf
#   if MB_MASTER > 0
            , .rq_is_broadcast = (pbMBMasterRequestIsBroadcast)xMBASCIIMasterRequestIsBroadcast
#   endif //master
        };
        inst->trmt = (mb_tr_mtab *)&mb_ascii_mtab;

        inst->port = &(((MBASCIIInstance*)transport)->serial_port);
        PDUSndLength = &(((MBASCIIInstance*)transport)->snd_pdu_len);
        isMaster = ((MBASCIIInstance*)transport)->is_master;

        eStatus = eMBASCIIInit((MBASCIIInstance*)transport, ucMBAddress, ucPort, ulBaudRate, eParity );
        ((MBASCIIInstance*)(transport))->parent = (void*)(inst);
        break;
    }
#endif//ASCII
    default:
        eStatus = MB_EINVAL;
    }

#if (MB_PORT_HAS_CLOSE > 0)
#   define MB_SERIAL_CLOSE vMBPortClose
#else
#   define MB_SERIAL_CLOSE NULL
#endif // MB_PORT_HAS_CLOSE
    static const mb_port_mtab mb_serial_mtab =
    {
        .frm_close = (pvMBFrameClose) MB_SERIAL_CLOSE,
        .evt_post  = (pvPortEventPost)xMBPortEventPost,
        .evt_get   = (pvPortEventGet) xMBPortEventGet
    };
    inst->pmt = (mb_port_mtab *)&mb_serial_mtab;

    pvMBGetTxFrame(inst->transport, (void*)&(txFrame));      //Можно было прописать сразу.

#if MB_MASTER > 0
    if(isMaster == TRUE)
    {

        xMBRunInMasterMode = TRUE;
        xFuncHandlers = xMasterFuncHandlers;
    }
    else
#endif
    {
        xFuncHandlers = defaultFuncHandlers;
    }

    /* check preconditions */
    if(( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
            ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )&& (isMaster == FALSE))
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        ucMBAddress = ucSlaveAddress;



        if( eStatus == MB_ENOERR )
        {
            if( !xMBPortEventInit( (mb_port_ser*)inst->port ) )
            {
                /* port dependent event module initalization failed. */
                eStatus = MB_EPORTERR;
            }
            else
            {
                eMBCurrentMode = eMode;
                eMBCurrentState = STATE_DISABLED;
            }
        }
    }

    //if(isMaster == TRUE)
    //vMBMasterOsResInit(); //FIXME: what is it?

    return eStatus;
}
#endif

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit(MBInstance* inst, MBTCPInstance* transport, USHORT ucTCPPort, SOCKADDR_IN hostaddr, BOOL bMaster  )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    inst->transport = transport;
    transport->parent = (void*)(inst);
    int i;
    if(transport->tcpMaster ==  TRUE)
    {
        inst->xMBRunInMasterMode = TRUE;
        xFuncHandlers = xMasterFuncHandlers;
//        for(i =0; i<MB_FUNC_HANDLERS_MAX; i++)
//            inst->xFuncHandlers[i]=xMasterFuncHandlers[i];
    }
    else
    {
        xFuncHandlers = defaultFuncHandlers;
//        for(i =0; i<MB_FUNC_HANDLERS_MAX; i++)
//            inst->xFuncHandlers[i]=defaultFuncHandlers[i];
    }
    if( ( eStatus = eMBTCPDoInit(transport, ucTCPPort, hostaddr, bMaster  ) ) != MB_ENOERR )
    {
        inst->eMBCurrentState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit( &(transport->tcp_port) ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        //TODO: place const tu mb_ascii.c
        static const mb_tr_mtab mb_tcp_mtab =
        {
            .frm_start   = (pvMBFrameStart)  eMBTCPStart,
            .frm_stop    = (pvMBFrameStop)   eMBTCPStop,
            .frm_send    = (peMBFrameSend)   eMBTCPSend,
            .frm_rcv     = (peMBFrameReceive)eMBTCPReceive,

            .get_rx_frm      = (pvGetRxFrame)vMBTCPMasterGetPDURcvBuf,
            .get_tx_frm      = (pvGetTxFrame)vMBTCPMasterGetPDUSndBuf
#   if MB_MASTER > 0
            , .rq_is_broadcast = (pbMBMasterRequestIsBroadcast)xMBTCPMasterRequestIsBroadcast
#   endif //master
        };
        inst->trmt = (mb_tr_mtab *)&mb_tcp_mtab;

        inst->ucMBAddress = MB_TCP_PSEUDO_ADDRESS;
        inst->cur_mode = MB_TCP;
        inst->eMBCurrentState = STATE_DISABLED;
        inst->pdu_snd_len = &(((MBTCPInstance*)transport)->usSendPDULength);
        inst->port = (void*) &(((MBTCPInstance*)transport)->tcp_port);

        inst->trmt->get_rx_frm(inst->transport,&inst->rxFrame);
        inst->trmt->get_tx_frm(inst->transport,&inst->txFrame);//Зачем 2 раза???

#if (MB_PORT_HAS_CLOSE > 0)
#   define MB_TCP_CLOSE vMBPortClose
#else
#   define MB_TCP_CLOSE NULL
#endif // MB_PORT_HAS_CLOSE
        static const mb_port_mtab mb_tcp_mtab =
        {
            .frm_close = (pvMBFrameClose) MB_TCP_CLOSE,
            .evt_post  = (pvPortEventPost)xMBPortEventPost,
            .evt_get   = (pvPortEventGet) xMBPortEventGet
        };
        inst->pmt = (mb_port_mtab *)&mb_tcp_mtab;
    }
    return eStatus;
}
#endif

eMBErrorCode
eMBClose(MBInstance* inst)
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBCurrentState == STATE_DISABLED )
    {
        if( pvMBFrameCloseCur != NULL )
        {
            pvMBFrameCloseCur( inst->transport );
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBEnable(MBInstance* inst)
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBCurrentState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        pvMBFrameStartCur( inst->transport );
        eMBCurrentState = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable(MBInstance* inst)
{
    eMBErrorCode    eStatus;

    if( eMBCurrentState == STATE_ENABLED )
    {
        pvMBFrameStopCur(inst->transport );
        eMBCurrentState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if(eMBCurrentState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBPoll(MBInstance* inst)
{
    static UCHAR    ucRcvAddress;
    static UCHAR    ucFunctionCode;
    static eMBException eException;
    eMBMasterErrorEventType errorType;

    int             i,j;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    /* Check if the protocol stack is ready. */
    if( eMBCurrentState != STATE_ENABLED )
    {
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    BOOL gotEvent;

    gotEvent = pvPortEventGetCur(inst->port, inst, &eEvent );

    if(gotEvent  == TRUE )
    {
        switch ( eEvent )
        {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            eStatus = peMBFrameReceiveCur(inst->transport, &ucRcvAddress,(UCHAR**) &rxFrame, (USHORT*)&usLength );
            if( eStatus == MB_ENOERR )
            {
#if MB_MASTER > 0
                if(xMBRunInMasterMode == TRUE)
                {
                    if(ucRcvAddress == ucMBMasterDestAddress || eMBCurrentMode== MB_TCP) //All addresses work in tcp mode
                    {
                        ( void ) pvPortEventPostCur(inst->port, EV_EXECUTE );
                    }
                }
                else
#endif // MB_MASTER
                {
                    /* Check if the frame is for us. If not ignore the frame. */
                    if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                    {
                        ( void ) pvPortEventPostCur(inst->port, EV_EXECUTE );
                    }
                }

            }
            else
            {
#if MB_MASTER > 0
                if(xMBRunInMasterMode)
                {
                    eMBMasterCurErrorType = ERR_EV_ERROR_RECEIVE_DATA;
                    ( void ) pvPortEventPostCur(inst->port, EV_ERROR_PROCESS );
                }
#endif // MB_MASTER
            }
            break;

        case EV_EXECUTE:
            ucFunctionCode = rxFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;

#if MB_MASTER > 0
            if((ucFunctionCode >> 7) && xMBRunInMasterMode)
            {
                eException = (eMBException)rxFrame[MB_PDU_DATA_OFF];
            }
            else
#endif // MB_MASTER
            {
                for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
                {
                    /* No more function handlers registered. Abort. */
                    if( xFuncHandlers[i].ucFunctionCode == 0 )
                    {
                        break;
                    }
                    else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                    {
#if MB_MASTER > 0
                        if (pbMBMasterRequestIsBroadcastCur(inst->transport) && xMBRunInMasterMode )
                        {

                            for(j = 1; j <= MB_MASTER_TOTAL_SLAVE_NUM; j++)
                            {
                                ucMBMasterDestAddress =j;
                                eException = xFuncHandlers[i].pxHandler(inst,(UCHAR*)(rxFrame), (USHORT*)&usLength);
                            }
                        }
                        else
#endif
                        {
#if MB_MASTER > 0
                            if(xMBRunInMasterMode == FALSE)
#endif// MB_MASTER
                            {
                                for(j=0; j<usLength; j++)
                                    txFrame[j]=rxFrame[j];
                                eException = xFuncHandlers[i].pxHandler(inst, (UCHAR*)(txFrame),(USHORT*)&usLength);
                            }
#if MB_MASTER > 0
                            else
                                eException = xFuncHandlers[i].pxHandler(inst, (UCHAR*)(rxFrame), (USHORT*)&usLength);
#endif// MB_MASTER
                        }
                        break;
                    }
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */
#if MB_MASTER > 0
            if(xMBRunInMasterMode)
            {
                if (eException != MB_EX_NONE)
                {
                    eMBMasterCurErrorType =  ERR_EV_ERROR_EXECUTE_FUNCTION;
                    ( void )pvPortEventPostCur(inst->port, EV_ERROR_PROCESS );
                }
                else
                {
                    vMBMasterCBRequestScuuess(inst->transport );
                    //vMBMasterRunResRelease(inst->transport ); //FIXME
                }
            }
            else
#endif
            {
                if( ucRcvAddress != MB_ADDRESS_BROADCAST )
                {
                    if( eException != MB_EX_NONE )
                    {
                        /* An exception occured. Build an error frame. */
                        usLength = 0;
                        txFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
                        txFrame[usLength++] = eException;
                    }
                    if( ( eMBCurrentMode == MB_ASCII ) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS )
                    {
                        vMBPortTimersDelay(inst->port, MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS );
                    }
                    eStatus = peMBFrameSendCur(inst->transport, ucMBAddress,(UCHAR*)(txFrame), usLength );
                }
            }

            break;

        case EV_FRAME_SENT:
            /* Master is busy now. */
#if MB_MASTER > 0
            if(xMBRunInMasterMode)
            {
                pvMBGetTxFrame(inst->transport, (UCHAR**)(&txFrame));
                eStatus = peMBFrameSendCur(inst->transport, ucMBMasterDestAddress,(UCHAR*)(txFrame), *PDUSndLength );
            }
#endif
            break;

        case EV_ERROR_PROCESS:
#if MB_MASTER > 0
            /* Execute specified error process callback function. */
            errorType = eMBMasterCurErrorType;
            //vMBRTUMasterGetPDUSndBuf(inst->transport, &ucMBFrame );
            switch (errorType)
            {
            case ERR_EV_ERROR_RESPOND_TIMEOUT:
                vMBMasterErrorCBRespondTimeout(ucMBMasterDestAddress,
                                               txFrame, *PDUSndLength);
                break;
            case ERR_EV_ERROR_RECEIVE_DATA:
                vMBMasterErrorCBReceiveData(ucMBMasterDestAddress,
                                            txFrame, *PDUSndLength);
                break;
            case ERR_EV_ERROR_EXECUTE_FUNCTION:
                vMBMasterErrorCBExecuteFunction(ucMBMasterDestAddress,
                                                txFrame, *PDUSndLength);
                break;
            }
#endif
            //vMBMasterRunResRelease(); FIXME
            break;
        }
    }
    return MB_ENOERR;
}

