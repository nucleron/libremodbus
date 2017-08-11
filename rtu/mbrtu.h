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

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Defines ------------------------------------------*/
#define MB_RTU_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_RTU_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_RTU_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_RTU_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_RTU_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    MB_RTU_RX_STATE_INIT,              /*!< Receiver is in initial state. */
    MB_RTU_RX_STATE_IDLE,              /*!< Receiver is in idle state. */
    MB_RTU_RX_STATE_RCV,               /*!< Frame is beeing received. */
    MB_RTU_RX_STATE_ERROR              /*!< If the frame is invalid. */
} mb_rtu_rcv_state_enum;

typedef enum
{
    MB_RTU_TX_STATE_IDLE,              /*!< Transmitter is in idle state. */
    MB_RTU_TX_STATE_XMIT,              /*!< Transmitter is in transfer state. */
    MB_RTU_TX_STATE_XFWR
} mb_rtu_snd_state_enum;

typedef struct
{
    mb_trans_base                  base;
    void                           *parent;
    //mb_port_ser               serial_port;
    volatile mb_rtu_snd_state_enum snd_state;
    volatile mb_rtu_rcv_state_enum rcv_state;

    volatile UCHAR                 snd_buf[MB_PDU_SIZE_MAX];
    volatile UCHAR                 rcv_buf[MB_RTU_SER_PDU_SIZE_MAX];
    volatile USHORT                snd_pdu_len;

    volatile UCHAR                 *snd_buf_cur;
    volatile USHORT                snd_buff_cnt;

    volatile USHORT                rcv_buf_pos;

    BOOL                           is_master;
    BOOL                           frame_is_broadcast;
    volatile eMBMasterTimerMode    cur_tmr_mode;
} MBRTUInstance;

extern const mb_tr_mtab mb_rtu_mtab;

eMBErrorCode            eMBRTUInit                    (MBRTUInstance* inst, BOOL is_master, UCHAR slaveAddress, ULONG ulBaudRate, eMBParity eParity);
void                    eMBRTUStart                   (MBRTUInstance* inst                                                                       );
void                    eMBRTUStop                    (MBRTUInstance* inst                                                                       );
eMBErrorCode            eMBRTUReceive                 (MBRTUInstance* inst, UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength         );
eMBErrorCode            eMBRTUSend                    (MBRTUInstance* inst, UCHAR slaveAddress, const UCHAR * pucFrame, USHORT usLength          );
BOOL                    xMBRTUReceiveFSM              (MBRTUInstance* inst                                                                       );
BOOL                    xMBRTUTransmitFSM             (MBRTUInstance* inst                                                                       );
BOOL                    xMBRTUTimerT15Expired         (MBRTUInstance* inst                                                                       );
BOOL                    xMBRTUTimerT35Expired         (MBRTUInstance* inst                                                                       );
//master
void                    vMBRTUMasterGetPDUSndBuf      (MBRTUInstance* inst, UCHAR ** pucFrame                                                    );
USHORT                  usMBRTUMasterGetPDUSndLength  (MBRTUInstance* inst                                                                       );
void                    vMBRTUMasterSetPDUSndLength   (MBRTUInstance* inst, USHORT SendPDULength                                                 );
void                    vMBRTUMasterSetCurTimerMode   (MBRTUInstance* inst, eMBMasterTimerMode eMBTimerMode                                      );
BOOL                    xMBRTUMasterRequestIsBroadcast(MBRTUInstance* inst                                                                       );
eMBMasterErrorEventType eMBRTUMasterGetErrorType      (MBRTUInstance* inst                                                                       );
eMBMasterReqErrCode     eMBRTUMasterWaitRequestFinish (void /*Какого ???*/                                                                       );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
