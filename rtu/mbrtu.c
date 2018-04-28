/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2016, 2017 Nucleron R&D LLC <main@nucleron.ru>
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
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
 * File: $Id: mbrtu.c, v 1.18 2007/09/12 10:15:56 wolti Exp $
 */
#include <mb.h>
#include <mbcrc.h>

const mb_tr_mtab mb_rtu_mtab =
{
    .frm_start   = (mb_frm_start_fp) mb_rtu_start,
    .frm_stop    = (mb_frm_stop_fp)  mb_rtu_stop,
    .frm_send    = (mb_frm_snd_fp)   mb_rtu_send,
    .frm_rcv     = (mb_frm_rcv_fp)   mb_rtu_receive,

    .get_rx_frm  = NULL,
    .get_tx_frm  = (mb_get_tx_frm_fp)mb_rtu_get_snd_buf
#   if MB_MASTER > 0
    , .rq_is_broadcast = (mb_mstr_rq_is_bcast_fp)mb_rtu_rq_is_bcast
#   endif //master
};

/* ----------------------- Start implementation -----------------------------*/
mb_err_enum  mb_rtu_init(mb_rtu_tr_struct* inst, BOOL is_master, UCHAR slv_addr, ULONG baud, mb_port_ser_parity_enum parity)
{
    mb_err_enum    status = MB_ENOERR;
    ULONG           tmr_35_50us;

    static const mb_port_cb_struct mb_rtu_cb =
    {
        .byte_rcvd   = (mb_port_cb_fp)mb_rtu_rcv_fsm,
        .tx_empty    = (mb_port_cb_fp)mb_rtu_snd_fsm,
        .tmr_expired = (mb_port_cb_fp)mb_rtu_tmr_35_expired
    };

    (void)slv_addr;

    inst->base.port_obj->cb  = (mb_port_cb_struct *)&mb_rtu_cb;
    inst->base.port_obj->arg = inst;
    inst->is_master          = is_master;

    ENTER_CRITICAL_SECTION();

    /* Modbus RTU uses 8 Databits. */
    if (mb_port_ser_init((mb_port_ser_struct *)inst->base.port_obj, baud, 8, parity) != TRUE)
    {
        status = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if (baud > 19200)
        {
            tmr_35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / (Baudrate / 11)
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            tmr_35_50us = (7UL * 220000UL) / (2UL * baud);
        }
        if (mb_port_ser_tmr_init((mb_port_ser_struct *)inst->base.port_obj,  (USHORT) tmr_35_50us) != TRUE)
        {
            status = MB_EPORTERR;
        }
        inst->snd_state = MB_RTU_TX_STATE_IDLE;
        inst->rcv_state = MB_RTU_RX_STATE_INIT;

        //inst->serial_port.parent = (void*)(inst);
    }
    EXIT_CRITICAL_SECTION();

    return status;
}

void  mb_rtu_start(mb_rtu_tr_struct* inst)
{
    ENTER_CRITICAL_SECTION();
    /* Initially the receiver is in the state MB_RTU_RX_STATE_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to MB_RTU_RX_STATE_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    inst->rcv_state = MB_RTU_RX_STATE_INIT;
    mb_port_ser_enable((mb_port_ser_struct *)inst->base.port_obj, TRUE, FALSE);
    mb_port_ser_tmr_enable((mb_port_ser_struct *)inst->base.port_obj);

    EXIT_CRITICAL_SECTION();
}

void  mb_rtu_stop(mb_rtu_tr_struct* inst)
{
    ENTER_CRITICAL_SECTION();
    mb_port_ser_enable((mb_port_ser_struct *)inst->base.port_obj, FALSE, FALSE);
    mb_port_ser_tmr_disable((mb_port_ser_struct *)inst->base.port_obj);
    EXIT_CRITICAL_SECTION();
}

mb_err_enum  mb_rtu_receive(mb_rtu_tr_struct* inst, UCHAR * rcv_addr_buf, UCHAR ** frame_ptr_buf, USHORT *len_buf)
{
    //BOOL            xFrameReceived = FALSE;
    mb_err_enum    status = MB_ENOERR;

    ENTER_CRITICAL_SECTION();
    assert(inst->rcv_buf_pos < MB_RTU_SER_PDU_SIZE_MAX);

    /* Length and CRC check */
    if ((inst->rcv_buf_pos >= MB_RTU_SER_PDU_SIZE_MIN) && (mb_crc16((UCHAR *)inst->pdu_buf, inst->rcv_buf_pos) == 0))
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *rcv_addr_buf = inst->pdu_buf[MB_RTU_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *len_buf = (USHORT)(inst->rcv_buf_pos - MB_RTU_SER_PDU_PDU_OFF - MB_RTU_SER_PDU_SIZE_CRC);

        /* Return the start of the Modbus PDU to the caller. */
        *frame_ptr_buf = (UCHAR *)&inst->pdu_buf[MB_RTU_SER_PDU_PDU_OFF];

        //xFrameReceived = TRUE;
    }
    else
    {
        status = MB_EIO;
    }

    EXIT_CRITICAL_SECTION();
    return status;
}

mb_err_enum  mb_rtu_send(mb_rtu_tr_struct* inst, UCHAR slv_addr, const UCHAR *frame_ptr, USHORT len)
{
    mb_err_enum    status = MB_ENOERR;
    USHORT          crc16;

    ENTER_CRITICAL_SECTION();

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if (inst->rcv_state == MB_RTU_RX_STATE_IDLE)
    {
#       if MB_MASTER >0
        inst->frame_is_broadcast = (MB_ADDRESS_BROADCAST == slv_addr) ? TRUE : FALSE;
#       endif

        /* First byte before the Modbus-PDU is the slave address. */
        inst->snd_buf_cur = (UCHAR *)frame_ptr - MB_RTU_SER_PDU_PDU_OFF;
        inst->snd_buf_cnt = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        inst->snd_buf_cur[MB_RTU_SER_PDU_ADDR_OFF] = slv_addr;
        inst->snd_buf_cnt += len;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        crc16 = mb_crc16((UCHAR *)inst->snd_buf_cur, inst->snd_buf_cnt);
        inst->snd_buf_cur[inst->snd_buf_cnt++] = (UCHAR)(crc16 & 0xFF);
        inst->snd_buf_cur[inst->snd_buf_cnt++] = (UCHAR)(crc16 >> 8);

        /* Activate the transmitter. */
        inst->snd_state = MB_RTU_TX_STATE_XMIT;
        mb_port_ser_enable((mb_port_ser_struct *)inst->base.port_obj, FALSE, TRUE);
    }
    else
    {
        status = MB_EIO;
    }
    EXIT_CRITICAL_SECTION();
    return status;
}

BOOL  mb_rtu_rcv_fsm(mb_rtu_tr_struct* inst)
{
    BOOL            task_need_switch = FALSE;
    UCHAR           byte_val;

    assert((inst->snd_state == MB_RTU_TX_STATE_IDLE) || (inst->snd_state == MB_RTU_TX_STATE_XFWR));

    /* Always read the character. */
    (void)mb_port_ser_get_byte((mb_port_ser_struct *)inst->base.port_obj, (CHAR *)&byte_val);

    switch (inst->rcv_state)
    {
    /* If we have received a character in the init state we have to
     * wait until the frame is finished.
     */
    case MB_RTU_RX_STATE_INIT:
        mb_port_ser_tmr_enable((mb_port_ser_struct *)inst->base.port_obj);
        break;

    /* In the error state we wait until all characters in the
     * damaged frame are transmitted.
     */
    case MB_RTU_RX_STATE_ERROR:
        mb_port_ser_tmr_enable((mb_port_ser_struct *)inst->base.port_obj);
        break;

    /* In the idle state we wait for a new character. If a character
     * is received the t1.5 and t3.5 timers are started and the
     * receiver is in the state MB_RTU_RX_STATE_RECEIVCE.
     */
    case MB_RTU_RX_STATE_IDLE:
#if MB_MASTER > 0
        if (inst->is_master)
        {
            mb_port_ser_tmr_disable((mb_port_ser_struct *)inst->base.port_obj);
            inst->snd_state = MB_RTU_TX_STATE_IDLE;
        }
#endif
        inst->rcv_buf_pos = 0;
        inst->pdu_buf[inst->rcv_buf_pos++] = byte_val;
        inst->rcv_state = MB_RTU_RX_STATE_RCV;

        /* Enable t3.5 timers. */
        mb_port_ser_tmr_enable((mb_port_ser_struct *)inst->base.port_obj);
        break;

    /* We are currently receiving a frame. Reset the timer after
     * every character received. If more than the maximum possible
     * number of bytes in a modbus frame is received the frame is
     * ignored.
     */
    case MB_RTU_RX_STATE_RCV:
        if (inst->rcv_buf_pos < MB_RTU_SER_PDU_SIZE_MAX)
        {
            inst->pdu_buf[inst->rcv_buf_pos++] = byte_val;
        }
        else
        {
            inst->rcv_state = MB_RTU_RX_STATE_ERROR;
        }
        mb_port_ser_tmr_enable((mb_port_ser_struct *)inst->base.port_obj);
        break;
    }
    return task_need_switch;
}

BOOL  mb_rtu_snd_fsm(mb_rtu_tr_struct* inst)
{
    BOOL            need_poll = FALSE;

    assert(inst->rcv_state == MB_RTU_RX_STATE_IDLE);

    switch (inst->snd_state)
    {
    /* We should not get a transmitter event if the transmitter is in
     * idle state.  */
    case MB_RTU_TX_STATE_IDLE:
        /* enable receiver/disable transmitter. */
        mb_port_ser_enable((mb_port_ser_struct *)inst->base.port_obj,  TRUE, FALSE);
        break;

    case MB_RTU_TX_STATE_XMIT:
        /* check if we are finished. */
        if (inst->snd_buf_cnt != 0)
        {
            mb_port_ser_put_byte((mb_port_ser_struct *)inst->base.port_obj, (CHAR)*inst->snd_buf_cur);
            inst->snd_buf_cur++;  /* next byte in sendbuffer. */
            inst->snd_buf_cnt--;
        }
        else
        {
#if MB_MASTER >0
            if (inst->is_master==TRUE)
            {
                /* Disable transmitter. This prevents another transmit buffer
                 * empty interrupt. */
                mb_port_ser_enable((mb_port_ser_struct *)inst->base.port_obj, TRUE, FALSE);
                inst->snd_state = MB_RTU_TX_STATE_XFWR;
                /* If the frame is broadcast , master will enable timer of convert delay,
                 * else master will enable timer of respond timeout. */
                if (inst->frame_is_broadcast == TRUE)
                {
                    mb_port_ser_tmr_convert_delay_enable((mb_port_ser_struct *)inst->base.port_obj);
                }
                else
                {
                    mb_port_ser_tmr_respond_timeout_enable((mb_port_ser_struct *)inst->base.port_obj);
                }

            }
            else
#endif
            {
                need_poll = mb_port_ser_evt_post((mb_port_ser_struct *)inst->base.port_obj, EV_FRAME_SENT);
                /* Disable transmitter. This prevents another transmit buffer
                 * empty interrupt. */
                mb_port_ser_enable((mb_port_ser_struct *)inst->base.port_obj, TRUE, FALSE);
                inst->snd_state = MB_RTU_TX_STATE_IDLE;
            }
        }
        break;
    default:
        break;
    }

    return need_poll;
}



BOOL  mb_rtu_tmr_35_expired(mb_rtu_tr_struct* inst)
{
    BOOL            need_poll = FALSE;

    switch (inst->rcv_state)
    {
    /* Timer t35 expired. Startup phase is finished. */
    case MB_RTU_RX_STATE_INIT:
        need_poll = mb_port_ser_evt_post((mb_port_ser_struct *)inst->base.port_obj, EV_READY);
        break;

    /* A frame was received and t35 expired. Notify the listener that
     * a new frame was received. */
    case MB_RTU_RX_STATE_RCV:
        need_poll = mb_port_ser_evt_post((mb_port_ser_struct *)inst->base.port_obj, EV_FRAME_RECEIVED);
        break;

    /* An error occured while receiving the frame. */
    case MB_RTU_RX_STATE_ERROR:
        break;

    /* Function called in an illegal state. */
    default:
        assert((inst->rcv_state == MB_RTU_RX_STATE_INIT) ||
                (inst->rcv_state == MB_RTU_RX_STATE_RCV) || (inst->rcv_state == MB_RTU_RX_STATE_ERROR));
    }

    mb_port_ser_tmr_disable((mb_port_ser_struct *)inst->base.port_obj);
    inst->rcv_state = MB_RTU_RX_STATE_IDLE;
#if MB_MASTER >0
    if (inst->is_master == TRUE)
    {
        switch (inst->snd_state)
        {
        /* A frame was send finish and convert delay or respond timeout expired.
         * If the frame is broadcast, The master will idle, and if the frame is not
         * broadcast.Notify the listener process error.*/
        case MB_RTU_TX_STATE_XFWR:
            if (inst->frame_is_broadcast == FALSE)
            {
                need_poll = mb_port_ser_evt_post((mb_port_ser_struct *)inst->base.port_obj, EV_MASTER_ERROR_RESPOND_TIMEOUT);
            }
            break;
        /* Function called in an illegal state. */
        default:
            assert((inst->snd_state == MB_RTU_TX_STATE_XFWR) || (inst->snd_state == MB_RTU_TX_STATE_IDLE));
            break;
        }
        inst->snd_state = MB_RTU_TX_STATE_IDLE;

        mb_port_ser_tmr_disable((mb_port_ser_struct *)inst->base.port_obj);
        /* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
        if (inst->cur_tmr_mode == MB_TMODE_CONVERT_DELAY)
        {
            need_poll = mb_port_ser_evt_post((mb_port_ser_struct *)inst->base.port_obj, EV_EXECUTE);
        }
    }
#endif

    return need_poll;
}

/* Get Modbus Master send PDU's buffer address pointer.*/
void mb_rtu_get_snd_buf(mb_rtu_tr_struct* inst, UCHAR ** frame_ptr_buf)
{
    *frame_ptr_buf = (UCHAR *)&inst->pdu_buf[MB_RTU_SER_PDU_PDU_OFF];
}

#if MB_MASTER > 0
//* Get Modbus send RTU's buffer address pointer.*/
//void vMBMasterGetRTUSndBuf(mb_rtu_tr_struct* inst, UCHAR ** frame_ptr_buf)
//{
//    *frame_ptr_buf = (UCHAR *)inst->snd_buf;
//}


/* Set Modbus Master send PDU's buffer length.*/
void mb_rtu_set_snd_len(mb_rtu_tr_struct* inst, USHORT snd_pdu_len)
{
    inst->snd_pdu_len = snd_pdu_len;
}

/* Get Modbus Master send PDU's buffer length.*/
USHORT mb_rtu_get_snd_len(mb_rtu_tr_struct* inst)
{
    return inst->snd_pdu_len;
}

/* Set Modbus Master current timer mode.*/
void mb_rtu_set_cur_tmr_mode(mb_rtu_tr_struct* inst, mb_tmr_mode_enum tmr_mode)
{
    inst->cur_tmr_mode = tmr_mode;
}

/* The master request is broadcast? */
BOOL mb_rtu_rq_is_bcast(mb_rtu_tr_struct* inst)
{
    return inst->frame_is_broadcast;
}
#endif
