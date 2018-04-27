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
 * File: $Id: mb.c, v 1.28 2010/06/06 13:54:40 wolti Exp $
 */
#include <mb.h>
/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static mb_fn_handler_struct slave_handlers[MB_FUNC_HANDLERS_MAX] =
{
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, (void*)mb_fn_report_slv_id},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, (void*)mb_fn_read_input_reg},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, (void*)mb_fn_read_holding_reg},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, (void*)mb_fn_write_multi_holding_reg},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, (void*)mb_fn_write_holding_reg},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, (void*)mb_fn_rw_multi_holding_reg},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, (void*)mb_fn_read_coils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, (void*)mb_fn_write_coil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, (void*)mb_fn_write_multi_coils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, (void*)mb_fn_read_discrete_inp},
#endif
};

#if MB_MASTER >0
static mb_fn_handler_struct master_handlers[MB_FUNC_HANDLERS_MAX] =
{
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    //TODO Add Master function define
    {MB_FUNC_OTHER_REPORT_SLAVEID, (void*)mb_fn_report_slv_id},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, (void*)mb_mstr_fn_read_inp_reg},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, (void*)mb_mstr_fn_read_holding_reg},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, (void*)mb_mstr_fn_write_multi_holding_reg},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, (void*)mb_mstr_fn_write_holding_reg},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, (void*)mb_mstr_fn_rw_multi_holding_regs},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, (void*)mb_mstr_fn_read_coils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, (void*)mb_mstr_fn_write_coil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, (void*)mb_mstr_fn_write_multi_coils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, (void*)mb_mstr_fn_read_discrete_inputs},
#endif
};
#endif

/* ----------------------- Start implementation -----------------------------*/
#if MB_RTU_ENABLED > 0
mb_err_enum  mb_init_rtu(mb_inst_struct *inst, mb_rtu_tr_struct* transport, UCHAR slv_addr, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity)
{
    return mb_init(inst, (void*)transport, MB_RTU, FALSE, slv_addr, port_obj, baud, parity);
}
#endif

#if MB_ASCII_ENABLED > 0
mb_err_enum  mb_init_ascii(mb_inst_struct *inst, mb_ascii_tr_struct* transport, UCHAR slv_addr, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity)
{

    return mb_init(inst, (void*)transport, MB_ASCII, FALSE, slv_addr, port_obj, baud, parity);
}

#endif


#if MB_MASTER >0

#if MB_RTU_ENABLED > 0
mb_err_enum  mb_mstr_init_rtu(mb_inst_struct *inst, mb_rtu_tr_struct* transport, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity)
{
    return mb_init(inst, (void*)transport, MB_RTU, TRUE, 0, port_obj, baud, parity);
}
#endif

#if MB_ASCII_ENABLED > 0
mb_err_enum  mb_mstr_init_ascii(mb_inst_struct *inst, mb_ascii_tr_struct* transport, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity)
{
    return mb_init(inst, (void*)transport, MB_ASCII, TRUE, 0, port_obj, baud, parity);
}

#endif

#if MB_TCP_ENABLED > 0
mb_err_enum mb_mstr_init_tcp(mb_inst_struct *inst, mb_tcp_tr* transport, USHORT tcp_port_num, SOCKADDR_IN hostaddr)
{
    return mb_init_tcp(inst, transport, tcp_port_num, hostaddr, TRUE);
}

#endif

#endif //MASTER

#if MB_RTU_ENABLED || MB_ASCII_ENABLED
mb_err_enum  mb_init(mb_inst_struct *inst, mb_trans_union *transport, mb_mode_enum mode, BOOL is_master, UCHAR slv_addr, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity)
{
    mb_err_enum    status = MB_ENOERR;

    inst->cur_state          = STATE_NOT_INITIALIZED;
    inst->transport          = (mb_trans_base_struct *)transport;
    inst->port               = port_obj;
    transport->base.port_obj = port_obj;

    switch (mode)
    {
#if MB_RTU_ENABLED > 0
    case MB_RTU:
    {
        inst->trmt = (mb_tr_mtab *)&mb_rtu_mtab;
        inst->pdu_snd_len = &(transport->rtu.snd_pdu_len);
        status = mb_rtu_init((mb_rtu_tr_struct*)transport, is_master, inst->address, baud, parity);
        break;
    }
#endif //RTU
#if MB_ASCII_ENABLED > 0
    case MB_ASCII:
    {
        inst->trmt = (mb_tr_mtab *)&mb_ascii_mtab;
        inst->pdu_snd_len = &(transport->ascii.snd_pdu_len);
        status = mb_ascii_init((mb_ascii_tr_struct*)transport, is_master, inst->address, baud, parity);
        break;
    }
#endif//ASCII
    default:
        return MB_EINVAL;
    }

#if (MB_PORT_HAS_CLOSE > 0)
#   define MB_SERIAL_CLOSE mb_port_ser_close
#else
#   define MB_SERIAL_CLOSE NULL
#endif // MB_PORT_HAS_CLOSE
    static const mb_port_mtab_struct mb_serial_mtab =
    {
        .frm_close = (mb_frm_close_fp) MB_SERIAL_CLOSE,
        .evt_post  = (mp_port_evt_post_fp)mb_port_ser_evt_post,
        .evt_get   = (mb_port_evt_get_fp) mb_port_ser_evt_get
    };
    inst->pmt = (mb_port_mtab_struct *)&mb_serial_mtab;

    inst->trmt->get_tx_frm(inst->transport, (void*)&(inst->frame));

#if MB_MASTER > 0
    if (is_master == TRUE)
    {
        inst->master_is_busy  = FALSE;
        inst->master_mode_run = TRUE;
        inst->func_handlers   = master_handlers;
    }
    else
#endif //MB_MASTER
    {
        if ((slv_addr == MB_ADDRESS_BROADCAST) ||
            (slv_addr < MB_ADDRESS_MIN) ||
            (slv_addr > MB_ADDRESS_MAX))
        {
            return MB_EINVAL;
        }
        inst->func_handlers = slave_handlers;
    }

    inst->address = slv_addr;

    if (status == MB_ENOERR)
    {
        if (!mb_port_ser_evt_init((mb_port_ser_struct*)inst->port))
        {
            /* port dependent event module initalization failed. */
            return MB_EPORTERR;
        }
        inst->cur_mode  = mode;
        inst->cur_state = STATE_DISABLED;
    }

    return status;
}
#endif

#if MB_TCP_ENABLED > 0
mb_err_enum  mb_init_tcp(mb_inst_struct *inst, mb_tcp_tr* transport, USHORT tcp_port_num, SOCKADDR_IN hostaddr, BOOL is_master)
{
    mb_err_enum    status = MB_ENOERR;

    inst->transport   = transport;
    transport->parent = (void*)(inst);
    int i;

    if (transport->is_master ==  TRUE)
    {
        inst->master_mode_run = TRUE;
        inst->func_handlers   = master_handlers;
    }
    else
    {
        inst->func_handlers = slave_handlers;
    }

    if ((status = mb_tcp_init(transport, tcp_port_num, hostaddr, is_master)) != MB_ENOERR)
    {
        inst->cur_state = STATE_DISABLED;
    }
    else if (!mb_port_ser_evt_init(&(transport->tcp_port)))
    {
        /* Port dependent event module initalization failed. */
        status = MB_EPORTERR;
    }
    else
    {
        inst->trmt        = (mb_tr_mtab *)&mb_tcp_mtab;

        inst->address     = MB_TCP_PSEUDO_ADDRESS;
        inst->cur_mode    = MB_TCP;
        inst->cur_state   = STATE_DISABLED;
        inst->pdu_snd_len = &(transport->tcp.snd_pdu_len);

        inst->trmt->get_rx_frm(inst->transport, &inst->frame);
        inst->trmt->get_tx_frm(inst->transport, &inst->frame);

#if (MB_PORT_HAS_CLOSE > 0)
#   define MB_TCP_CLOSE mb_port_ser_close
#else
#   define MB_TCP_CLOSE NULL
#endif // MB_PORT_HAS_CLOSE
        static const mb_port_mtab_struct mb_tcp_mtab =
        {
            .frm_close = (mb_frm_close_fp) MB_TCP_CLOSE,
            .evt_post  = (mp_port_evt_post_fp)mb_port_ser_evt_post,
            .evt_get   = (mb_port_evt_get_fp) mb_port_ser_evt_get
        };
        inst->pmt = (mb_port_mtab_struct *)&mb_tcp_mtab;
    }
    return status;
}
#endif

mb_err_enum
mb_close(mb_inst_struct *inst)
{
    if (inst->cur_state == STATE_DISABLED)
    {
        if (inst->pmt->frm_close != NULL)
        {
            inst->pmt->frm_close((mb_port_base_struct *)inst->transport);
        }
        return MB_ENOERR;
    }
    else
    {
        return MB_EILLSTATE;
    }
}

mb_err_enum  mb_enable(mb_inst_struct *inst)
{
    if (inst->cur_state == STATE_DISABLED)
    {
        /* Activate the protocol stack. */
        inst->trmt->frm_start(inst->transport);
        inst->cur_state = STATE_ENABLED;
        return MB_ENOERR;
    }
    else
    {
        return MB_EILLSTATE;
    }
}

mb_err_enum  mb_disable(mb_inst_struct *inst)
{
    mb_err_enum    status;

    if (inst->cur_state == STATE_ENABLED)
    {
        inst->trmt->frm_stop(inst->transport);
        inst->cur_state = STATE_DISABLED;
        status          = MB_ENOERR;
    }
    else if (inst->cur_state == STATE_DISABLED)
    {
        status = MB_ENOERR;
    }
    else
    {
        status = MB_EILLSTATE;
    }
    return status;
}

mb_err_enum mb_poll(mb_inst_struct *inst)
{
    int                      i;
    //int                      j;
    mb_err_enum              status = MB_ENOERR;
    mb_event_enum            event;
    BOOL                     got_event;

    /* Check if the protocol stack is ready. */
    if (inst->cur_state != STATE_ENABLED)
    {
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    got_event = inst->pmt->evt_get(inst->port, inst, &event);
    if (got_event  == TRUE)
    {
        switch (event)
        {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            status = inst->trmt->frm_rcv(inst->transport, &(inst->rcv_addr), (UCHAR**) &inst->frame, (USHORT*)&inst->len);
            if (status == MB_ENOERR)
            {
#if MB_MASTER > 0
                if (inst->master_mode_run == TRUE)
                {
                    if (inst->rcv_addr == inst->master_dst_addr || inst->cur_mode == MB_TCP) //All addresses work in tcp mode
                    {
                        (void)inst->pmt->evt_post(inst->port, EV_EXECUTE);
                    }
                }
                else
#endif // MB_MASTER
                {
                    /* Check if the frame is for us. If not ignore the frame. */
                    if ((inst->rcv_addr == inst->address) || (inst->rcv_addr == MB_ADDRESS_BROADCAST))
                    {
                        (void)inst->pmt->evt_post(inst->port, EV_EXECUTE);
                    }
                }

            }
            else
            {
#if MB_MASTER > 0
                if (inst->master_mode_run)
                {
                    (void)inst->pmt->evt_post(inst->port, EV_MASTER_ERROR_RECEIVE_DATA);
                }
#endif // MB_MASTER
            }
            break;

        case EV_EXECUTE:
        {
            inst->func_code = inst->frame[MB_PDU_FUNC_OFF];
            inst->exception = MB_EX_ILLEGAL_FUNCTION;

#if MB_MASTER > 0
            if ((inst->func_code >> 7) && inst->master_mode_run)
            {
                inst->exception = (mb_exception_enum)inst->frame[MB_PDU_DATA_OFF];
            }
            else
#endif // MB_MASTER
            {
                for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++)
                {
                    /* No more function handlers registered. Abort. */
                    if (inst->func_handlers[i].func_code == 0)
                    {
                        break;
                    }
                    else if (inst->func_handlers[i].func_code == inst->func_code)
                    {
                        inst->exception = inst->func_handlers[i].handler(inst, (UCHAR*)(inst->frame), (USHORT*)&inst->len);
                        break;
                    }
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */
#if MB_MASTER > 0
            if (inst->master_mode_run)
            {
                if (inst->exception != MB_EX_NONE)
                {
                    (void)inst->pmt->evt_post(inst->port, EV_MASTER_ERROR_EXECUTE_FUNCTION);
                }
                else
                {
                    inst->master_is_busy = FALSE;
                    mb_mstr_rq_success_cb(inst);
                }
            }
            else
#endif
            {
                if (inst->rcv_addr != MB_ADDRESS_BROADCAST)
                {
                    if (inst->exception != MB_EX_NONE)
                    {
                        /* An exception occured. Build an error frame. */
                        inst->len = 0;
                        inst->frame[inst->len++] = (UCHAR)(inst->func_code | MB_FUNC_ERROR);
                        inst->frame[inst->len++] = inst->exception;
                    }
                    if ((inst->cur_mode == MB_ASCII) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS)
                    {
                        mb_port_ser_tmr_delay((mb_port_ser_struct *)inst->port, MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS);///WTF?????????
                    }
                    status = inst->trmt->frm_send(inst->transport, inst->address, (UCHAR*)(inst->frame), inst->len);
                    if (MB_ENOERR != status)
                    {
                        return MB_EIO;
                    }
                }
            }
        }
        break;

        case EV_FRAME_SENT:
#if MB_MASTER > 0
            if (inst->master_mode_run)
            {
                if (FALSE == inst->master_is_busy)
                {
                    inst->trmt->get_tx_frm(inst->transport, (UCHAR**)(&inst->frame));
                    status = inst->trmt->frm_send(inst->transport, inst->master_dst_addr, (UCHAR*)(inst->frame), *inst->pdu_snd_len);
                    if (MB_ENOERR == status)
                    {
                        inst->master_is_busy = TRUE; /* Master is busy now. */
                    }
                    else
                    {
                        return MB_EIO;
                    }
                }
            }
#endif
            break;

//        case EV_ERROR_PROCESS:
#if MB_MASTER > 0
        case EV_MASTER_ERROR_RESPOND_TIMEOUT:
        {
            inst->master_is_busy = FALSE;
            mb_mstr_error_timeout_cb(inst);
            //status = MB_ETIMEDOUT;
        }
        break;
        case EV_MASTER_ERROR_RECEIVE_DATA:
        {
            inst->master_is_busy = FALSE;
            mb_mstr_error_rcv_data_cb(inst);
            //status = MB_EIO;
        }
        break;
        case EV_MASTER_ERROR_EXECUTE_FUNCTION:
        {
            inst->master_is_busy = FALSE;
            mb_mstr_error_exec_fn_cb(inst);
            //status = MB_EILLFUNC;
        }
        break;
#endif
        }
    }
    return MB_ENOERR;//status;
}

