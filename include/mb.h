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
 * File: $Id: mb.h, v 1.17 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_H
#define _MB_H

#include <mb_common.h>
PR_BEGIN_EXTERN_C
/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
/* ----------------------- Modbus includes ----------------------------------*/
#include <mbconfig.h>
#include <mb_types.h>
#include <mbport.h>

#if (MB_RTU_ENABLED>0) || (MB_ASCII_ENABLED>0)
#include <serial_port.h>
#endif

#if MB_TCP_ENABLED > 0
#   include <tcp_port.h>
#endif

#include <mbframe.h>
#include <mbproto.h>
#include <mbutils.h>

typedef struct
{
    mb_frm_snd_fp    frm_send;
    mb_frm_start_fp   frm_start;
    mb_frm_stop_fp    frm_stop;
    mb_frm_rcv_fp frm_rcv;

    mb_get_rx_frm_fp     get_rx_frm;
    mb_get_rx_frm_fp     get_tx_frm;
#if MB_MASTER > 0
    mb_mstr_rq_is_bcast_fp rq_is_broadcast;
#endif //MB_MASTER
}
mb_tr_mtab;//!< Transport method tab

#if MB_RTU_ENABLED == 1
#   include <mbrtu.h>
#endif

#if MB_ASCII_ENABLED == 1
#   include <mbascii.h>
#endif

#if MB_TCP_ENABLED == 1
#   include <mbtcp.h>
#endif

typedef union
{
    mb_trans_base_struct base;
#if MB_RTU_ENABLED == 1
    mb_rtu_tr_struct   rtu;
#endif

#if MB_ASCII_ENABLED == 1
    mb_ascii_tr_struct ascii;
#endif

#if MB_TCP_ENABLED == 1
    mb_tcp_tr   tcp;
#endif
}mb_trans_union;

typedef enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} mb_state_enum;

typedef struct
{
    mb_trans_base_struct *transport;
    mb_port_base_struct  *port;

    UCHAR         address;
    mb_mode_enum  cur_mode;
    mb_state_enum cur_state;

    volatile UCHAR *rx_frame;
    volatile UCHAR *tx_frame;

    volatile USHORT   len;

    volatile USHORT * pdu_snd_len;


    //Transport methods
    mb_tr_mtab * trmt;

    //Port methods
    mb_port_mtab_struct * pmt; //!< Port method tab

    //Place to const
    mb_fn_handler_struct * func_handlers;//[MB_FUNC_HANDLERS_MAX];

    //for slave id
    UCHAR    slave_id[MB_FUNC_OTHER_REP_SLAVEID_BUF];
    USHORT   slave_id_len;

#if MB_MASTER > 0
    //master variables
    BOOL master_mode_run;
    BOOL master_is_busy;
    UCHAR master_dst_addr;
    //eMBMasterErrorEventType  master_err_cur;
#endif //MB_MASTER
} mb_instance;

#include <mbfunc.h>

#if MB_MASTER > 0
#include "mb_master.h"
#endif

/*! \defgroup modbus Modbus
 * \code #include "mb.h" \endcode
 *
 * This module defines the interface for the application. It contains
 * the basic functions and types required to use the Modbus protocol stack.
 * A typical application will want to call mb_init() first. If the device
 * is ready to answer network requests it must then call mb_enable() to activate
 * the protocol stack. In the main loop the function mb_poll() must be called
 * periodically. The time interval between pooling depends on the configured
 * Modbus timeout. If an RTOS is available a separate task should be created
 * and the task should always call the function mb_poll().
 *
 * \code
 * // Initialize protocol stack in RTU mode for a slave with address 10 = 0x0A
 * mb_init(MB_RTU, 0x0A, 38400, MB_PAR_EVEN);
 * // Enable the Modbus Protocol Stack.
 * mb_enable();
 * for (;;)
 * {
 *     // Call the main polling loop of the Modbus protocol stack.
 *     mb_poll();
 *     ...
 * }
 * \endcode
 */

/* ----------------------- Defines ------------------------------------------*/

/*! \ingroup modbus
 * \brief Use the default Modbus TCP port (502)
 */
#define MB_TCP_PORT_USE_DEFAULT 0


/* ----------------------- Function prototypes ------------------------------*/
/*! \ingroup modbus
 * \brief Initialize the Modbus protocol stack.
 *
 * This functions initializes the ASCII or RTU module and calls the
 * init functions of the porting layer to prepare the hardware. Please
 * note that the receiver is still disabled and no Modbus frames are
 * processed until mb_enable() has been called.
 *
 * \param mode If ASCII or RTU mode should be used.
 * \param slv_addr The slave address. Only frames sent to this
 *   address or to the broadcast address are processed.
 * \param ucPort The port to use. E.g. 1 for COM1 on windows. This value
 *   is platform dependent and some ports simply choose to ignore it.
 * \param baud The baudrate. E.g. 19200. Supported baudrates depend
 *   on the porting layer.
 * \param parity Parity used for serial transmission.
 *
 * \return If no error occurs the function returns mb_err_enum::MB_ENOERR.
 *   The protocol is then in the disabled state and ready for activation
 *   by calling mb_enable(). Otherwise one of the following error codes
 *   is returned:
 *    - mb_err_enum::MB_EINVAL If the slave address was not valid. Valid
 *        slave addresses are in the range 1 - 247.
 *    - mb_err_enum::MB_EPORTERR IF the porting layer returned an error.
 */
mb_err_enum mb_init(mb_instance* inst, mb_trans_union *transport, mb_mode_enum mode, BOOL is_master, UCHAR slv_addr, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity);
#if MB_RTU_ENABLED
mb_err_enum mb_init_rtu(mb_instance* inst, mb_rtu_tr_struct* transport, UCHAR slv_addr, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity);
#endif

#if MB_ASCII_ENABLED
mb_err_enum mb_init_ascii(mb_instance* inst, mb_ascii_tr_struct* transport, UCHAR slv_addr, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity);
#endif

#if MB_TCP_ENABLED > 0
mb_err_enum mb_init_tcp(mb_instance* inst, mb_tcp_tr* transport, USHORT tcp_port_num, SOCKADDR_IN hostaddr, BOOL is_master);
#endif

#if MB_MASTER >0
#   if MB_RTU_ENABLED > 0
mb_err_enum mb_mstr_init_rtu(mb_instance* inst, mb_rtu_tr_struct* transport, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity);
#   endif
#   if MB_ASCII_ENABLED > 0
mb_err_enum mb_mstr_init_ascii(mb_instance* inst, mb_ascii_tr_struct* transport, mb_port_base_struct * port_obj, ULONG baud, mb_port_ser_parity_enum parity);
#   endif
#   if MB_TCP_ENABLED >0
mb_err_enum mb_mstr_init_tcp(mb_instance* inst, mb_tcp_tr* transport, USHORT tcp_port_num, SOCKADDR_IN hostaddr);
#   endif
#endif
/*! \ingroup modbus
 * \brief Initialize the Modbus protocol stack for Modbus TCP.
 *
 * This function initializes the Modbus TCP Module. Please note that
 * frame processing is still disabled until mb_enable() is called.
 *
 * \param tcp_port_num The TCP port to listen on.
 * \return If the protocol stack has been initialized correctly the function
 *   returns mb_err_enum::MB_ENOERR. Otherwise one of the following error
 *   codes is returned:
 *    - mb_err_enum::MB_EINVAL If the slave address was not valid. Valid
 *        slave addresses are in the range 1 - 247.
 *    - mb_err_enum::MB_EPORTERR IF the porting layer returned an error.
 */

/*! \ingroup modbus
 * \brief Release resources used by the protocol stack.
 *
 * This function disables the Modbus protocol stack and release all
 * hardware resources. It must only be called when the protocol stack
 * is disabled.
 *
 * \note Note all ports implement this function. A port which wants to
 *   get an callback must define the macro MB_PORT_HAS_CLOSE to 1.
 *
 * \return If the resources where released it return mb_err_enum::MB_ENOERR.
 *   If the protocol stack is not in the disabled state it returns
 *   mb_err_enum::MB_EILLSTATE.
 */
mb_err_enum mb_close(mb_instance* inst);

/*! \ingroup modbus
 * \brief Enable the Modbus protocol stack.
 *
 * This function enables processing of Modbus frames. Enabling the protocol
 * stack is only possible if it is in the disabled state.
 *
 * \return If the protocol stack is now in the state enabled it returns
 *   mb_err_enum::MB_ENOERR. If it was not in the disabled state it
 *   return mb_err_enum::MB_EILLSTATE.
 */
mb_err_enum mb_enable(mb_instance* inst);

/*! \ingroup modbus
 * \brief Disable the Modbus protocol stack.
 *
 * This function disables processing of Modbus frames.
 *
 * \return If the protocol stack has been disabled it returns
 *  mb_err_enum::MB_ENOERR. If it was not in the enabled state it returns
 *  mb_err_enum::MB_EILLSTATE.
 */
mb_err_enum mb_disable(mb_instance* inst);

/*! \ingroup modbus
 * \brief The main pooling loop of the Modbus protocol stack.
 *
 * This function must be called periodically. The timer interval required
 * is given by the application dependent Modbus slave timeout. Internally the
 * function calls mb_port_ser_evt_get() and waits for an event from the receiver or
 * transmitter state machines.
 *
 * \return If the protocol stack is not in the enabled state the function
 *   returns mb_err_enum::MB_EILLSTATE. Otherwise it returns
 *   mb_err_enum::MB_ENOERR.
 */
mb_err_enum mb_poll(mb_instance* inst);

/*! \ingroup modbus
 * \brief Configure the slave id of the device.
 *
 * This function should be called when the Modbus function <em>Report Slave ID</em>
 * is enabled (By defining MB_FUNC_OTHER_REP_SLAVEID_ENABLED in mbconfig.h).
 *
 * \param slv_id Values is returned in the <em>Slave ID</em> byte of the
 *   <em>Report Slave ID</em> response.
 * \param is_running If TRUE the <em>Run Indicator Status</em> byte is set to 0xFF.
 *   otherwise the <em>Run Indicator Status</em> is 0x00.
 * \param slv_idstr Values which should be returned in the <em>Additional</em>
 *   bytes of the <em> Report Slave ID</em> response.
 * \param slv_idstr_len Length of the buffer <code>pucAdditonal</code>.
 *
 * \return If the static buffer defined by MB_FUNC_OTHER_REP_SLAVEID_BUF in
 *   mbconfig.h is to small it returns mb_err_enum::MB_ENORES. Otherwise
 *   it returns mb_err_enum::MB_ENOERR.
 */
mb_err_enum mb_set_slv_id(mb_instance* inst, UCHAR slv_id, BOOL is_running, UCHAR const *slv_idstr, USHORT slv_idstr_len);

/* ----------------------- Callback -----------------------------------------*/

/*! \defgroup modbus_registers Modbus Registers
 * \code #include "mb.h" \endcode
 * The protocol stack does not internally allocate any memory for the
 * registers. This makes the protocol stack very small and also usable on
 * low end targets. In addition the values don't have to be in the memory
 * and could for example be stored in a flash.<br>
 * Whenever the protocol stack requires a value it calls one of the callback
 * function with the register address and the number of registers to read
 * as an argument. The application should then read the actual register values
 * (for example the ADC voltage) and should store the result in the supplied
 * buffer.<br>
 * If the protocol stack wants to update a register value because a write
 * register function was received a buffer with the new register values is
 * passed to the callback function. The function should then use these values
 * to update the application register values.
 */

/*! \ingroup modbus_registers
 * \brief Callback function used if the value of a <em>Input Register</em>
 *   is required by the protocol stack. The starting register address is given
 *   by \c reg_addr and the last register is given by <tt>reg_addr +
 *   reg_num - 1</tt>.
 *
 * \param reg_buff A buffer where the callback function should write
 *   the current value of the modbus registers to.
 * \param reg_addr The starting address of the register. Input registers
 *   are in the range 1 - 65535.
 * \param reg_num Number of registers the callback function must supply.
 *
 * \return The function must return one of the following error codes:
 *   - mb_err_enum::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - mb_err_enum::MB_ENOREG If the application can not supply values
 *       for registers within this range. In this case a
 *       <b>ILLEGAL DATA ADDRESS</b> exception frame is sent as a response.
 *   - mb_err_enum::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - mb_err_enum::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
mb_err_enum mb_reg_input_cb(UCHAR * reg_buff, USHORT reg_addr, USHORT reg_num);

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Holding Register</em> value is
 *   read or written by the protocol stack. The starting register address
 *   is given by \c reg_addr and the last register is given by
 *   <tt>reg_addr + reg_num - 1</tt>.
 *
 * \param reg_buff If the application registers values should be updated the
 *   buffer points to the new registers values. If the protocol stack needs
 *   to now the current values the callback function should write them into
 *   this buffer.
 * \param reg_addr The starting address of the register.
 * \param reg_num Number of registers to read or write.
 * \param mode If mb_reg_mode_enum::MB_REG_WRITE the application register
 *   values should be updated from the values in the buffer. For example
 *   this would be the case when the Modbus master has issued an
 *   <b>WRITE SINGLE REGISTER</b> command.
 *   If the value mb_reg_mode_enum::MB_REG_READ the application should copy
 *   the current values into the buffer \c reg_buff.
 *
 * \return The function must return one of the following error codes:
 *   - mb_err_enum::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - mb_err_enum::MB_ENOREG If the application can not supply values
 *       for registers within this range. In this case a
 *       <b>ILLEGAL DATA ADDRESS</b> exception frame is sent as a response.
 *   - mb_err_enum::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - mb_err_enum::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
mb_err_enum mb_reg_holding_cb(UCHAR * reg_buff, USHORT reg_addr, USHORT reg_num, mb_reg_mode_enum mode);

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Coil Register</em> value is
 *   read or written by the protocol stack. If you are going to use
 *   this function you might use the functions xMBUtilSetBits() and
 *   mb_util_get_bits() for working with bitfields.
 *
 * \param reg_buff The bits are packed in bytes where the first coil
 *   starting at address \c reg_addr is stored in the LSB of the
 *   first byte in the buffer <code>reg_buff</code>.
 *   If the buffer should be written by the callback function unused
 *   coil values (I.e. if not a multiple of eight coils is used) should be set
 *   to zero.
 * \param reg_addr The first coil number.
 * \param coil_num Number of coil values requested.
 * \param mode If mb_reg_mode_enum::MB_REG_WRITE the application values should
 *   be updated from the values supplied in the buffer \c reg_buff.
 *   If mb_reg_mode_enum::MB_REG_READ the application should store the current
 *   values in the buffer \c reg_buff.
 *
 * \return The function must return one of the following error codes:
 *   - mb_err_enum::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - mb_err_enum::MB_ENOREG If the application does not map an coils
 *       within the requested address range. In this case a
 *       <b>ILLEGAL DATA ADDRESS</b> is sent as a response.
 *   - mb_err_enum::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - mb_err_enum::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
mb_err_enum mb_reg_coils_cb(UCHAR * reg_buff, USHORT reg_addr, USHORT coil_num, mb_reg_mode_enum mode);

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Input Discrete Register</em> value is
 *   read by the protocol stack.
 *
 * If you are going to use his function you might use the functions
 * xMBUtilSetBits() and mb_util_get_bits() for working with bitfields.
 *
 * \param reg_buff The buffer should be updated with the current
 *   coil values. The first discrete input starting at \c reg_addr must be
 *   stored at the LSB of the first byte in the buffer. If the requested number
 *   is not a multiple of eight the remaining bits should be set to zero.
 * \param reg_addr The starting address of the first discrete input.
 * \param disc_num Number of discrete input values.
 * \return The function must return one of the following error codes:
 *   - mb_err_enum::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - mb_err_enum::MB_ENOREG If no such discrete inputs exists.
 *       In this case a <b>ILLEGAL DATA ADDRESS</b> exception frame is sent
 *       as a response.
 *   - mb_err_enum::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - mb_err_enum::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
mb_err_enum mb_reg_discrete_cb(UCHAR * reg_buff, USHORT reg_addr, USHORT disc_num);

PR_END_EXTERN_C
#endif
