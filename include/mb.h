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
    peMBFrameSend    frm_send;
    pvMBFrameStart   frm_start;
    pvMBFrameStop    frm_stop;
    peMBFrameReceive frm_rcv;

    pvGetRxFrame     get_rx_frm;
    pvGetRxFrame     get_tx_frm;
#if MB_MASTER > 0
    pbMBMasterRequestIsBroadcast rq_is_broadcast;
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
    mb_trans_base base;
#if MB_RTU_ENABLED == 1
    mb_rtu_tr   rtu;
#endif

#if MB_ASCII_ENABLED == 1
    mb_ascii_tr ascii;
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
} eMBState ;

typedef struct
{
    mb_trans_base *transport;
    mb_port_base  *port;

    UCHAR    address;
    mb_mode_enum  cur_mode;
    eMBState cur_state;

    volatile UCHAR *rx_frame;
    volatile UCHAR *tx_frame;

    volatile USHORT   len;

    volatile USHORT * pdu_snd_len;


    //Transport methods
    mb_tr_mtab * trmt;

    //Port methods
    mb_port_mtab * pmt; //!< Port method tab

    //Place to const
    xMBFunctionHandler * xFuncHandlers;//[MB_FUNC_HANDLERS_MAX];

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
 * A typical application will want to call eMBInit() first. If the device
 * is ready to answer network requests it must then call eMBEnable() to activate
 * the protocol stack. In the main loop the function eMBPoll() must be called
 * periodically. The time interval between pooling depends on the configured
 * Modbus timeout. If an RTOS is available a separate task should be created
 * and the task should always call the function eMBPoll().
 *
 * \code
 * // Initialize protocol stack in RTU mode for a slave with address 10 = 0x0A
 * eMBInit(MB_RTU, 0x0A, 38400, MB_PAR_EVEN);
 * // Enable the Modbus Protocol Stack.
 * eMBEnable();
 * for (;;)
 * {
 *     // Call the main polling loop of the Modbus protocol stack.
 *     eMBPoll();
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
 * processed until eMBEnable() has been called.
 *
 * \param eMode If ASCII or RTU mode should be used.
 * \param ucSlaveAddress The slave address. Only frames sent to this
 *   address or to the broadcast address are processed.
 * \param ucPort The port to use. E.g. 1 for COM1 on windows. This value
 *   is platform dependent and some ports simply choose to ignore it.
 * \param ulBaudRate The baudrate. E.g. 19200. Supported baudrates depend
 *   on the porting layer.
 * \param eParity Parity used for serial transmission.
 *
 * \return If no error occurs the function returns mb_err_enum::MB_ENOERR.
 *   The protocol is then in the disabled state and ready for activation
 *   by calling eMBEnable(). Otherwise one of the following error codes
 *   is returned:
 *    - mb_err_enum::MB_EINVAL If the slave address was not valid. Valid
 *        slave addresses are in the range 1 - 247.
 *    - mb_err_enum::MB_EPORTERR IF the porting layer returned an error.
 */
mb_err_enum eMBInit(mb_instance* inst, mb_trans_union *transport, mb_mode_enum eMode, BOOL is_master, UCHAR ucSlaveAddress, mb_port_base * port_obj, ULONG ulBaudRate, eMBParity eParity);
#if MB_RTU_ENABLED
mb_err_enum
eMBInitRTU(mb_instance* inst, mb_rtu_tr* transport, UCHAR ucSlaveAddress, mb_port_base * port_obj, ULONG ulBaudRate, eMBParity eParity);
#endif

#if MB_ASCII_ENABLED
mb_err_enum eMBInitASCII(mb_instance* inst, mb_ascii_tr* transport, UCHAR ucSlaveAddress, mb_port_base * port_obj, ULONG ulBaudRate, eMBParity eParity);
#endif

#if MB_TCP_ENABLED > 0
mb_err_enum eMBTCPInit(mb_instance* inst, mb_tcp_tr* transport, USHORT ucTCPPort, SOCKADDR_IN hostaddr, BOOL bMaster);
#endif

#if MB_MASTER >0

#if MB_RTU_ENABLED > 0

mb_err_enum eMBMasterInitRTU(mb_instance* inst, mb_rtu_tr* transport, mb_port_base * port_obj, ULONG ulBaudRate, eMBParity eParity);
#endif
#if MB_ASCII_ENABLED > 0
mb_err_enum eMBMasterInitASCII(mb_instance* inst, mb_ascii_tr* transport, mb_port_base * port_obj, ULONG ulBaudRate, eMBParity eParity);
#endif
#if MB_TCP_ENABLED >0
mb_err_enum eMBMasterInitTCP(mb_instance* inst, mb_tcp_tr* transport, USHORT ucTCPPort, SOCKADDR_IN hostaddr);
#endif
#endif
/*! \ingroup modbus
 * \brief Initialize the Modbus protocol stack for Modbus TCP.
 *
 * This function initializes the Modbus TCP Module. Please note that
 * frame processing is still disabled until eMBEnable() is called.
 *
 * \param usTCPPort The TCP port to listen on.
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
mb_err_enum eMBClose(mb_instance* inst);

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
mb_err_enum eMBEnable(mb_instance* inst);

/*! \ingroup modbus
 * \brief Disable the Modbus protocol stack.
 *
 * This function disables processing of Modbus frames.
 *
 * \return If the protocol stack has been disabled it returns
 *  mb_err_enum::MB_ENOERR. If it was not in the enabled state it returns
 *  mb_err_enum::MB_EILLSTATE.
 */
mb_err_enum eMBDisable(mb_instance* inst);

/*! \ingroup modbus
 * \brief The main pooling loop of the Modbus protocol stack.
 *
 * This function must be called periodically. The timer interval required
 * is given by the application dependent Modbus slave timeout. Internally the
 * function calls xMBPortEventGet() and waits for an event from the receiver or
 * transmitter state machines.
 *
 * \return If the protocol stack is not in the enabled state the function
 *   returns mb_err_enum::MB_EILLSTATE. Otherwise it returns
 *   mb_err_enum::MB_ENOERR.
 */
mb_err_enum eMBPoll(mb_instance* inst);

/*! \ingroup modbus
 * \brief Configure the slave id of the device.
 *
 * This function should be called when the Modbus function <em>Report Slave ID</em>
 * is enabled (By defining MB_FUNC_OTHER_REP_SLAVEID_ENABLED in mbconfig.h).
 *
 * \param ucSlaveID Values is returned in the <em>Slave ID</em> byte of the
 *   <em>Report Slave ID</em> response.
 * \param xIsRunning If TRUE the <em>Run Indicator Status</em> byte is set to 0xFF.
 *   otherwise the <em>Run Indicator Status</em> is 0x00.
 * \param pucAdditional Values which should be returned in the <em>Additional</em>
 *   bytes of the <em> Report Slave ID</em> response.
 * \param usAdditionalLen Length of the buffer <code>pucAdditonal</code>.
 *
 * \return If the static buffer defined by MB_FUNC_OTHER_REP_SLAVEID_BUF in
 *   mbconfig.h is to small it returns mb_err_enum::MB_ENORES. Otherwise
 *   it returns mb_err_enum::MB_ENOERR.
 */
mb_err_enum eMBSetSlaveID(mb_instance* inst, UCHAR ucSlaveID, BOOL xIsRunning, UCHAR const *pucAdditional, USHORT usAdditionalLen);

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
 *   by \c usAddress and the last register is given by <tt>usAddress +
 *   usNRegs - 1</tt>.
 *
 * \param pucRegBuffer A buffer where the callback function should write
 *   the current value of the modbus registers to.
 * \param usAddress The starting address of the register. Input registers
 *   are in the range 1 - 65535.
 * \param usNRegs Number of registers the callback function must supply.
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
mb_err_enum eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Holding Register</em> value is
 *   read or written by the protocol stack. The starting register address
 *   is given by \c usAddress and the last register is given by
 *   <tt>usAddress + usNRegs - 1</tt>.
 *
 * \param pucRegBuffer If the application registers values should be updated the
 *   buffer points to the new registers values. If the protocol stack needs
 *   to now the current values the callback function should write them into
 *   this buffer.
 * \param usAddress The starting address of the register.
 * \param usNRegs Number of registers to read or write.
 * \param eMode If mb_reg_mode_enum::MB_REG_WRITE the application register
 *   values should be updated from the values in the buffer. For example
 *   this would be the case when the Modbus master has issued an
 *   <b>WRITE SINGLE REGISTER</b> command.
 *   If the value mb_reg_mode_enum::MB_REG_READ the application should copy
 *   the current values into the buffer \c pucRegBuffer.
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
mb_err_enum eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, mb_reg_mode_enum eMode);

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Coil Register</em> value is
 *   read or written by the protocol stack. If you are going to use
 *   this function you might use the functions xMBUtilSetBits() and
 *   xMBUtilGetBits() for working with bitfields.
 *
 * \param pucRegBuffer The bits are packed in bytes where the first coil
 *   starting at address \c usAddress is stored in the LSB of the
 *   first byte in the buffer <code>pucRegBuffer</code>.
 *   If the buffer should be written by the callback function unused
 *   coil values (I.e. if not a multiple of eight coils is used) should be set
 *   to zero.
 * \param usAddress The first coil number.
 * \param usNCoils Number of coil values requested.
 * \param eMode If mb_reg_mode_enum::MB_REG_WRITE the application values should
 *   be updated from the values supplied in the buffer \c pucRegBuffer.
 *   If mb_reg_mode_enum::MB_REG_READ the application should store the current
 *   values in the buffer \c pucRegBuffer.
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
mb_err_enum eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, mb_reg_mode_enum eMode);

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Input Discrete Register</em> value is
 *   read by the protocol stack.
 *
 * If you are going to use his function you might use the functions
 * xMBUtilSetBits() and xMBUtilGetBits() for working with bitfields.
 *
 * \param pucRegBuffer The buffer should be updated with the current
 *   coil values. The first discrete input starting at \c usAddress must be
 *   stored at the LSB of the first byte in the buffer. If the requested number
 *   is not a multiple of eight the remaining bits should be set to zero.
 * \param usAddress The starting address of the first discrete input.
 * \param usNDiscrete Number of discrete input values.
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
mb_err_enum eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete);

PR_END_EXTERN_C
#endif
