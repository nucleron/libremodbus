/*
 * tcp_multiport.h
 *
 *  Created on: 28 июля 2016 г.
 *      Author: Radeon
 */

#ifndef MODBUS_TCP_TCP_MULTIPORT_H_
#define MODBUS_TCP_TCP_MULTIPORT_H_

#include "mb_types.h"
#include "tcp_multi.h"

#if defined(MB_MULTIPORT) && (defined(MB_TCP_ENABLED))

	/* ----------------------- Type definitions ---------------------------------*/
	typedef struct
	{
		MBTCPPortInstance tcp_port;
		BOOL tcpMaster;
		USHORT usSendPDULength;
		BOOL xFrameIsBroadcast;
	} MBTCPInstance;

	#define TCP_ARG MBTCPInstance* inst,
	#define TCP_ARG_VOID MBTCPInstance* inst

	#define TCPPORT_ARG &(inst->tcp_port),
	#define TCPPORT_ARG_VOID &(inst->tcp_port)

#else

	#define TCP_ARG
	#define TCP_ARG_VOID void
	//#define SERIAL_ARG

#endif  /*MB_MULTIPORT*/

#endif /* MODBUS_TCP_TCP_MULTIPORT_H_ */
