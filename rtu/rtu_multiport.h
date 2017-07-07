/*
 * rtu_multiport.h
 *
 *  Created on: 20 июля 2016 г.
 *      Author: Radeon
 */

#ifndef MODBUS_RTU_RTU_MULTIPORT_H_
#define MODBUS_RTU_RTU_MULTIPORT_H_


#include "mb_types.h"
#include "serial_multi.h"

#if MB_MULTIPORT>0 && (defined(MB_RTU_ENABLED))

#   define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#   define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#   define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#   define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#   define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */
	/* ----------------------- Type definitions ---------------------------------*/
	typedef enum
	{
		RTU_STATE_RX_INIT,              /*!< Receiver is in initial state. */
		RTU_STATE_RX_IDLE,              /*!< Receiver is in idle state. */
		RTU_STATE_RX_RCV,               /*!< Frame is beeing received. */
		RTU_STATE_RX_ERROR              /*!< If the frame is invalid. */
	}RTU_eMBRcvState;

	typedef enum
	{
		RTU_STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
		RTU_STATE_TX_XMIT,              /*!< Transmitter is in transfer state. */
		RTU_STATE_TX_XFWR
	} RTU_eMBSndState;

	typedef struct
	{
	    void* parent;
		MBSerialInstance serial_port;
		volatile RTU_eMBSndState eSndState;
		volatile RTU_eMBRcvState eRcvState;

		volatile UCHAR  ucRTUSndBuf[MB_PDU_SIZE_MAX];
		volatile UCHAR  ucRTURcvBuf[MB_SER_PDU_SIZE_MAX];
		volatile USHORT usSendPDULength;

		volatile UCHAR *pucSndBufferCur;
		volatile USHORT usSndBufferCount;

		volatile USHORT usRcvBufferPos;

		BOOL rtuMaster;

		BOOL xFrameIsBroadcast;
		volatile eMBMasterTimerMode eCurTimerMode;

	} MBRTUInstance;

#define RTU_ARG MBRTUInstance* inst,
#define RTU_ARG_VOID MBRTUInstance* inst

#else

#define RTU_ARG
#define RTU_ARG_VOID void
#define SERIAL_ARG
#define SERIAL_ARG_VOID

#endif  /*RTU_MULTIPORT*/



#endif /* MODBUS_RTU_RTU_MULTIPORT_H_ */
