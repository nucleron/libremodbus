/*
 * ascii_multiport.h
 *
 *  Created on: 20 июля 2016 г.
 *      Author: Radeon
 */

#ifndef MODBUS_RTU_ASCII_MULTIPORT_H_
#define MODBUS_RTU_ASCII_MULTIPORT_H_

#include "mb_types.h"
#include "serial_multi.h"

#if defined(MB_MULTIPORT) && (defined(MB_ASCII_ENABLED))


typedef enum
{
    ASCII_STATE_RX_IDLE,              /*!< Receiver is in idle state. */
	ASCII_STATE_RX_RCV,               /*!< Frame is beeing received. */
	ASCII_STATE_RX_WAIT_EOF           /*!< Wait for End of Frame. */
} ASCII_eMBRcvState;

typedef enum
{
	ASCII_STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
	ASCII_STATE_TX_START,             /*!< Starting transmission (':' sent). */
	ASCII_STATE_TX_DATA,              /*!< Sending of data (Address, Data, LRC). */
	ASCII_STATE_TX_END,               /*!< End of transmission. */
	ASCII_STATE_TX_NOTIFY,             /*!< Notify sender that the frame has been sent. */
	ASCII_STATE_TX_XFWR
} ASCII_eMBSndState;

typedef enum
{
    BYTE_HIGH_NIBBLE,           /*!< Character for high nibble of byte. */
    BYTE_LOW_NIBBLE             /*!< Character for low nibble of byte. */
} eMBBytePos;


typedef struct
{
	void* parent;
	MBSerialInstance serial_port;
	/*static*/ volatile ASCII_eMBSndState eSndState;
	/*static*/ volatile ASCII_eMBRcvState eRcvState;


	/*static*/ volatile UCHAR ucASCIIRcvBuf[128];
			   volatile UCHAR ucASCIISndBuf[128];
			   volatile USHORT usSendPDULength;

	/*static*/ volatile USHORT usRcvBufferPos;
	/*static*/ volatile eMBBytePos eBytePos;

	/*static*/ volatile UCHAR *pucSndBufferCur;
	/*static*/ volatile USHORT usSndBufferCount;

	/*static*/ volatile UCHAR ucLRC;
	/*static*/ volatile UCHAR ucMBLFCharacter;
	BOOL xFrameIsBroadcast;
	BOOL asciiMaster;
	volatile eMBMasterTimerMode eCurTimerMode;
}MBASCIIInstance;


#define ASCII_ARG      MBASCIIInstance* inst,
#define ASCII_ARG_VOID MBASCIIInstance* inst

#else

#define ASCII_ARG
#define ASCII_ARG_VOID void
#define SERIAL_ARG
#define SERIAL_ARG_VOID

#endif


#endif /* MODBUS_RTU_ASCII_MULTIPORT_H_ */
