#ifndef MB_MULTIPORT_H
#define MB_MULTIPORT_H


#include "mb_types.h"
#include <mbframe.h>
#include <mbproto.h>
#include <mbconfig.h>

#include "mbport.h"

typedef enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState ;

#if MB_MULTIPORT > 0
    #if MB_RTU_ENABLED == 1
        #define RTU_MULTIPORT
        #include "mbrtu.h"
        #include "rtu_multiport.h"
    #endif
    #if MB_ASCII_ENABLED == 1
        #define ASCII_MULTIPORT
        #include "mbascii.h"
        #include "ascii_multiport.h"
    #endif
    #if MB_TCP_ENABLED == 1
        #define TCP_MULTIPORT
        #include "mbtcp.h"
        //#include "tcp_multiport.h"
        #include "tcp_multi.h"
    #endif

typedef BOOL( *mbBoolFunc) ( CALLBACK_ARG );

typedef struct
{
	void* transport;
	void* port;

	UCHAR    ucMBAddress;
	eMBMode  eMBCurrentMode;
	eMBState eMBCurrentState;

	volatile UCHAR *rxFrame;
	volatile UCHAR *txFrame;

	volatile USHORT   usLength;

	volatile USHORT* PDUSndLength;

	mbBoolFunc pxMBFrameCBByteReceived;
	mbBoolFunc pxMBFrameCBTransmitterEmpty;
	mbBoolFunc pxMBPortCBTimerExpired;

	BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
	BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );

	peMBFrameSend peMBFrameSendCur;
	pvMBFrameStart pvMBFrameStartCur;
	pvMBFrameStop pvMBFrameStopCur;
	peMBFrameReceive peMBFrameReceiveCur;
	pvMBFrameClose pvMBFrameCloseCur;

	pvPortEventPost pvPortEventPostCur;
	pvPortEventGet pvPortEventGetCur;

	pbMBMasterRequestIsBroadcast pbMBMasterRequestIsBroadcastCur;

	pvGetRxFrame pvMBGetRxFrame;
	pvGetRxFrame pvMBGetTxFrame;

	xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX];

	//for slave id
	UCHAR    ucMBSlaveID[MB_FUNC_OTHER_REP_SLAVEID_BUF];
	USHORT   usMBSlaveIDLen;

	//master variables
	BOOL xMBRunInMasterMode;
	UCHAR ucMBMasterDestAddress;
	eMBMasterErrorEventType  eMBMasterCurErrorType;

} MBInstance;

#define MB_MULTI_ARG MBInstance* inst,
#else

#define MB_MULTI_ARG

#endif // MB_MULTIPORT



#endif // MB_MULTIPORT_H
