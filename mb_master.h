/*
 * mb_master.h
 *
 *  Created on: 1 рту. 2016 у.
 *      Author: Radeon
 */

#ifndef MODBUS_MB_MASTER_H_
#define MODBUS_MB_MASTER_H_

#include "mb_types.h"

//void vMBMasterSetErrorType( eMBMasterErrorEventType errorType );

#define PR_BEGIN_EXTERN_C			extern "C" {
#define	PR_END_EXTERN_C				}

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

eMBMasterReqErrCode
eMBMasterReqReadInputRegister(MBInstance* inst, UCHAR ucSndAddr, USHORT usRegAddr, USHORT usNRegs, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqWriteHoldingRegister(MBInstance* inst, UCHAR ucSndAddr, USHORT usRegAddr, USHORT usRegData, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqWriteMultipleHoldingRegister(MBInstance* inst, UCHAR ucSndAddr, USHORT usRegAddr,
		USHORT usNRegs, USHORT * pusDataBuffer, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqReadHoldingRegister(MBInstance* inst, UCHAR ucSndAddr, USHORT usRegAddr, USHORT usNRegs, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqReadWriteMultipleHoldingRegister(MBInstance* inst, UCHAR ucSndAddr,
		USHORT usReadRegAddr, USHORT usNReadRegs, USHORT * pusDataBuffer,
		USHORT usWriteRegAddr, USHORT usNWriteRegs, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqReadCoils(MBInstance* inst, UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usNCoils, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqWriteCoil(MBInstance* inst, UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usCoilData, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqWriteMultipleCoils(MBInstance* inst, UCHAR ucSndAddr,
		USHORT usCoilAddr, USHORT usNCoils, UCHAR * pucDataBuffer, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqReadDiscreteInputs(MBInstance* inst, UCHAR ucSndAddr, USHORT usDiscreteAddr, USHORT usNDiscreteIn, LONG lTimeOut );

eMBException
eMBMasterFuncReportSlaveID(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncReadInputRegister(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncReadHoldingRegister(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncWriteHoldingRegister(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncWriteMultipleHoldingRegister(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncReadCoils(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncWriteCoil(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncWriteMultipleCoils(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncReadDiscreteInputs(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );
eMBException
eMBMasterFuncReadWriteMultipleHoldingRegister(MBInstance* inst, UCHAR * pucFrame, USHORT * usLen );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif


#endif /* MODBUS_MB_MASTER_H_ */
