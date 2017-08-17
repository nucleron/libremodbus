/*
 * mb_master.h
 *
 *  Created on: 1 рту. 2016 у.
 *      Author: Radeon
 */

#ifndef MODBUS_MB_MASTER_H_
#define MODBUS_MB_MASTER_H_

#include <mb_common.h>
PR_BEGIN_EXTERN_C

#include "mb_types.h"
//void vMBMasterSetErrorType(eMBMasterErrorEventType errorType);

eMBMasterReqErrCode eMBMasterReqReadInputRegister                (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usNRegs, LONG lTimeOut                                                                       );
eMBMasterReqErrCode eMBMasterReqWriteHoldingRegister             (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usRegData, LONG lTimeOut                                                                     );
eMBMasterReqErrCode eMBMasterReqWriteMultipleHoldingRegister     (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usNRegs, USHORT * pusDataBuffer, LONG lTimeOut                                               );
eMBMasterReqErrCode eMBMasterReqReadHoldingRegister              (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usNRegs, LONG lTimeOut                                                                       );
eMBMasterReqErrCode eMBMasterReqReadWriteMultipleHoldingRegister (mb_instance* inst, UCHAR ucSndAddr, USHORT usReadRegAddr,  USHORT usNReadRegs, USHORT * pusDataBuffer, USHORT usWriteRegAddr, USHORT usNWriteRegs, LONG lTimeOut);
eMBMasterReqErrCode eMBMasterReqReadCoils                        (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT usNCoils, LONG lTimeOut                                                                      );
eMBMasterReqErrCode eMBMasterReqWriteCoil                        (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT usCoilData, LONG lTimeOut                                                                    );
eMBMasterReqErrCode eMBMasterReqWriteMultipleCoils               (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT usNCoils, UCHAR * pucDataBuffer, LONG lTimeOut                                               );
eMBMasterReqErrCode eMBMasterReqReadDiscreteInputs               (mb_instance* inst, UCHAR ucSndAddr, USHORT usDiscreteAddr, USHORT usNDiscreteIn, LONG lTimeOut                                                                 );

eMBException eMBMasterFuncReportSlaveID                    (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncReadInputRegister                (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncReadHoldingRegister              (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncWriteHoldingRegister             (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncWriteMultipleHoldingRegister     (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncReadCoils                        (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncWriteCoil                        (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncWriteMultipleCoils               (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncReadDiscreteInputs               (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);
eMBException eMBMasterFuncReadWriteMultipleHoldingRegister (mb_instance* inst, UCHAR * pucFrame, USHORT * usLen);

void vMBMasterErrorCBExecuteFunction(mb_instance* inst, UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength);
void vMBMasterErrorCBReceiveData(mb_instance* inst, UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength);
void vMBMasterErrorCBRespondTimeout(mb_instance* inst, UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength);
void vMBMasterCBRequestSuccess(mb_instance* inst);

eMBErrorCode eMBMasterRegInputCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);
eMBErrorCode eMBMasterRegHoldingCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);
eMBErrorCode eMBMasterRegDiscreteCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete);
eMBErrorCode eMBMasterRegCoilsCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils);

PR_END_EXTERN_C
#endif /* MODBUS_MB_MASTER_H_ */
