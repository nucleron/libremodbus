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

mb_err_enum eMBMasterReqReadInputRegister                (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usNRegs, LONG lTimeOut                                                                       );
mb_err_enum eMBMasterReqWriteHoldingRegister             (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usRegData, LONG lTimeOut                                                                     );
mb_err_enum eMBMasterReqWriteMultipleHoldingRegister     (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usNRegs, USHORT * pusDataBuffer, LONG lTimeOut                                               );
mb_err_enum eMBMasterReqReadHoldingRegister              (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usNRegs, LONG lTimeOut                                                                       );
mb_err_enum eMBMasterReqReadWriteMultipleHoldingRegister (mb_instance* inst, UCHAR ucSndAddr, USHORT usReadRegAddr,  USHORT usNReadRegs, USHORT * pusDataBuffer, USHORT usWriteRegAddr, USHORT usNWriteRegs, LONG lTimeOut);
mb_err_enum eMBMasterReqReadCoils                        (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT usNCoils, LONG lTimeOut                                                                      );
mb_err_enum eMBMasterReqWriteCoil                        (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT usCoilData, LONG lTimeOut                                                                    );
mb_err_enum eMBMasterReqWriteMultipleCoils               (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT usNCoils, UCHAR * pucDataBuffer, LONG lTimeOut                                               );
mb_err_enum eMBMasterReqReadDiscreteInputs               (mb_instance* inst, UCHAR ucSndAddr, USHORT usDiscreteAddr, USHORT usNDiscreteIn, LONG lTimeOut                                                                 );

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

mb_err_enum eMBMasterRegInputCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);
mb_err_enum eMBMasterRegHoldingCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs);
mb_err_enum eMBMasterRegDiscreteCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete);
mb_err_enum eMBMasterRegCoilsCB(mb_instance* inst, UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils);

PR_END_EXTERN_C
#endif /* MODBUS_MB_MASTER_H_ */
