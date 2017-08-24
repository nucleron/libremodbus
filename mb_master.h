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

mb_err_enum eMBMasterReqReadInputRegister                (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT reg_num, LONG lTimeOut                                                                       );
mb_err_enum eMBMasterReqWriteHoldingRegister             (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT usRegData, LONG lTimeOut                                                                     );
mb_err_enum eMBMasterReqWriteMultipleHoldingRegister     (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT reg_num, USHORT * pusDataBuffer, LONG lTimeOut                                               );
mb_err_enum eMBMasterReqReadHoldingRegister              (mb_instance* inst, UCHAR ucSndAddr, USHORT usRegAddr,      USHORT reg_num, LONG lTimeOut                                                                       );
mb_err_enum eMBMasterReqReadWriteMultipleHoldingRegister (mb_instance* inst, UCHAR ucSndAddr, USHORT usReadRegAddr,  USHORT usNReadRegs, USHORT * pusDataBuffer, USHORT usWriteRegAddr, USHORT usNWriteRegs, LONG lTimeOut);
mb_err_enum eMBMasterReqReadCoils                        (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT coil_num, LONG lTimeOut                                                                      );
mb_err_enum eMBMasterReqWriteCoil                        (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT usCoilData, LONG lTimeOut                                                                    );
mb_err_enum eMBMasterReqWriteMultipleCoils               (mb_instance* inst, UCHAR ucSndAddr, USHORT usCoilAddr,     USHORT coil_num, UCHAR * pucDataBuffer, LONG lTimeOut                                               );
mb_err_enum eMBMasterReqReadDiscreteInputs               (mb_instance* inst, UCHAR ucSndAddr, USHORT usDiscreteAddr, USHORT usNDiscreteIn, LONG lTimeOut                                                                 );

mb_exception_enum eMBMasterFuncReportSlaveID                    (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncReadInputRegister                (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncReadHoldingRegister              (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncWriteHoldingRegister             (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncWriteMultipleHoldingRegister     (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncReadCoils                        (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncWriteCoil                        (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncWriteMultipleCoils               (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncReadDiscreteInputs               (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);
mb_exception_enum eMBMasterFuncReadWriteMultipleHoldingRegister (mb_instance* inst, UCHAR * frame_ptr, USHORT * len_buf);

void vMBMasterErrorCBExecuteFunction(mb_instance* inst, UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength);
void vMBMasterErrorCBReceiveData(mb_instance* inst, UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength);
void vMBMasterErrorCBRespondTimeout(mb_instance* inst, UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength);
void vMBMasterCBRequestSuccess(mb_instance* inst);

mb_err_enum eMBMasterRegInputCB(mb_instance* inst, UCHAR * reg_buff, USHORT reg_addr, USHORT reg_num);
mb_err_enum eMBMasterRegHoldingCB(mb_instance* inst, UCHAR * reg_buff, USHORT reg_addr, USHORT reg_num);
mb_err_enum eMBMasterRegDiscreteCB(mb_instance* inst, UCHAR * reg_buff, USHORT reg_addr, USHORT disc_num);
mb_err_enum eMBMasterRegCoilsCB(mb_instance* inst, UCHAR * reg_buff, USHORT reg_addr, USHORT coil_num);

PR_END_EXTERN_C
#endif /* MODBUS_MB_MASTER_H_ */
