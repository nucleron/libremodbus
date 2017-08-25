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

mb_err_enum  mb_mstr_rq_read_inp_reg           (mb_instance *inst, UCHAR snd_addr, USHORT reg_addr,      USHORT reg_num,      LONG timeout                                                         );
mb_err_enum  mb_mstr_rq_write_holding_reg      (mb_instance *inst, UCHAR snd_addr, USHORT reg_addr,      USHORT reg_data,     LONG timeout                                                         );
mb_err_enum  mb_mstr_rq_write_multi_holding_reg(mb_instance *inst, UCHAR snd_addr, USHORT reg_addr,      USHORT reg_num,      USHORT *data_ptr, LONG timeout                                       );
mb_err_enum  mb_mstr_rq_read_holding_reg       (mb_instance *inst, UCHAR snd_addr, USHORT reg_addr,      USHORT reg_num,      LONG timeout                                                         );
mb_err_enum  mb_mstr_rq_rw_multi_holding_reg   (mb_instance *inst, UCHAR snd_addr, USHORT rd_reg_addr,   USHORT rd_reg_num,   USHORT *data_ptr, USHORT wr_reg_addr, USHORT wr_reg_num, LONG timeout);
mb_err_enum  mb_mstr_rq_read_coils             (mb_instance *inst, UCHAR snd_addr, USHORT coil_addr,     USHORT coil_num,     LONG timeout                                                         );
mb_err_enum  mb_mstr_rq_write_coil             (mb_instance *inst, UCHAR snd_addr, USHORT coil_addr,     USHORT coil_data,    LONG timeout                                                         );
mb_err_enum  mb_mstr_rq_write_multi_coils      (mb_instance *inst, UCHAR snd_addr, USHORT coil_addr,     USHORT coil_num,     UCHAR *data_ptr,  LONG timeout                                       );
mb_err_enum  mb_mstr_rq_read_discrete_inputs   (mb_instance *inst, UCHAR snd_addr, USHORT discrete_addr, USHORT discrete_num, LONG timeout                                                         );

//mb_exception_enum  mb_mstr_fn_report_slv_id                    (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_read_inp_reg           (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_read_holding_reg       (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_write_holding_reg      (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_write_multi_holding_reg(mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_read_coils             (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_write_coil             (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_write_multi_coils      (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_read_discrete_inputs   (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);
mb_exception_enum  mb_mstr_fn_rw_multi_holding_regs  (mb_instance *inst, UCHAR *frame_ptr, USHORT *len_buf);

void mb_mstr_error_exec_fn_cb (mb_instance *inst, UCHAR dst_addr, const UCHAR* pdu_data_ptr, USHORT pdu_len);
void mb_mstr_error_rcv_data_cb(mb_instance *inst, UCHAR dst_addr, const UCHAR* pdu_data_ptr, USHORT pdu_len);
void mb_mstr_error_timeout_cb (mb_instance *inst, UCHAR dst_addr, const UCHAR* pdu_data_ptr, USHORT pdu_len);
void mb_mstr_rq_success_cb    (mb_instance *inst                                                           );

mb_err_enum  mb_mstr_reg_input_cb   (mb_instance *inst, UCHAR *reg_buff, USHORT reg_addr, USHORT reg_num );
mb_err_enum  mb_mstr_reg_holding_cb (mb_instance *inst, UCHAR *reg_buff, USHORT reg_addr, USHORT reg_num );
mb_err_enum  mb_mstr_reg_discrete_cb(mb_instance *inst, UCHAR *reg_buff, USHORT reg_addr, USHORT disc_num);
mb_err_enum  mb_mstr_reg_coils_cb   (mb_instance *inst, UCHAR *reg_buff, USHORT reg_addr, USHORT coil_num);

PR_END_EXTERN_C
#endif /* MODBUS_MB_MASTER_H_ */
