#ifndef MB_TYPES_H
#define MB_TYPES_H

#include <mb_common.h>
PR_BEGIN_EXTERN_C
/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup modbus
 * \brief Modbus serial transmission modes (RTU/ASCII).
 *
 * Modbus serial supports two transmission modes. Either ASCII or RTU. RTU
 * is faster but has more hardware requirements and requires a network with
 * a low jitter. ASCII is slower and more reliable on slower links (E.g. modems)
 */
    typedef enum
{
    MB_RTU,                     /*!< RTU transmission mode. */
    MB_ASCII,                   /*!< ASCII transmission mode. */
    MB_TCP                      /*!< TCP mode. */
} mb_mode_enum;

/*! \ingroup modbus
 * \brief If register should be written or read.
 *
 * This value is passed to the callback functions which support either
 * reading or writing register values. Writing means that the application
 * registers should be updated and reading means that the modbus protocol
 * stack needs to know the current register values.
 *
 * \see eMBRegHoldingCB(), eMBRegCoilsCB(), eMBRegDiscreteCB() and
 *   eMBRegInputCB().
 */
typedef enum
{
    MB_REG_READ,                /*!< Read register values and pass to protocol stack. */
    MB_REG_WRITE                /*!< Update register values. */
} mb_reg_mode_enum;

/*! \ingroup modbus
 * \brief Event types used by all function in the protocol stack.
 */
typedef enum
{
    EV_READY,                   /*!< Startup finished. */
    EV_FRAME_RECEIVED,          /*!< Frame received. */
    EV_EXECUTE,                 /*!< Execute function. */
    EV_FRAME_SENT,               /*!< Frame sent. */
	/*master events*/
	//EV_ERROR_PROCESS,
//	EV_ERROR_RESPOND_TIMEOUT,
//
#if MB_MASTER >0
    EV_MASTER_ERROR_RESPOND_TIMEOUT,  /*!< Request respond timeout. */
    EV_MASTER_ERROR_RECEIVE_DATA,     /*!< Request receive data error. */
    EV_MASTER_ERROR_EXECUTE_FUNCTION, /*!< Request execute function error. */
#endif
} mb_event_enum;

//typedef enum
//{
//    ERR_EV_ERROR_RESPOND_TIMEOUT,         /*!< Slave respond timeout. */
//    ERR_EV_ERROR_RECEIVE_DATA,            /*!< Receive frame data erroe. */
//    ERR_EV_ERROR_EXECUTE_FUNCTION,        /*!< Execute function error. */
//} eMBMasterErrorEventType;

/*! \ingroup modbus
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    MB_ENOERR,                  /*!< no error. */
    MB_ENOREG,                  /*!< illegal register address. */
    MB_EINVAL,                  /*!< illegal argument. */
    MB_EPORTERR,                /*!< porting layer error. */
    MB_ENORES,                  /*!< insufficient resources. */
    MB_EIO,                     /*!< I/O error. */
    MB_EILLSTATE,               /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT,               /*!< timeout error occurred. */
    MB_EILLFUNC,                /*!< illegal MB function. */
    MB_EBUSY                   /*!< master is busy now. */
} mb_err_enum;

/* master */
//typedef enum
//{
//    MB_ENOERR,                  /*!< no error. */
//    MB_ENOREG,                  /*!< illegal register address. */
//    MB_EINVAL,                 /*!< illegal argument. */
//    MB_ERCV,                /*!< receive data error. */
//    MB_ETIMEDOUT,                /*!< timeout error occurred. */
//    MB_EBUSY,             /*!< master is busy now. */
//    MB_EFNEXEC                  /*!< execute function error. */
//} mb_err_enum;
/*! \ingroup modbus
 *  \brief TimerMode is Master 3 kind of Timer modes.
 */
typedef enum
{
	MB_TMODE_T35,                   /*!< Master receive frame T3.5 timeout. */
	MB_TMODE_RESPOND_TIMEOUT,       /*!< Master wait respond for slave. */
	MB_TMODE_CONVERT_DELAY          /*!< Master sent broadcast , then delay sometime.*/
}mb_tmr_mode_enum;

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

typedef uint32_t DWORD;

PR_END_EXTERN_C
#endif // MB_TYPES_H
