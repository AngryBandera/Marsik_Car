/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#ifndef CM4_COMMON_H
#define CM4_COMMON_H
    
#include "ipc_def.h"
#include "cm4_ipc.h"
#include "debug_cm4.h"
#include "cm4_command_list.h"
#include "cm0_cm4_shared_cmd.h"
#include "ledctrl.h"

enum cm4MessageTypes
{
    CM4_MESSAGE_TYPE_START = 0x00,
    CM4_MESSAGE_TYPE_CMD,
    CM4_MESSAGE_TYPE_REQ,
    CM4_MESSAGE_TYPE_END = CM4_MESSAGE_TYPE_REQ,
};
    
#endif /* CM4_COMMON_H */

/* [] END OF FILE */
