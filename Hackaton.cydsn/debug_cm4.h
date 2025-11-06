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

#ifndef DEBUG_CM4_H
#define DEBUG_CM4_H

// Please do not use debug on both cores simultaneously!
#define DEBUG_UART_ENABLED 1
    
// Override the standard printf without retarget-io
#include <stdio.h>
// Wrap debug UART functions
#include "UART_Debug.h"

#if (DEBUG_UART_ENABLED == 1)
    #define DBG_PRINTF(...)                 (printf(__VA_ARGS__))
    #define UART_DEB_PUT_CHAR(...)           while(1UL != UART_Debug_Put(__VA_ARGS__))
    #define UART_DEB_GET_CHAR(...)          (UART_Debug_Get())
    #define UART_DEB_IS_TX_COMPLETE(...)    (UART_Debug_IsTxComplete())
    #define UART_DEB_WAIT_TX_COMPLETE(...)   while(UART_Debug_IS_TX_COMPLETE() == 0) ;    
    #define UART_DEB_SCB_CLEAR_RX_FIFO(...) (Cy_SCB_ClearRxFifo(UART_Debug_SCB__HW))
    #define UART_START(...)                 (UART_Debug_Start(__VA_ARGS__))
#else
    #define DBG_PRINTF(...)
    #define UART_DEB_PUT_CHAR(...)
    #define UART_DEB_GET_CHAR(...)          (0u)
    #define UART_DEB_IS_TX_COMPLETE(...)    (1u)
    #define UART_DEB_WAIT_TX_COMPLETE(...)  (0u)
    #define UART_DEB_SCB_CLEAR_RX_FIFO(...) (0u)
    #define UART_START(...)
#endif /* (DEBUG_UART_ENABLED == ENABLED) */

#define UART_DEB_NO_DATA                (char8) CY_SCB_UART_RX_NO_DATA

#endif /* DEBUG_CM0_H */

/* [] END OF FILE */
