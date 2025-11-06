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
#include "project.h"
#include "cm0p_common.h"

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    #if(CY_BLE_CONFIG_HOST_CORE == CY_BLE_CORE_CORTEX_M0P)   
    
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
   
    
    /* Run BLE Logger */
    BleNUSInit();

    #endif

    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    //Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
    
    for(;;)
    {    
        
    }
}

/* [] END OF FILE */
