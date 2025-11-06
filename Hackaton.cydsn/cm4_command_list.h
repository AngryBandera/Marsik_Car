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

#ifndef CM4_COMMAND_LIST_H
#define CM4_COMMAND_LIST_H
    
// У файлі cm4_common.h (або де оголошено enum cm4CommandList)

enum cm4CommandList
{
    CM4_COMMAND_LED_ENA = 0x01,
    CM4_COMMAND_LED_DIS = 0x02,
    CM4_COMMAND_CAR_SAY = 0x03,
    CM4_COMMAND_ECHO    = 0x04,
    
    // --- ДОДАЙТЕ ЦІ РЯДКИ ---
    // (Використовуємо 0x10, 0x11... щоб уникнути конфліктів)
    CM4_COMMAND_SET_KP  = 0x10,
    CM4_COMMAND_SET_KI  = 0x11,
    CM4_COMMAND_SET_KD  = 0x12
    // ------------------------
};

#endif /* CM4_COMMAND_LIST_H */

/* [] END OF FILE */
