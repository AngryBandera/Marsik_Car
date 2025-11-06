#include <project.h>
#include "car.h"
#include "music.h"
#include "cm4_common.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Implement ISR for I2C_1 */
void I2C_1_Isr(void)
{
    Cy_SCB_I2C_Interrupt(I2C_Main_HW, &I2C_Main_context);
}

/* Allocate buffer */
#define I2C_BUFFER_SIZE (128UL)
uint8_t i2C_buffer[I2C_BUFFER_SIZE];

ipc_msg_t ipcMsgForCM0 = {               /* IPC structure to be sent to CM0 */
    .clientId = IPC_CM4_TO_CM0_CLIENT_ID,
    .userCode = 0,
    .intrMask = CY_SYS_CYPIPE_INTR_MASK,
    .buffer   = {},
    .len      = 0
};
static void SendBleNotificationF(const char* format, ...)
{
    // Буфер тепер локальний, всередині функції (на стеку)
    // Переконайтеся, що він достатньо великий для ваших повідомлень
    char ble_buffer[128]; 
    
    // Обробка змінних аргументів (va_list)
    va_list args;
    va_start(args, format);
    
    // Створюємо відформатований рядок у нашому буфері
    // vsnprintf - це версія snprintf, що приймає va_list
    vsnprintf(ble_buffer, sizeof(ble_buffer), format, args);
    
    va_end(args);
    
    // Тепер просто викликаємо нашу стару функцію з готовим буфером
    SendBleNotification(ble_buffer);
}

static void SendBleNotification(const char* message)
{
    if (!CM4_IsCM0Ready())
    {
        DBG_PRINTF("CM0 not ready, cannot send BLE msg\r\n");
        return;
    }
    size_t messageLen = strlen(message);
    
    if ((messageLen + 1) > sizeof(ipcMsgForCM0.buffer))
    {
        DBG_PRINTF("BLE Msg too long, truncating.\r\n");
        messageLen = sizeof(ipcMsgForCM0.buffer) - 1;
    }

    ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
    
    ipcMsgForCM0.buffer[0] = (uint8_t)CM0_SHARED_BLE_NTF_RELAY;
    
    memcpy(&ipcMsgForCM0.buffer[1], message, messageLen);
    
    ipcMsgForCM0.len = messageLen + 1;

    CM4_SendCM0Message(&ipcMsgForCM0);
    
    DBG_PRINTF("Sent BLE: %s\r\n", message);
}
static void processIncomingIPCMessage(ipc_msg_t* msg);
static void processCM4Command(enum cm4CommandList cmd);


int main(void)
{    
    /* SETUP */
    UART_START();
    DBG_PRINTF("Hello world!\r\n");
    
    // Initialize SCB for I2C operation, and configure desired data rate.  
    // Hook I2C interrupt service routine and enable interrupt 
    (void)Cy_SCB_I2C_Init(I2C_Main_HW, &I2C_Main_config, &I2C_Main_context);
    (void)Cy_SCB_I2C_SetDataRate(I2C_Main_HW, I2C_Main_DATA_RATE_HZ, I2C_Main_CLK_FREQ_HZ);
    Cy_SysInt_Init(&I2C_Main_SCB_IRQ_cfg, &I2C_1_Isr);
    NVIC_EnableIRQ(I2C_Main_SCB_IRQ_cfg.intrSrc);
    Cy_SCB_I2C_Enable(I2C_Main_HW);
    
    // Enable global interrupts.
    __enable_irq(); 
    
    /* Register the Message Callback */
    Cy_IPC_Pipe_RegisterCallback(CY_IPC_EP_CYPIPE_ADDR, CM4_MessageCallback, CY_IPC_EP_CYPIPE_CM0_ADDR);
    
    // Initialize SPI LED controller
    (void)Leds_Init();
    
    // Initialize driver that controls motors
    Motor_Init(); 

    // Initialize sound driver
    Sound_Init();

    // Initialize line tracking driver
    Track_Init();
    
    //Initialize timing driver
    Timing_Init();
    
    // Turn on LEDs on PSoC6 board
    Cy_GPIO_Clr(LEDG_0_PORT, LEDG_0_NUM); //green LED
    Cy_GPIO_Clr(LEDR_0_PORT, LEDR_0_NUM); //red LED
   

//    CyDelay(1000);

    uint32_t timeout = Timing_GetMillisecongs();
    uint32_t cycle = 0;

    // Then execute remaining code
    Leds_FillSolidColor(0, 0, 0);
    
    // MAIN LOOP   
    SendBleNotification("Wroom! from CM4!");
    for(;;)
    {
        // Check for new messages from CM0 core and process them. This is the most important task.
        if (CM4_isDataAvailableFromCM0())
        {
            processIncomingIPCMessage(CM4_GetCM0Message());
        }
       
        
        // Duplicate track sensor on Smart LEDs
        uint8_t track = Track_Read();
        for (uint8_t i=0; i<7u; i++)
        {
            Leds_PutPixel(i,track & 0x01u ? 0x55u : 0x00u, 0x00u, 0x00u);
            track = track >> 1;
        }    
        
        track = Track_Read();
        if(track & 0x08){
            Leds_PutPixel(7, 255, 0, 0);
        }else{
            Leds_PutPixel(7, 0, 0, 0);
        }
        
        track = Track_Read();
        
        int16_t error = 0;
        int8_t activeCount = 0;
        
        // рахуєм відхилення
        for (int8_t i = 0; i < 7; i++) {
            if (track & (1 << i)) {
                error += (i - 3); // від -3 до +3
                activeCount++;
            }
        }

        if (activeCount > 0)
            error /= activeCount;

        // PID
        float Kp = 500.0f;
        float Ki = 0.0f;
        float Kd = 200.0f;

        static float lastError = 0;
        static float integral = 0;

        float derivative = error - lastError;
        integral += error;
        float correction = Kp * error + Ki * integral + Kd * derivative;
        lastError = error;

        // керування
        int baseSpeed = 1000;
        int leftSpeed  = baseSpeed + correction;
        int rightSpeed = baseSpeed - correction;

        // обмеження
        if (leftSpeed > 4095) leftSpeed = 4095;
        if (rightSpeed > 4095) rightSpeed = 4095;
        if (leftSpeed < -4095) leftSpeed = -4095;
        if (rightSpeed < -4095) rightSpeed = -4095;

        Motor_Move(-leftSpeed, -leftSpeed, -rightSpeed, -rightSpeed);

        Leds_Update();
        
        SendBleNotificationF("Count = %d\r\n", activeCount);
        SendBleNotificationF("Err:%ld, P:%ld, D:%ld", error, p_term, d_term);
       
        CyDelay(50);
    }
}

static void processIncomingIPCMessage(ipc_msg_t* msg)
{
    // In general, impossible situation, but never trust anyone.
    if (msg != NULL)
    {
        // Process messages with at least 1 byte of payload
        if (msg->len > 0)
        {
// Decode message code. You would mostly like to use Commands
            switch (msg->userCode)
            {
                case IPC_USR_CODE_CMD:
                {
                    // Only [0] element was used for simplicity.
                    // IDEA: You could expand it to accept strings or more complex data packets!
                    // You can define array of strings and perform memcmp. This operation is a bit more costful.
                    processCM4Command((enum cm4CommandList)msg->buffer[0]);
                    break;
                }
                case IPC_USR_CODE_REQ:
                {
                    // Not implemented. You can be very creative here!
                    break;
                }
                default:
                    break;
            }
        }
    }
}

static void processCM4Command(enum cm4CommandList cmd)
{
    switch (cmd)
    {
        case CM4_COMMAND_LED_ENA:
        {
            Cy_GPIO_Write(LEDG_0_PORT, LEDG_0_NUM, 0);
            Cy_GPIO_Write(LEDR_0_PORT, LEDR_0_NUM, 0);
            break;
        }
        case CM4_COMMAND_LED_DIS:
        {
            Cy_GPIO_Write(LEDG_0_PORT, LEDG_0_NUM, 1);
            Cy_GPIO_Write(LEDR_0_PORT, LEDR_0_NUM, 1);
            break;
        }
        case CM4_COMMAND_CAR_SAY:
        {
            if (CM4_IsCM0Ready())
            {
                ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
                ipcMsgForCM0.len = 0x01;
                ipcMsgForCM0.buffer[0] = (uint8_t)CM0_SHARED_CAR_SAY;
                CM4_SendCM0Message(&ipcMsgForCM0);
            }
            break;
        }
        case CM4_COMMAND_ECHO:
        {
            if (CM4_IsCM0Ready())
            {
                ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
                ipcMsgForCM0.len = CM4_GetCM0Message()->len;
                ipcMsgForCM0.buffer[0] = (uint8_t)CM0_SHARED_BLE_NTF_RELAY;
                memcpy(&ipcMsgForCM0.buffer[1], &CM4_GetCM0Message()->buffer[1], ipcMsgForCM0.len - 1);
                CM4_SendCM0Message(&ipcMsgForCM0);
            }
            break;
        }
        default:
            break;
    }
}



/* [] END OF FILE */