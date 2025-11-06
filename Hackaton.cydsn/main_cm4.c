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

static float Kp = 1000.0f;
static float Ki = 0.0f;
static float Kd = 0.0f;

static float lastError = 0;
static float integral = 0;

static void SendBleNotification(const char* message);
static void SendBleNotificationF(const char* format, ...);
static void processIncomingIPCMessage(ipc_msg_t* msg);
static void processCM4Command(ipc_msg_t* msg);


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
   

    CyDelay(500);

    uint32_t timeout = Timing_GetMillisecongs();
    uint32_t cycle = 0;

    // Then execute remaining code
    Leds_FillSolidColor(0, 0, 0);
    
    uint32_t bleNotifyTimer = Timing_GetMillisecongs();
    
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
        uint8_t track_copy_for_led = track; // Робимо копію, щоб не зчитувати ще раз
        for (uint8_t i=0; i<7u; i++)
        {
            Leds_PutPixel(i, track_copy_for_led & 0x01u ? 0x55u : 0x00u, 0x00u, 0x00u);
            track_copy_for_led = track_copy_for_led >> 1;
        }    
        
        if(track & 0x08){ // Центральний датчик
            Leds_PutPixel(7, 255, 0, 0); // Червоний
        }else{
            Leds_PutPixel(7, 0, 0, 0);
        }
        
        
        // Логіка ПІД-регулятора
        float error = 0.0f; 
        int8_t activeCount = 0;
        
        for (int8_t i = 0; i < 7; i++) {
            if (track & (1 << i)) {
                error += (i - 3); // від -3 до +3
                activeCount++;
            }
        }

        if (activeCount > 0)
        {
            error /= (float)activeCount; // Ділення float
        }
        // (Якщо activeCount == 0, error залишається 0, що є безпечним)

        // PID
        float derivative = error - lastError;
        integral += error;
        float correction = Kp * error + Ki * integral + Kd * derivative;
        lastError = error;

        // Керування
        // Логіка для руху задом наперед
        float baseSpeed = 1000.0f;
        float leftSpeed  = baseSpeed - correction; // Інвертована логіка
        float rightSpeed = baseSpeed + correction; // Інвертована логіка

        // обмеження
        if (leftSpeed > 4095.0f) leftSpeed = 4095.0f;
        if (rightSpeed > 4095.0f) rightSpeed = 4095.0f;
        if (leftSpeed < -4095.0f) leftSpeed = -4095.0f;
        if (rightSpeed < -4095.0f) rightSpeed = -4095.0f;

        // Конвертуємо у int лише перед відправкою в мотор
        Motor_Move((int)-leftSpeed, (int)-leftSpeed, (int)-rightSpeed, (int)-rightSpeed);
        Leds_Update();
        
        
        // Відправка телеметрії раз на секунду
        if ((Timing_GetMillisecongs() - bleNotifyTimer) >= 1000u) // 1000ms = 1 sec
        {
            // Конвертуємо float в int32_t для printf (множимо на 100)
            int32_t error_tx      = (int32_t)(error * 100);
            int32_t integral_tx   = (int32_t)(integral * 100);
            int32_t derivative_tx = (int32_t)(derivative * 100);

            // Використовуємо %ld (long) замість %f (float)
            SendBleNotificationF("Cnt:%d, Err:%ld, I:%ld, D:%ld\r\n", 
                                 activeCount, 
                                 error_tx, 
                                 integral_tx, 
                                 derivative_tx);
            
            // Скидаємо таймер
            bleNotifyTimer = Timing_GetMillisecongs();
        }
        
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
                    processCM4Command(msg);
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
static void processCM4Command(ipc_msg_t* msg)
{
    // Отримуємо ID команди з першого байта буфера
    enum cm4CommandList cmd = (enum cm4CommandList)msg->buffer[0];

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
            // Цю команду тепер можна замінити на SendBleNotification("Wroom!")
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
            // *** ВИПРАВЛЕНА ЛОГІКА ECHO ***
            // (Використовуємо 'msg', що прийшов, а не викликаємо CM4_GetCM0Message() знову)
            if (CM4_IsCM0Ready())
            {
                ipcMsgForCM0.userCode = IPC_USR_CODE_CMD;
                ipcMsgForCM0.len = msg->len; // Використовуємо довжину з 'msg'
                ipcMsgForCM0.buffer[0] = (uint8_t)CM0_SHARED_BLE_NTF_RELAY;
                
                // Копіюємо дані (все, крім першого байта команди)
                if (msg->len > 1)
                {
                    memcpy(&ipcMsgForCM0.buffer[1], &msg->buffer[1], msg->len - 1);
                }
                CM4_SendCM0Message(&ipcMsgForCM0);
            }
            break;
        }

        // =======================================================
        // +++ ДОДАНО: Обробка нових команд ПІД-регулятора +++
        // =======================================================

        case CM4_COMMAND_SET_KP:
        {
            // Очікуємо 5 байтів: 1 (команда) + 4 (float)
            if (msg->len == 5)
            {
                // Безпечно копіюємо 4 байти з буфера у нашу float змінну
                memcpy(&Kp, &msg->buffer[1], sizeof(float)); 
                SendBleNotificationF("OK: Kp set to %f\r\n", Kp);
            }
            else
            {
                SendBleNotification("ERR: Kp packet bad len\r\n");
            }
            break;
        }

        case CM4_COMMAND_SET_KI:
        {
            if (msg->len == 5)
            {
                memcpy(&Ki, &msg->buffer[1], sizeof(float)); 
                SendBleNotificationF("OK: Ki set to %f\r\n", Ki);
            }
            else
            {
                SendBleNotification("ERR: Ki packet bad len\r\n");
            }
            break;
        }

        case CM4_COMMAND_SET_KD:
        {
            if (msg->len == 5)
            {
                memcpy(&Kd, &msg->buffer[1], sizeof(float)); 
                SendBleNotificationF("OK: Kd set to %f\r\n", Kd);
            }
            else
            {
                SendBleNotification("ERR: Kd packet bad len\r\n");
            }
            break;
        }
        // =======================================================

        default:
            // Невідома команда
            SendBleNotificationF("ERR: Unknown cmd 0x%X\r\n", cmd);
            break;
    }
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


/* [] END OF FILE */