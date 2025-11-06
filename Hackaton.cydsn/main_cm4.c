#include <project.h>
#include "car.h"
#include "music.h"
#include "cm4_common.h"

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
    
    //Music_FurElise();

    //Sound_Play(440u,1000u);
   
//    Motor_Move(2000, 2000, 2000, 2000);     //go forward
//    CyDelay(1000);
//    Motor_Move(0, 0, 0, 0);                 //stop
//    CyDelay(1000);
//    Motor_Move(-1000, -1000, -1000, -1000); //go back
//    CyDelay(2000);
//    Motor_Move(0, 0, 0, 0);                 //stop
//    CyDelay(1000);

//    Motor_Move(-2000, -2000, 2000, 2000);   //turn left
//    CyDelay(1000);
//    Motor_Move(0, 0, 0, 0);                 //stop
//    CyDelay(1000);
//    Motor_Move(3000, 3000, -3000, -3000);   //turn right
//    CyDelay(750);
//    Motor_Move(0, 0, 0, 0);                 //stop
//    CyDelay(1000);   

    uint32_t timeout = Timing_GetMillisecongs();
    uint32_t cycle = 0;

    // Then execute remaining code
    Leds_FillSolidColor(0, 0, 0);
    
    // MAIN LOOP   
    for(;;)
    {
        // Check for new messages from CM0 core and process them. This is the most important task.
        if (CM4_isDataAvailableFromCM0())
        {
            processIncomingIPCMessage(CM4_GetCM0Message());
        }
       
        // Blink Reg and blue LEDs based on timer
       // if((Timing_GetMillisecongs() - timeout) > 1000u)
      //  {
           // if (cycle == 0u)
           // {
               // Leds_PutPixel(7u, 0x00u, 0x55u, 0x00u);
               // Leds_PutPixel(10u, 0x00u, 0x00u, 0x55u);
             //   cycle++;
           // }
           // else             
           // {
                //Leds_PutPixel(7u, 0x00u, 0x00u, 0x55u);
               // Leds_PutPixel(10u, 0x00u, 0x55u, 0x00u);
             //   cycle = 0;
           // }
         //   timeout = Timing_GetMillisecongs();
       // }
        
        // Duplicate track sensor on Smart LEDs
        uint8_t track = Track_Read();
        for (uint8_t i=0; i<7u; i++)
        {
            Leds_PutPixel(i,track & 0x01u ? 0x55u : 0x00u, 0x00u, 0x00u);
            track = track >> 1;
        }    

        Leds_Update();
       
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