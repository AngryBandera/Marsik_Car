#include "car.h"
#include "project.h"
#include "utils.h"
#include "PCF8574.h"
#include <math.h>

/////////////////////PCA9685 drive area///////////////////////////////////
#define SERVO_FREQUENCY 100     //Define the operating frequency of servo
#define MOTOR_SPEED_MIN -4095    //Define a minimum speed limit for wheels
#define MOTOR_SPEED_MAX 4095     //Define a maximum speed limit for wheels
#define PIN_MOTOR_M1_IN1 15      //Define the positive pole of M1
#define PIN_MOTOR_M1_IN2 14      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 9       //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 8       //Define the negative pole of M2
#define PIN_MOTOR_M3_IN1 12      //Define the positive pole of M3
#define PIN_MOTOR_M3_IN2 13      //Define the negative pole of M3
#define PIN_MOTOR_M4_IN1 10      //Define the positive pole of M4
#define PIN_MOTOR_M4_IN2 11      //Define the negative pole of M4

#define SOUND_PWM_CLOCK (1000000u)  // Income clock frequency

/* 8MHz IMO clock with 8000000 reload value to generate 1s interrupt */
#define SYSTICK_RELOAD_VAL   (8000UL)

static volatile uint32_t milliseconds = 0;

///////////////////// MOTORS API //////////////////////////////////////////////

//Motor & servo subsystem initialization
void Motor_Init(void) 
{
  PCA9685_Init();
  PCA9685_setToFrequency(SERVO_FREQUENCY);
}

//Function to control the car motors
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed) {
  m1_speed = MOTOR_1_DIRECTION * constrain_int(m1_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m2_speed = MOTOR_2_DIRECTION * constrain_int(m2_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m3_speed = MOTOR_3_DIRECTION * constrain_int(m3_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m4_speed = MOTOR_4_DIRECTION * constrain_int(m4_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  if (m1_speed >= 0) {
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN1, m1_speed);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN2, 0);
  } else {
    m1_speed = -m1_speed;
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN1, 0);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M1_IN2, m1_speed);
  }
  if (m2_speed >= 0) {
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN1, m2_speed);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN2, 0);
  } else {
    m2_speed = -m2_speed;
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN1, 0);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M2_IN2, m2_speed);
  }
  if (m3_speed >= 0) {
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN1, m3_speed);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN2, 0);
  } else {
    m3_speed = -m3_speed;
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN1, 0);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M3_IN2, m3_speed);
  }
  if (m4_speed >= 0) {
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN1, m4_speed);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN2, 0);
  } else {
    m4_speed = -m4_speed;
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN1, 0);
    PCA9685_setChannelPulseWidth(PIN_MOTOR_M4_IN2, m4_speed);
  }
}


///////////////////// SOUND API ///////////////////////////////////////////////

//Sound subsystem initialization
void Sound_Init(void)
{
    (void) Cy_TCPWM_PWM_Init(PWM_Buz_HW, PWM_Buz_CNT_NUM, &PWM_Buz_config);
    Cy_TCPWM_Enable_Multiple(PWM_Buz_HW, PWM_Buz_CNT_MASK);
    PWM_Buz_SetCompare0(0);
    Cy_TCPWM_TriggerStart(PWM_Buz_HW, PWM_Buz_CNT_MASK);
}

//Set permanent output (non-blocking) of tone with desired frequency
void Sound_WriteTone(uint32_t freq)
{
    uint32_t period;
    if (freq != 0u)
    {
        period = SOUND_PWM_CLOCK / freq;
        PWM_Buz_SetPeriod0(period);
        PWM_Buz_SetCompare0(period/2);
        PWM_Buz_SetCounter(0);
    }
    else
    {
        PWM_Buz_SetCompare0(0);
    }
}

//Play a tone with defined frequency and duraction
void Sound_Play(uint32_t freq, uint32_t duration)
{
    Sound_WriteTone(freq);
    CyDelay(duration);
    Sound_WriteTone(0);
}


///////////////////// TRACK SENSOR API ////////////////////////////////////////
void Track_Init(void)
{
    PCF8574_begin();
}

//Tracking module reading
uint8_t Track_Read(void)
{
  return (PCF8574_read8() & 0x7F); 
}

///////////////////// TIMING API //////////////////////////////////////////////
static void systick_handler(void)
{
    milliseconds++;
}

void Timing_Init(void)
{
    /* Enable Systick and the Systick interrupt */
    Cy_SysTick_Enable();
    
    /* Initialize the systick, set the 8MHz IMO as clock source */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, SYSTICK_RELOAD_VAL);

    /* Set Systick interrupt callback */
    Cy_SysTick_SetCallback(0, systick_handler);
}

uint32_t Timing_GetMillisecongs(void)
{
   return milliseconds;   
}