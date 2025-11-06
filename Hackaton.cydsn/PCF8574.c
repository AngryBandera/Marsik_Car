
#include "PCF8574.h"
#include "project.h"    

#define PCF8574_ADDRESS            (0x25u)    // Tracking module I2C address
#define PCF8574_INITIAL_VALUE      (0xFFu)      
#define I2C_TIMEOUT_MS             (100u)  // Timeout for I2C operations    

void PCF8574_begin(void)
{
  PCF8574_write8(PCF8574_INITIAL_VALUE);
}

uint8_t PCF8574_read8(void)
{
  cy_stc_scb_i2c_master_xfer_config_t transaction;
  static uint8_t dataPacket[1];

  // Wait for previous transaction completion
  while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context)))
  {
  }
  
  transaction.slaveAddress = PCF8574_ADDRESS;
  transaction.buffer = dataPacket;
  transaction.bufferSize = 1;
  transaction.xferPending = false;
  Cy_SCB_I2C_MasterRead(I2C_Main_HW, &transaction, &I2C_Main_context);

  return dataPacket[0];
}


void PCF8574_write8(uint8_t value)
{
  cy_stc_scb_i2c_master_xfer_config_t transaction;
  static uint8_t dataPacket[1];

  // Wait for previous transaction completion 
  while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_Main_HW, &I2C_Main_context)))
  {
  }

  dataPacket[0] = value;

  transaction.slaveAddress = PCF8574_ADDRESS;
  transaction.buffer = dataPacket;
  transaction.bufferSize = 1;
  transaction.xferPending = false;
  Cy_SCB_I2C_MasterWrite(I2C_Main_HW, &transaction, &I2C_Main_context);
}

//  -- END OF FILE --