/*
 * i2c.h
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include "fsl_common.h"

#define I2C_SLAVE_NRF_KL_COMMS      (0x70U)
#define I2C_SLAVE_HID               (0x71U)
#define I2C_SLAVE_FLASH             (0x72U)

/*! i2c Write Callback prototype */
typedef void (*i2cWriteCallback_t)
(
    uint8_t*    pData,
    uint8_t     size
);

/*! i2c Read Callback prototype */
typedef void (*i2cReadCallback_t)
(
    uint8_t*    pData,
    uint8_t     size
);

int32_t i2c_initialize(void);
int32_t i2c_deinitialize(void);
status_t i2c_RegisterWriteCallback(i2cWriteCallback_t writeCallback, uint8_t slaveAddress);
status_t i2c_RegisterReadCallback(i2cReadCallback_t readCallback, uint8_t slaveAddress);
void i2c_clearBuffer(void); 
void i2c_fillBuffer(uint8_t* data, uint32_t position, uint32_t size);

#endif /* I2C_H_ */
