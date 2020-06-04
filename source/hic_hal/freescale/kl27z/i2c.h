/*
 * i2c.h
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include "fsl_common.h"

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
void i2c_RegisterWriteCallback(i2cWriteCallback_t writeCallback);
void i2c_RegisterReadCallback(i2cReadCallback_t readCallback);
void i2c_fillBuffer(uint8_t* data, uint8_t size);

#endif /* I2C_H_ */
