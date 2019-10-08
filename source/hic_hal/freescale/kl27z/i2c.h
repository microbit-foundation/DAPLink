/*
 * i2c.h
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include "stdint.h"

int32_t i2c_initialize(void);
int32_t i2c_deinitialize(void);
int32_t i2c_reset(void);
int32_t i2c_write_free(void);
int32_t i2c_write_data(uint8_t *data, uint16_t size);
int32_t i2c_read_data(uint8_t *data, uint16_t size);

#endif /* I2C_H_ */
