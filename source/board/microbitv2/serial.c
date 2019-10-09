/*
 * serial.c
 */
#include "i2c.h"

int32_t serial_initialize(void)
{
    return i2c_initialize();
}

int32_t serial_uninitialize(void)
{
    return i2c_deinitialize();
}

int32_t serial_reset(void)
{
    return i2c_reset();
}

int32_t serial_set_configuration(void *config)
{
    return 1;
}

int32_t serial_write_free(void)
{
    return i2c_write_free();
}

int32_t serial_write_data(uint8_t *data, uint16_t size)
{
    return i2c_write_data(data, size);
}

int32_t serial_read_data(uint8_t *data, uint16_t size)
{
    return i2c_read_data(data, size);
}
