/*
 * serial.h
 */

#ifndef SERIAL_H_
#define SERIAL_H_

int32_t serial_initialize(void);
int32_t serial_uninitialize(void);
int32_t serial_reset(void);
int32_t serial_set_configuration(void *config);
int32_t serial_write_free(void);
int32_t serial_write_data(uint8_t *data, uint16_t size);
int32_t serial_read_data(uint8_t *data, uint16_t size);

#endif /* SERIAL_H_ */
