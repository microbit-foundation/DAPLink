#include "i2c.h"
#include "circ_buf.h"
#include "fsl_i2c.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "settings.h" // for config_get_overflow_detect

/* I2C source clock */
#define I2C_SLAVE_BASEADDR I2C1
#define I2C_SLAVE_CLK_SRC I2C1_CLK_SRC
#define I2C_SLAVE_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)

#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU

#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (32)
#define I2C_DATA_LENGTH     (BUFFER_SIZE + 2)

circ_buf_t i2c_write_buffer;
uint8_t i2c_write_buffer_data[BUFFER_SIZE];
circ_buf_t i2c_read_buffer;
uint8_t i2c_read_buffer_data[BUFFER_SIZE];

uint8_t g_slave_buff[I2C_DATA_LENGTH];

i2c_slave_handle_t g_s_handle;
volatile bool g_SlaveCompletionFlag = false;

static int32_t i2c_start_transfer(void);

static void i2c_slave_callback(I2C_Type *base, i2c_slave_transfer_t *xfer, void *userData)
{
    static uint8_t g_slave_temp_var = 0;
    uint8_t size = 0;
    uint32_t free = 0;

    switch (xfer->event)
    {
        /*  Address match event */
        case kI2C_SlaveAddressMatchEvent:
            xfer->data     = NULL;
            xfer->dataSize = 0;
            break;
        /*  Transmit request */
        case kI2C_SlaveTransmitEvent:
            /*  Update information for transmit process */
            size = circ_buf_count_used(&i2c_write_buffer);
            if (NULL == xfer->data){    // Send buffer size in first byte
                g_slave_temp_var = size;
            } else {                    // Send data in the rest of the transfer
                if (size > 0) { // Only pop data if there's data
                    g_slave_temp_var = circ_buf_pop(&i2c_write_buffer);
                } else {        // Otherwise send dummy 0's
                    g_slave_temp_var = 0;
                }
            }
            xfer->data     = &g_slave_temp_var;
            xfer->dataSize = 1;
            break;

        /*  Receive request */
        case kI2C_SlaveReceiveEvent:
            /*  Update information for received process */
            xfer->data     = g_slave_buff;
            xfer->dataSize = I2C_DATA_LENGTH;
            break;

        /*  Transfer done */
        case kI2C_SlaveCompletionEvent:
            g_SlaveCompletionFlag = true;
            xfer->data            = NULL;
            xfer->dataSize        = 0;

            // Store received data
            free = circ_buf_count_free(&i2c_read_buffer);

            if (free >= g_slave_buff[1] + RX_OVRF_MSG_SIZE) {
                // Whole slave buff fits in read buffer
                circ_buf_write(&i2c_read_buffer, &g_slave_buff[2], g_slave_buff[1]);
            } else {
                if (config_get_overflow_detect()) { // Overflow detection enabled
                    // Write only the data that fits and drop newest
                    if (free > RX_OVRF_MSG_SIZE) {
                        circ_buf_write(&i2c_read_buffer, &g_slave_buff[2], free - RX_OVRF_MSG_SIZE);
                    }
                    circ_buf_write(&i2c_read_buffer, (uint8_t*) RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
                } else { // Overflow detection not enabled
                    // Write whatever fits in the buffer and drop newest
                    circ_buf_write(&i2c_read_buffer, &g_slave_buff[2], g_slave_buff[1]);
                }
            }

            memset(&g_slave_buff, 0, sizeof(g_slave_buff));
            break;

        default:
            g_SlaveCompletionFlag = false;
            break;
    }
}

static void clear_buffers(void)
{
    circ_buf_init(&i2c_write_buffer, i2c_write_buffer_data, sizeof(i2c_write_buffer_data));
    circ_buf_init(&i2c_read_buffer, i2c_read_buffer_data, sizeof(i2c_read_buffer_data));
}

static void i2c_init_pins(void)
{
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* PORTC1 is configured as I2C1_SCL */
    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAlt2);

    PORTC->PCR[1] = ((PORTC->PCR[1] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin. */
                     | (uint32_t)(PORT_PCR_PE_MASK));

    /* PORTC2 is configured as I2C1_SDA */
    PORT_SetPinMux(PORTC, 2U, kPORT_MuxAlt2);

    PORTC->PCR[2] = ((PORTC->PCR[2] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin. */
                     | (uint32_t)(PORT_PCR_PE_MASK));
}

int32_t i2c_initialize(void)
{
    i2c_slave_config_t slaveConfig;

    i2c_init_pins();

    clear_buffers();

    I2C_SlaveGetDefaultConfig(&slaveConfig);

    slaveConfig.addressingMode = kI2C_Address7bit;
    slaveConfig.slaveAddress   = I2C_MASTER_SLAVE_ADDR_7BIT;

    I2C_SlaveInit(I2C_SLAVE_BASEADDR, &slaveConfig, I2C_SLAVE_CLK_FREQ);

    i2c_start_transfer();

    return 1;
}

int32_t i2c_deinitialize(void)
{
    I2C_SlaveDeinit(I2C_SLAVE_BASEADDR);
    return 1;
}

static int32_t i2c_start_transfer(void)
{
    memset(&g_slave_buff, 0, sizeof(g_slave_buff));
    memset(&g_s_handle, 0, sizeof(g_s_handle));

    I2C_SlaveTransferCreateHandle(I2C_SLAVE_BASEADDR, &g_s_handle,
            i2c_slave_callback, NULL);

    /* Set up slave transfer. */
    I2C_SlaveTransferNonBlocking(I2C_SLAVE_BASEADDR, &g_s_handle,
            kI2C_SlaveCompletionEvent | kI2C_SlaveAddressMatchEvent);

    return 1;
}

int32_t i2c_reset(void)
{
    clear_buffers();
    return 1;
}

int32_t i2c_write_free(void)
{
    return circ_buf_count_free(&i2c_write_buffer);
}

int32_t i2c_write_data(uint8_t *data, uint16_t size)
{
    uint32_t free = 0;
    uint32_t cnt = 0;

    free = circ_buf_count_free(&i2c_write_buffer);

    if (free >= size + RX_OVRF_MSG_SIZE) {
        // Whole slave buff fits in write buffer
        cnt = circ_buf_write(&i2c_write_buffer, data, size);
    } else {
        if (config_get_overflow_detect()) { // Overflow detection enabled
            // Write only the data that fits and drop newest
            if (free > RX_OVRF_MSG_SIZE) {
                cnt = circ_buf_write(&i2c_write_buffer, data, free - RX_OVRF_MSG_SIZE);
            }
            cnt += circ_buf_write(&i2c_write_buffer, (uint8_t*) RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else { // Overflow detection not enabled
            // Write whatever fits in the buffer and drop newest
            cnt = circ_buf_write(&i2c_write_buffer, data, size);
        }
    }

    return cnt;
}

int32_t i2c_read_data(uint8_t *data, uint16_t size)
{
    return circ_buf_read(&i2c_read_buffer, data, size);
}
