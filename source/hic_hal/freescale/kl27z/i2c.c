#include "i2c.h"
#include "fsl_i2c.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "settings.h" // for config_get_overflow_detect

/* I2C source clock */
#define I2C_SLAVE_BASEADDR I2C1
#define I2C_SLAVE_CLK_SRC I2C1_CLK_SRC
#define I2C_SLAVE_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)

#define I2C_MASTER_SLAVE_ADDR_7BIT  0x7AU
#define I2C_DATA_LENGTH             (32U)

// We'll start with 5 RX commands
#define RX_CMDS_LENGTH              5
#define I2C_CMD_SLEEP               0xABU

static uint8_t g_slave_buff[I2C_DATA_LENGTH];
static i2c_slave_handle_t g_s_handle;
static volatile bool g_SlaveCompletionFlag = false;
static volatile bool g_SlaveRxFlag = false;

static i2cWriteCallback_t pfWriteCallback = NULL;
static i2cReadCallback_t pfReadCallback = NULL;

static void i2c_slave_callback(I2C_Type *base, i2c_slave_transfer_t *xfer, void *userData) {
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
            xfer->data     = g_slave_buff;
            xfer->dataSize = I2C_DATA_LENGTH;
            break;

        /*  Receive request */
        case kI2C_SlaveReceiveEvent:
            /*  Update information for received process */
            // Hack: Default driver can't differentiate between RX or TX on
            // completion event, so we set a flag here. Can't process more
            // than I2C_DATA_LENGTH bytes on RX
            memset(&g_slave_buff, 0, sizeof(g_slave_buff));
            xfer->data     = g_slave_buff;
            xfer->dataSize = I2C_DATA_LENGTH;
            g_SlaveRxFlag = true;
            break;

        /*  Transfer done */
        case kI2C_SlaveCompletionEvent:
            g_SlaveCompletionFlag = true;
            xfer->data            = NULL;
            xfer->dataSize        = 0;
            
            // Default driver couldn't differentiate between RX or TX completion
            // Check flag set in kI2C_SlaveReceiveEvent
            if (g_SlaveRxFlag) {
                g_SlaveRxFlag = false;
                
                if (pfWriteCallback) {
                    pfWriteCallback(&g_slave_buff[0], xfer->transferredCount);
                }
            } else { 
                if (pfReadCallback) {
                    pfReadCallback(&g_slave_buff[0], xfer->transferredCount);
                }
            }
            break;

        default:
            g_SlaveCompletionFlag = false;
            break;
    }
}

static void i2c_init_pins(void) {
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

static int32_t i2c_start_transfer(void) {
    memset(&g_slave_buff, 0, sizeof(g_slave_buff));
    memset(&g_s_handle, 0, sizeof(g_s_handle));

    I2C_SlaveTransferCreateHandle(I2C_SLAVE_BASEADDR, &g_s_handle,
            i2c_slave_callback, NULL);

    /* Set up slave transfer. */
    I2C_SlaveTransferNonBlocking(I2C_SLAVE_BASEADDR, &g_s_handle,
            kI2C_SlaveCompletionEvent | kI2C_SlaveAddressMatchEvent);

    return 1;
}

int32_t i2c_initialize(void) {
    i2c_slave_config_t slaveConfig;
    
    i2c_init_pins();

    I2C_SlaveGetDefaultConfig(&slaveConfig);

    slaveConfig.addressingMode = kI2C_Address7bit;
    slaveConfig.slaveAddress   = I2C_MASTER_SLAVE_ADDR_7BIT;
    slaveConfig.enableWakeUp   = true;

    I2C_SlaveInit(I2C_SLAVE_BASEADDR, &slaveConfig, I2C_SLAVE_CLK_FREQ);

    i2c_start_transfer();

    return 1;
}

int32_t i2c_deinitialize(void) {
    I2C_SlaveDeinit(I2C_SLAVE_BASEADDR);
    return 1;
}

void i2c_RegisterWriteCallback(i2cWriteCallback_t writeCallback)
{
    pfWriteCallback = writeCallback;
    return;
}

void i2c_RegisterReadCallback(i2cReadCallback_t readCallback)
{
    pfReadCallback = readCallback;
    return;
}

void i2c_fillBuffer (uint8_t* data, uint8_t size) {
    if (size > I2C_DATA_LENGTH) {
        size = I2C_DATA_LENGTH;
    }
    
    memset(&g_slave_buff, 0, sizeof(g_slave_buff));
    
    for (uint32_t i = 0U; i < size; i++) { 
            g_slave_buff[i] = data[i];
    }
}

