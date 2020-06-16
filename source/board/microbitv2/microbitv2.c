/**
 * @file    microbitv2.c
 * @brief   board ID for the BBC Microbit board
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "fsl_device_registers.h"
#include "IO_Config.h"
#include "DAP.h"
#include "target_family.h"
#include "target_board.h"
#include "flexio_pwm.h"
#include "gpio.h"
#include "power.h"
#include "rl_usb.h" 
#include "pwr_mon.h"
#include "main.h"
#include "i2c.h"
#include "i2c_commands.h"
#include "adc.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "led_error_app.h"
#include "flash_manager.h"

#ifdef DRAG_N_DROP_SUPPORT
#include "flash_intf.h"
#endif

#define M0_RESERVED_VECT_OFFSET     (4 * 4)
#define M0_RESERVED_VECT_SIZE       (3 * 4) // Size for mem fault, bus fault and usage fault vectors

#define BRD_ID_1_UPPER_THR_V        935   // Upper threshold in mV for 100nF and 4700R
#define BRD_ID_1_LOWER_THR_V        268   // Lower threshold in mV for 100nF and 4700R

#define PWR_LED_ON_BATT_BRIGHTNESS      40 // LED Brightness while being powered by battery
#define PWR_LED_FADEOUT_MIN_BRIGHTNESS  30 // Minimum LED brightness when fading out

const char * const board_id_mb_2_0 = "9903";
const char * const board_id_mb_2_1 = "9904";

uint16_t board_id_hex_min = 0x9903;
uint16_t board_id_hex = 0;

typedef enum {
    BOARD_VERSION_2_0 = 0,
    BOARD_VERSION_2_1 = 1,
} mb_version_t;

extern target_cfg_t target_device_nrf52_64;
extern main_usb_connect_t usb_state;
extern bool go_to_sleep;

typedef enum main_shutdown_state {
    MAIN_SHUTDOWN_WAITING = 0,
    MAIN_SHUTDOWN_PENDING,
    MAIN_SHUTDOWN_REACHED,
    MAIN_SHUTDOWN_REQUESTED,
    MAIN_LED_BLINK,
    MAIN_LED_FULL_BRIGHTNESS,
    MAIN_SHUTDOWN_CANCEL
} main_shutdown_state_t;

extern void main_powerdown_event(void);

static void i2c_write_callback(uint8_t* pData, uint8_t size);
static void i2c_read_callback(uint8_t* pData, uint8_t size);

// shutdown state
static main_shutdown_state_t main_shutdown_state = MAIN_SHUTDOWN_WAITING;
static uint8_t shutdown_led_dc = 100;
static uint8_t power_led_max_duty_cycle = 100;
static app_power_mode_t interface_power_mode = kAPP_PowerModeVlls0;
static power_source_t power_source;

// Board Rev ID detection. Reads BRD_REV_ID voltage
// Depends on gpio_init() to have been executed already
static mb_version_t read_brd_rev_id_pin(void) {
    gpio_pin_config_t pin_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    mb_version_t board_version = BOARD_VERSION_2_0;
    uint32_t board_rev_id_adc = 0;
    uint32_t board_rev_id_mv = 0;

    // Set Board Rev ID pin as output but pin disabled
    PORT_SetPinMux(PIN_BOARD_REV_ID_PORT , PIN_BOARD_REV_ID_BIT,  kPORT_PinDisabledOrAnalog);
    PORT_SetPinDriveStrength(PIN_BOARD_REV_ID_PORT, PIN_BOARD_REV_ID_BIT, kPORT_HighDriveStrength);
    GPIO_PinInit(PIN_BOARD_REV_ID_GPIO, PIN_BOARD_REV_ID_BIT, &pin_config);

    adc_init();
    
    // 1. Discharge capacitor
    //    Drive BRD_REV_ID pin to low
    GPIO_PortClear(PIN_BOARD_REV_ID_GPIO, PIN_BOARD_REV_ID);
    PORT_SetPinMux(PIN_BOARD_REV_ID_PORT , PIN_BOARD_REV_ID_BIT,  kPORT_MuxAsGpio);
    //    Add a 3ms delay to allow the 100nF Cap to discharge 
    //    at least 5*RC with 4700R.
    for (uint32_t count = 16 * 3000; count > 0UL; count--);
    
    // 2. Charge capacitor for 100us
    //    Drive BRD_REV_ID pin to high
    GPIO_PortSet(PIN_BOARD_REV_ID_GPIO, PIN_BOARD_REV_ID);
    //    Add a ~100us delay
    //    3 clock cycles per loop at -O2 ARMCC optimization
    for (uint32_t count = 1600; count > 0UL; count--);
    //    Change pin to ADC (High-Z). Capacitor will stop charging
    PORT_SetPinMux(PIN_BOARD_REV_ID_PORT , PIN_BOARD_REV_ID_BIT,  kPORT_PinDisabledOrAnalog);
    
    // 3. Take ADC measurement
    board_rev_id_adc = adc_read_channel(0, PIN_BOARD_REV_ID_ADC_CH, PIN_BOARD_REV_ID_ADC_MUX);
    board_rev_id_mv = board_rev_id_adc * 3300 / 0xFFF;  // Convert ADC 12-bit value to mV with 3.3V reference
    
    // 4. Discharge capacitor
    //    Drive BRD_REV_ID pin to low
    GPIO_PortClear(PIN_BOARD_REV_ID_GPIO, PIN_BOARD_REV_ID);
    PORT_SetPinMux(PIN_BOARD_REV_ID_PORT , PIN_BOARD_REV_ID_BIT,  kPORT_MuxAsGpio);
    //    Add a 3ms delay to allow the 100nF Cap to discharge 
    //    at least 5*RC with 4700R.
    for (uint32_t count = 16 * 3000; count > 0UL; count--);
    
    // 5. Identify board ID depending on voltage
    if ( board_rev_id_mv > BRD_ID_1_LOWER_THR_V && board_rev_id_mv < BRD_ID_1_UPPER_THR_V) {
        board_version = BOARD_VERSION_2_1;
    } else {
        board_version = BOARD_VERSION_2_0;
    }
    
    return board_version;
}

static void set_board_id(mb_version_t board_version) {
    target_device = target_device_nrf52_64;
    switch (board_version) {
        case BOARD_VERSION_2_0:
            target_device.rt_board_id = board_id_mb_2_0;
            board_id_hex = 0x9903;
            break;
        case BOARD_VERSION_2_1:
            target_device.rt_board_id = board_id_mb_2_1;
            board_id_hex = 0x9904;
            break;
        default:
            target_device.rt_board_id = board_id_mb_2_0;
            board_id_hex = 0x9903;
            break;
    }
}

// Apply a gamma curve to the LED. Input brightness between 0-100
static inline uint8_t get_led_gamma(uint8_t brightness) {
    uint8_t duty_cycle;
    duty_cycle = (brightness * brightness * brightness + 5000) / 10000;
    return duty_cycle;
}

// Called in main_task() to init before USB and files are configured
static void prerun_board_config(void)
{
    mb_version_t board_version = read_brd_rev_id_pin();
    set_board_id(board_version);
    
    // init power monitoring
    power_init();
    pwr_mon_init();

    power_source = pwr_mon_get_power_source();

    flexio_pwm_init();
    flexio_pwm_init_pins();
    
    if (power_source == PWR_BATT_ONLY){
        // Turn on the red LED with low duty cycle to conserve power.
        power_led_max_duty_cycle = PWR_LED_ON_BATT_BRIGHTNESS;
        
    } else {
        // Turn on the red LED with max duty cycle when powered by USB or EC
        power_led_max_duty_cycle = 100;
    }
    uint8_t gamma_led_dc = get_led_gamma(power_led_max_duty_cycle);
    flexio_pwm_set_dutycycle(gamma_led_dc);
    
    i2c_initialize();
    i2c_RegisterWriteCallback(i2c_write_callback);
    i2c_RegisterReadCallback(i2c_read_callback);
    
    gpio_pin_config_t pin_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };

    /* COMBINED_SENSOR_INT pin mux ALT0 (Disabled High-Z) */
    PORT_SetPinMux(COMBINED_SENSOR_INT_PORT, COMBINED_SENSOR_INT_PIN, kPORT_PinDisabledOrAnalog);
    /* COMBINED_SENSOR_INT as output default low when pin mux ALT1 */
    GPIO_PinInit(COMBINED_SENSOR_INT_GPIO, COMBINED_SENSOR_INT_PIN, &pin_config);
}

// Handle the reset button behavior, this function is called in the main task every 30ms
void handle_reset_button()
{
    // button state
    static uint8_t reset_pressed = 0;
    // reset button state count
    static uint16_t gpio_reset_count = 0;

    // handle reset button without eventing
    if (!reset_pressed && gpio_get_reset_btn_fwrd()) {
#ifdef DRAG_N_DROP_SUPPORT
        if (!flash_intf_target->flash_busy()) //added checking if flashing on target is in progress
#endif
        {
            // Reset button pressed
            target_set_state(RESET_HOLD);
            reset_pressed = 1;
            gpio_reset_count = 0;
            main_shutdown_state = MAIN_LED_FULL_BRIGHTNESS;
        }
    } else if (reset_pressed && !gpio_get_reset_btn_fwrd()) {
        // Reset button released
        target_set_state(RESET_RUN);
        reset_pressed = 0;

        if (gpio_reset_count <= RESET_SHORT_PRESS) {
            main_shutdown_state = MAIN_LED_BLINK;
        }
        else if (gpio_reset_count < RESET_LONG_PRESS) {
            // Indicate button has been released to stop to cancel the shutdown
            main_shutdown_state = MAIN_LED_BLINK;
        }
        else if (gpio_reset_count >= RESET_LONG_PRESS) {
            // Indicate the button has been released when shutdown is requested
            interface_power_mode = kAPP_PowerModeVlls0;
            main_shutdown_state = MAIN_SHUTDOWN_REQUESTED;
        }
    } else if (reset_pressed && gpio_get_reset_btn_fwrd()) {
        // Reset button is still pressed
        if (gpio_reset_count > RESET_SHORT_PRESS && gpio_reset_count < RESET_LONG_PRESS) {
            // Enter the shutdown pending state to begin LED dimming
            main_shutdown_state = MAIN_SHUTDOWN_PENDING;
        }
        else if (gpio_reset_count >= RESET_LONG_PRESS) {
            // Enter the shutdown reached state to blink LED
            main_shutdown_state = MAIN_SHUTDOWN_REACHED;
        }

        // Avoid overflow, stop counting after longest event
        if (gpio_reset_count <= RESET_MAX_LENGTH_PRESS) {
            gpio_reset_count++;
        }
    }
}

void board_30ms_hook()
{
  static uint8_t blink_in_progress = 0;
    
    if (usb_state == USB_CONNECTED) {
      // configure pin as GPIO
      PIN_HID_LED_PORT->PCR[PIN_HID_LED_BIT] = PORT_PCR_MUX(1);
      power_led_max_duty_cycle = 100;
    }
    else if (usb_state == USB_DISCONNECTED) {
      // Disable pin
      PIN_HID_LED_PORT->PCR[PIN_HID_LED_BIT] = PORT_PCR_MUX(0);
      power_led_max_duty_cycle = PWR_LED_ON_BATT_BRIGHTNESS;
    }

    switch (main_shutdown_state) {
      case MAIN_LED_FULL_BRIGHTNESS:
          // Jump power LED to full brightness
          shutdown_led_dc = 100;
          break;
      case MAIN_SHUTDOWN_CANCEL:
          main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          // Set the PWM value back to max duty cycle
          shutdown_led_dc = power_led_max_duty_cycle;
          break;
      case MAIN_SHUTDOWN_PENDING:
          // Fade the PWM until the board is about to be shut down
          if (shutdown_led_dc > PWR_LED_FADEOUT_MIN_BRIGHTNESS) {
              shutdown_led_dc--;
          }
          break;
      case MAIN_SHUTDOWN_REACHED:
          // Turn off LED to indicate we are waiting for release
          shutdown_led_dc = 0;
          break;
      case MAIN_SHUTDOWN_REQUESTED:
          // TODO:  put nRF into deep sleep and wake nRF when KL27 wakes up
          if (power_source == PWR_BATT_ONLY || usb_state == USB_DISCONNECTED) {
              main_powerdown_event();
          }
          main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          break;
      case MAIN_SHUTDOWN_WAITING:
          // Set the PWM value back to max duty cycle
          shutdown_led_dc = power_led_max_duty_cycle;
          break;
      case MAIN_LED_BLINK:
          
          if (blink_in_progress) {
            blink_in_progress--;
            if (blink_in_progress == 5) {
              shutdown_led_dc = 0;
            }
          } else {
            blink_in_progress = 10;
            shutdown_led_dc = 100;
          }
          
          if (blink_in_progress == 0) { 
            main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          }
          break;
      default:
          break;
    }
    uint8_t gamma_led_dc = get_led_gamma(shutdown_led_dc);
    flexio_pwm_set_dutycycle(gamma_led_dc);
}

void board_handle_powerdown()
{
    switch(interface_power_mode){
        case kAPP_PowerModeVlps:
            power_enter_VLPS();
            break;
        case kAPP_PowerModeVlls0:
            power_enter_VLLS0();
            break;
        default:
            break;
    }
    
    i2c_deinitialize();
    i2c_initialize();
    
    usbd_connect(1);
    
    gpio_set_hid_led(HID_LED_DEF);
}

void board_error_hook(error_t error) {
    error_t ret;
    
    nrf52833_xxaa_bin[1984] = (uint8_t) error & 0xFF;
    nrf52833_xxaa_bin[1985] = (uint8_t) (error >> 8) & 0xFF;

    ret = flash_manager_init(flash_intf_target);
    if (ret != ERROR_SUCCESS) {
        util_assert(0);
    }

    ret = flash_manager_data(0, (const uint8_t*)nrf52833_xxaa_bin, nrf52833_xxaa_bin_len);
    if (ret != ERROR_SUCCESS) {
        flash_manager_uninit();
        util_assert(0);
    }

    ret = flash_manager_uninit();
    if (ret != ERROR_SUCCESS) {
        util_assert(0);
    }
}

uint8_t board_detect_incompatible_image(const uint8_t *data, uint32_t size)
{
    uint8_t result = 0;
    
    // Check difference in vectors (mem fault, bus fault, usage fault)
    // If these vectors are 0, we assume it's an M0 image (not compatible)
    result = memcmp(data + M0_RESERVED_VECT_OFFSET, (uint8_t[M0_RESERVED_VECT_SIZE]){0}, M0_RESERVED_VECT_SIZE);
    
    return result == 0;
}

// USB HID override function return 1 if the activity is trivial or response is null
uint8_t usbd_hid_no_activity(uint8_t *buf)
{
    if(buf[0] == ID_DAP_Vendor3 &&  buf[1] == 0)
        return 1;
    else
        return 0;
}

const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .family_id = kNordic_Nrf52_FamilyID,
    .daplink_url_name =       "MICROBITHTM",
    .daplink_drive_name =       "MICROBIT",
    .daplink_target_url = "https://microbit.org/device/?id=@B&v=@V",
    .prerun_board_config = prerun_board_config,
    .target_cfg = &target_device,
};

static void i2c_write_callback(uint8_t* pData, uint8_t size) {
    i2cCommand_t* pI2cCommand = (i2cCommand_t*) pData;
    i2cCommand_t i2cResponse = {0};

    switch (pI2cCommand->cmdId) {
        case gReadRequest_c:
            i2cResponse.cmdId = gReadResponse_c;
            i2cResponse.cmdData.readRspCmd.propertyId = pI2cCommand->cmdData.readReqCmd.propertyId;
            switch (pI2cCommand->cmdData.readReqCmd.propertyId) {
                case gDAPLinkBoardVersion_c:
                    i2cResponse.cmdData.readRspCmd.dataSize = sizeof(board_id_hex);
                    memcpy(&i2cResponse.cmdData.readRspCmd.data, &board_id_hex, sizeof(board_id_hex));
                break;
                case gI2CProtocolVersion_c: {
                    uint32_t i2c_version = 1;
                    i2cResponse.cmdData.readRspCmd.dataSize = sizeof(i2c_version);
                    memcpy(&i2cResponse.cmdData.readRspCmd.data, &i2c_version, sizeof(i2c_version));
                }
                break;
                case gDAPLinkVersion_c: {
                    uint32_t daplink_version = DAPLINK_VERSION;
                    i2cResponse.cmdData.readRspCmd.dataSize = sizeof(daplink_version);
                    memcpy(&i2cResponse.cmdData.readRspCmd.data, &daplink_version, sizeof(daplink_version));
                }
                break;
                case gPowerState_c:
                    i2cResponse.cmdData.readRspCmd.dataSize = sizeof(power_source);
                    memcpy(&i2cResponse.cmdData.readRspCmd.data, &power_source, sizeof(power_source));
                break;
                case gPowerConsumption_c: {
                    uint32_t power_consumption = 1;
                    i2cResponse.cmdData.readRspCmd.dataSize = sizeof(power_consumption);
                    memcpy(&i2cResponse.cmdData.readRspCmd.data, &power_consumption, sizeof(power_consumption));
                }
                break;
                case gUSBEnumerationState_c:
                    i2cResponse.cmdData.readRspCmd.dataSize = sizeof(usb_state);
                    memcpy(&i2cResponse.cmdData.readRspCmd.data, &usb_state, sizeof(usb_state));
                break;
                case gPowerMode_c:
                case gNRFPowerMode_c:
                    i2cResponse.cmdId = gErrorResponse_c;
                    i2cResponse.cmdData.errorRspCmd.errorCode = gErrorReadDisallowed_c;
                break;
                default:
                    i2cResponse.cmdId = gErrorResponse_c;
                    i2cResponse.cmdData.errorRspCmd.errorCode = gErrorUnknownProperty_c;
                break;
            }
        break;
        case gReadResponse_c:
            i2cResponse.cmdId = gErrorResponse_c;
            i2cResponse.cmdData.errorRspCmd.errorCode = gErrorCommandDisallowed_c;
        break;
        case gWriteRequest_c:
            switch (pI2cCommand->cmdData.writeReqCmd.propertyId) {
                case gDAPLinkBoardVersion_c:
                case gI2CProtocolVersion_c:
                case gDAPLinkVersion_c:
                case gPowerState_c:
                case gPowerConsumption_c:
                case gUSBEnumerationState_c:
                    i2cResponse.cmdId = gErrorResponse_c;
                    i2cResponse.cmdData.errorRspCmd.errorCode = gErrorWriteDisallowed_c;
                break;
                case gPowerMode_c:
                    if (pI2cCommand->cmdData.writeReqCmd.dataSize == 1) {
                        if (pI2cCommand->cmdData.writeReqCmd.data[0] == kAPP_PowerModeVlls0) {
                            if (power_source == PWR_BATT_ONLY || usb_state == USB_DISCONNECTED) {
                                interface_power_mode = kAPP_PowerModeVlls0;
                                i2cResponse.cmdId = gWriteResponse_c;
                                i2cResponse.cmdData.writeRspCmd.propertyId = pI2cCommand->cmdData.writeReqCmd.propertyId;
                            } else {
                                i2cResponse.cmdId = gErrorResponse_c;
                                i2cResponse.cmdData.errorRspCmd.errorCode = gErrorWriteDisallowed_c;
                            }
                        } else if (pI2cCommand->cmdData.writeReqCmd.data[0] == kAPP_PowerModeVlps) {
                            if (power_source == PWR_BATT_ONLY || usb_state == USB_DISCONNECTED) {
                                interface_power_mode = kAPP_PowerModeVlps;
                                i2cResponse.cmdId = gWriteResponse_c;
                                i2cResponse.cmdData.writeRspCmd.propertyId = pI2cCommand->cmdData.writeReqCmd.propertyId;
                            } else {
                                i2cResponse.cmdId = gErrorResponse_c;
                                i2cResponse.cmdData.errorRspCmd.errorCode = gErrorWriteDisallowed_c;
                            }
                        } else { 
                            i2cResponse.cmdId = gErrorResponse_c;
                            i2cResponse.cmdData.errorRspCmd.errorCode = gErrorWriteFail_c;
                        }
                    } else {
                        i2cResponse.cmdId = gErrorResponse_c;
                        i2cResponse.cmdData.errorRspCmd.errorCode = gErrorWrongPropertySize_c;
                    }
                break;
                case gNRFPowerMode_c:
                    i2cResponse.cmdId = gErrorResponse_c;
                    i2cResponse.cmdData.errorRspCmd.errorCode = gErrorWriteDisallowed_c;
                break;
                default:
                    i2cResponse.cmdId = gErrorResponse_c;
                    i2cResponse.cmdData.errorRspCmd.errorCode = gErrorUnknownProperty_c;
                break;
            }
        break;
        case gWriteResponse_c:
        break;
        case gErrorResponse_c:
        break;
        default:
            i2cResponse.cmdId = gErrorResponse_c;
            i2cResponse.cmdData.errorRspCmd.errorCode = gErrorUnknownProperty_c;
        break;
    }
    
    i2c_fillBuffer((uint8_t*) &i2cResponse, sizeof(i2cResponse));
    
    // Response ready, assert COMBINED_SENSOR_INT
    PORT_SetPinMux(COMBINED_SENSOR_INT_PORT, COMBINED_SENSOR_INT_PIN, kPORT_MuxAsGpio);
}

static void i2c_read_callback(uint8_t* pData, uint8_t size) {
    i2cCommand_t* pI2cCommand = (i2cCommand_t*) pData;

    switch (pI2cCommand->cmdId) {
        case gWriteResponse_c:
            switch (pI2cCommand->cmdData.writeRspCmd.propertyId) {
                case gPowerMode_c:
                    main_shutdown_state = MAIN_SHUTDOWN_REQUESTED;
                break;
            }
        break;
    }
    
    // Release COMBINED_SENSOR_INT
    PORT_SetPinMux(COMBINED_SENSOR_INT_PORT, COMBINED_SENSOR_INT_PIN, kPORT_PinDisabledOrAnalog);
}

