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
#include "adc.h"

#ifdef DRAG_N_DROP_SUPPORT
#include "flash_intf.h"
#endif

#define M0_RESERVED_VECT_OFFSET     (4 * 4)
#define M0_RESERVED_VECT_SIZE       (3 * 4) // Size for mem fault, bus fault and usage fault vectors

#define BRD_REV_ID_100R_V             3023  // 3.023 mV for 100nF and 100R
#define BRD_REV_ID_680R_V             2081  // 2.081 mV for 100nF and 680R
#define BRD_REV_ID_1500R_V            935   // 0.935 mV for 100nF and 1500R
#define BRD_REV_ID_4700R_V            268   // 0.268 mV for 100nF and 4700R
#define BRD_REV_ID_0R_V               0     // 0 mV for 0R

#define POWER_LED_LOW_DUTY_CYCLE      10

const char * const board_id_mb_2_0 = "9903";
const char * const board_id_mb_2_1 = "9904";
const char * const board_id_mb_2_2 = "9905";
const char * const board_id_mb_2_3 = "9906";
const char * const board_id_mb_2_4 = "9907";

uint16_t board_id_hex = 0;

typedef enum {
    BOARD_VERSION_2_0 = 0,
    BOARD_VERSION_2_1 = 1,
    BOARD_VERSION_2_2 = 2,
    BOARD_VERSION_2_3 = 3,
    BOARD_VERSION_2_4 = 4,
} mb_version_t;

extern target_cfg_t target_device_nrf52_64;
extern main_usb_connect_t usb_state;
extern bool go_to_sleep;

typedef enum main_shutdown_state {
    MAIN_SHUTDOWN_WAITING = 0,
    MAIN_SHUTDOWN_PENDING,
    MAIN_SHUTDOWN_1_REACHED,
    MAIN_SHUTDOWN_1_REQUESTED,
    MAIN_SHUTDOWN_2_REACHED,
    MAIN_SHUTDOWN_2_REQUESTED,
    MAIN_LED_BLINK,
    MAIN_LED_FULL_BRIGHTNESS,
    MAIN_SHUTDOWN_CANCEL
} main_shutdown_state_t;

extern void main_powerdown_event(void);

// shutdown state
static main_shutdown_state_t main_shutdown_state = MAIN_SHUTDOWN_WAITING;
static uint8_t shutdown_led_dc = 100;
static uint8_t power_led_max_duty_cycle = 100;
static app_power_mode_t interface_power_mode;
static power_source_t power_source;

// Board Rev ID detection. Reads BRD_REV_ID voltage
// Depends on gpio_init() to have been executed already
static mb_version_t read_brd_rev_id_pin(void) {
    mb_version_t board_version = BOARD_VERSION_2_0;
    uint32_t board_rev_id_adc = 0;
    uint32_t board_rev_id_mv = 0;

    adc_init();
    
    // 1. Check for older boards with R3 populated. (C10 will be charged)
    //    If C10 voltage is near VREG, it's an old board.
    //    In newer boards C10 will be discharged.
    //    Take ADC measurement
    PIN_BOARD_REV_ID_PORT->PCR[PIN_BOARD_REV_ID_BIT] &= ~PORT_PCR_MUX(1);
    board_rev_id_adc = adc_read_channel(0, PIN_BOARD_REV_ID_ADC_CH, PIN_BOARD_REV_ID_ADC_MUX);
    board_rev_id_mv = board_rev_id_adc * 3300 / 0xFFF;  // Convert ADC 12-bit value to mV with 3.3V reference
    
    if (board_rev_id_mv > BRD_REV_ID_100R_V) {
        board_version = BOARD_VERSION_2_0;
    } else {
        // 2. Discharge capacitor
        //    Drive BRD_REV_ID pin to low
        PIN_BOARD_REV_ID_PORT->PCR[PIN_BOARD_REV_ID_BIT] |= PORT_PCR_MUX(1);
        gpio_set_brd_rev_id(GPIO_OFF);
        //    Add a 3ms delay to allow the 100nF Cap to discharge 
        //    at least 5*RC with 4700R.
        for (uint32_t count = 16 * 3000; count > 0UL; count--);
        
        // 3. Charge capacitor for 100us
        //    Drive BRD_REV_ID pin to high
        gpio_set_brd_rev_id(GPIO_ON);
        //    Add a ~100us delay
        //    3 clock cycles per loop at -O2 ARMCC optimization
        for (uint32_t count = 1600; count > 0UL; count--);
        //    Change pin to ADC (High-Z). Capacitor will stop charging
        PIN_BOARD_REV_ID_PORT->PCR[PIN_BOARD_REV_ID_BIT] &= ~PORT_PCR_MUX(1);
        
        // 4. Take ADC measurement
        board_rev_id_adc = adc_read_channel(0, PIN_BOARD_REV_ID_ADC_CH, PIN_BOARD_REV_ID_ADC_MUX);
        board_rev_id_mv = board_rev_id_adc * 3300 / 0xFFF;  // Convert ADC 12-bit value to mV with 3.3V reference
        
        // 5. Discharge capacitor
        //    Drive BRD_REV_ID pin to low
        PIN_BOARD_REV_ID_PORT->PCR[PIN_BOARD_REV_ID_BIT] |= PORT_PCR_MUX(1);
        gpio_set_brd_rev_id(GPIO_OFF);
        //    Add a 3ms delay to allow the 100nF Cap to discharge 
        //    at least 5*RC with 4700R.
        for (uint32_t count = 16 * 3000; count > 0UL; count--);
        
        // 6. Identify board ID depending on voltage
        if ( board_rev_id_mv > BRD_REV_ID_100R_V ) {
            board_version = BOARD_VERSION_2_3;
        } else if ( board_rev_id_mv > BRD_REV_ID_680R_V ) {
            board_version = BOARD_VERSION_2_2;
        } else if ( board_rev_id_mv > BRD_REV_ID_1500R_V ) {
            board_version = BOARD_VERSION_2_1;
        } else if ( board_rev_id_mv > BRD_REV_ID_4700R_V ) {
            board_version = BOARD_VERSION_2_0;
        } else if ( board_rev_id_mv >= BRD_REV_ID_0R_V ) {
            board_version = BOARD_VERSION_2_4;
        }
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
        case BOARD_VERSION_2_2:
            target_device.rt_board_id = board_id_mb_2_2;
            board_id_hex = 0x9905;
            break;
        case BOARD_VERSION_2_3:
            target_device.rt_board_id = board_id_mb_2_3;
            board_id_hex = 0x9906;
            break;
        case BOARD_VERSION_2_4:
            target_device.rt_board_id = board_id_mb_2_4;
            board_id_hex = 0x9907;
            break;
        default:
            target_device.rt_board_id = board_id_mb_2_0;
            board_id_hex = 0x9903;
            break;
    }
}

// Called in main_task() to init before USB and files are configured
static void prerun_board_config(void)
{
    mb_version_t board_version = read_brd_rev_id_pin();
    set_board_id(board_version);
    
    // init power monitoring
    pwr_mon_init();

    power_source = pwr_mon_get_power_source();

    flexio_pwm_init();
    flexio_pwm_init_pins();
    
    if (power_source == PWR_BATT_ONLY){
        // Turn on the red LED with low duty cycle to conserve power.
        power_led_max_duty_cycle = POWER_LED_LOW_DUTY_CYCLE;
        
    } else {
        // Turn on the red LED with max duty cycle when powered by USB or EC
        power_led_max_duty_cycle = 100;
    }
    flexio_pwm_set_dutycycle(power_led_max_duty_cycle);

    power_init();
    
    i2c_initialize();
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
            reset_pressed = 1;
            gpio_reset_count = 0;
            main_shutdown_state = MAIN_LED_FULL_BRIGHTNESS;
        }
    } else if (reset_pressed && !gpio_get_reset_btn_fwrd()) {
        // Reset button released
        reset_pressed = 0;

        if (gpio_reset_count <= RESET_SHORT_PRESS) {
            target_set_state(RESET_RUN);
            main_shutdown_state = MAIN_LED_BLINK;
        }
        else if (gpio_reset_count < RESET_LONG_PRESS) {
            // Indicate button has been released to stop to cancel the shutdown
            main_shutdown_state = MAIN_SHUTDOWN_CANCEL;
        }
        else if (gpio_reset_count >= RESET_LONG_PRESS && gpio_reset_count < RESET_VERY_LONG_PRESS) {
            // Indicate the button has been released when shutdown is requested
            main_shutdown_state = MAIN_SHUTDOWN_1_REQUESTED;
        }
        else if (gpio_reset_count >= RESET_VERY_LONG_PRESS) {
            // Indicate the button has been released when shutdown is requested
            main_shutdown_state = MAIN_SHUTDOWN_2_REQUESTED;
        }
    } else if (reset_pressed && gpio_get_reset_btn_fwrd()) {
        // Reset button is still pressed
        if (gpio_reset_count > RESET_SHORT_PRESS && gpio_reset_count < RESET_LONG_PRESS) {
            // Enter the shutdown pending state to begin LED dimming
            main_shutdown_state = MAIN_SHUTDOWN_PENDING;
        }
        else if (gpio_reset_count >= RESET_LONG_PRESS && gpio_reset_count < RESET_VERY_LONG_PRESS) {
            // Enter the shutdown reached state to blink LED
            main_shutdown_state = MAIN_SHUTDOWN_1_REACHED;
        }
        else if (gpio_reset_count >= RESET_VERY_LONG_PRESS) {
            // Enter the shutdown reached state to blink LED
            main_shutdown_state = MAIN_SHUTDOWN_2_REACHED;
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
  
    if (go_to_sleep) {
        go_to_sleep = false;
        main_shutdown_state = MAIN_SHUTDOWN_1_REQUESTED;
    }
    
    if (usb_state == USB_CONNECTED) {
      // configure pin as GPIO
      PIN_HID_LED_PORT->PCR[PIN_HID_LED_BIT] = PORT_PCR_MUX(1);
      PIN_MSC_LED_PORT->PCR[PIN_MSC_LED_BIT] = PORT_PCR_MUX(1);
      PIN_CDC_LED_PORT->PCR[PIN_CDC_LED_BIT] = PORT_PCR_MUX(1);
      power_led_max_duty_cycle = 100;
    }
    else if (usb_state == USB_DISCONNECTED) {
      // Disable pin
      PIN_HID_LED_PORT->PCR[PIN_HID_LED_BIT] = PORT_PCR_MUX(0);
      PIN_MSC_LED_PORT->PCR[PIN_MSC_LED_BIT] = PORT_PCR_MUX(0);
      PIN_CDC_LED_PORT->PCR[PIN_CDC_LED_BIT] = PORT_PCR_MUX(0);
      power_led_max_duty_cycle = POWER_LED_LOW_DUTY_CYCLE;
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
          if (shutdown_led_dc > 0) {
              shutdown_led_dc--;
          }
          break;
      case MAIN_SHUTDOWN_1_REACHED:
          // Blink the LED to indicate we are waiting for release
          if (shutdown_led_dc < 10) {
              shutdown_led_dc++;
          } else if (shutdown_led_dc == 10) {
              shutdown_led_dc = 100;
          } else if (shutdown_led_dc <= 90) {
              shutdown_led_dc = 0;
          } else if (shutdown_led_dc > 90) {
              shutdown_led_dc--;
          }
          break;
      case MAIN_SHUTDOWN_2_REACHED:
          // Blink the LED to indicate we are waiting for release
          if (shutdown_led_dc < 10) {
              shutdown_led_dc += 2;
          } else if (shutdown_led_dc == 10) {
              shutdown_led_dc = 100;
          } else if (shutdown_led_dc <= 90) {
              shutdown_led_dc = 0;
          } else if (shutdown_led_dc > 90) {
              shutdown_led_dc -= 2;
          }
          break;
      case MAIN_SHUTDOWN_1_REQUESTED:
          if (power_source == PWR_BATT_ONLY || usb_state == USB_DISCONNECTED) {
              interface_power_mode = kAPP_PowerModeVlps;
              main_powerdown_event();
          }
          main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          break;
      case MAIN_SHUTDOWN_2_REQUESTED:
          if (power_source == PWR_BATT_ONLY || usb_state == USB_DISCONNECTED) {
              interface_power_mode = kAPP_PowerModeVlls0;
              main_powerdown_event();
          }
          main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          break;
      case MAIN_SHUTDOWN_WAITING:
          // Set the PWM value back to max duty cycle
          shutdown_led_dc = power_led_max_duty_cycle;
          break;
      case MAIN_LED_BLINK:
          shutdown_led_dc = 0;
          
          if (blink_in_progress) {
            blink_in_progress--;
          } else {
            blink_in_progress = 10;
          }
          
          if (blink_in_progress == 0) { 
            main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          }
          break;
      default:
          break;
    }
    flexio_pwm_set_dutycycle(shutdown_led_dc);
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
