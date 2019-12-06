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

#ifdef DRAG_N_DROP_SUPPORT
#include "flash_intf.h"
#endif

#define M0_RESERVED_VECT_OFFSET     (4 * 4)
#define M0_RESERVED_VECT_SIZE       (3 * 4) // Size for mem fault, bus fault and usage fault vectors

const char * const board_id_mb_2_0 = "9903";
const uint16_t board_id_hex = 0x9903;

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
    MAIN_SHUTDOWN_CANCEL
} main_shutdown_state_t;

extern void main_powerdown_event(void);

// shutdown state
static main_shutdown_state_t main_shutdown_state = MAIN_SHUTDOWN_WAITING;
static uint8_t shutdown_led_dc = 100;
static app_power_mode_t interface_power_mode;
static bool battery_powered;

// Called in main_task() to init before USB and files are configured
static void prerun_board_config(void)
{
    target_device = target_device_nrf52_64;
    target_device.rt_board_id = board_id_mb_2_0;

    // init power monitoring
    pwr_mon_init();

    battery_powered = pwr_mon_battery_powered();

    flexio_pwm_init();
    flexio_pwm_init_pins();
    
    if (battery_powered == true){
        // Turn on the red LED. TODO. Lower duty cycle to save power?
        flexio_pwm_set_dutycycle(100);
    } else {
        // Turn on the red LED when powered by USB or EC
        flexio_pwm_set_dutycycle(100);       
    }

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
        }
    } else if (reset_pressed && !gpio_get_reset_btn_fwrd()) {
        // Reset button released
        reset_pressed = 0;

        if (gpio_reset_count <= RESET_SHORT_PRESS) {
            target_set_state(RESET_RUN);
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
    if (go_to_sleep) {
        go_to_sleep = false;
        main_shutdown_state = MAIN_SHUTDOWN_1_REQUESTED;
    }
    
    if (usb_state == USB_CONNECTED) {
      // configure pin as GPIO
      PIN_HID_LED_PORT->PCR[PIN_HID_LED_BIT] = PORT_PCR_MUX(1);
      PIN_MSC_LED_PORT->PCR[PIN_MSC_LED_BIT] = PORT_PCR_MUX(1);
      PIN_CDC_LED_PORT->PCR[PIN_CDC_LED_BIT] = PORT_PCR_MUX(1);
    }
    else if (usb_state == USB_DISCONNECTED) {
      // Disable pin
      PIN_HID_LED_PORT->PCR[PIN_HID_LED_BIT] = PORT_PCR_MUX(0);
      PIN_MSC_LED_PORT->PCR[PIN_MSC_LED_BIT] = PORT_PCR_MUX(0);
      PIN_CDC_LED_PORT->PCR[PIN_CDC_LED_BIT] = PORT_PCR_MUX(0);
    }

    switch (main_shutdown_state) {
      case MAIN_SHUTDOWN_CANCEL:
          main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          // Set the PWM value back to 100%
          shutdown_led_dc = 100;
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
          if (battery_powered == true || usb_state == USB_DISCONNECTED) {
              interface_power_mode = kAPP_PowerModeVlps;
              main_powerdown_event();
          }
          main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          break;
      case MAIN_SHUTDOWN_2_REQUESTED:
          if (battery_powered == true || usb_state == USB_DISCONNECTED) {
              interface_power_mode = kAPP_PowerModeVlls0;
              main_powerdown_event();
          }
          main_shutdown_state = MAIN_SHUTDOWN_WAITING;
          break;
      case MAIN_SHUTDOWN_WAITING:
          // Set the PWM value back to 100%
          shutdown_led_dc = 100;
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
    .infoVersion = 0x0,
    .family_id = kNordic_Nrf52_FamilyID,
    .daplink_url_name =       "MICROBITHTM",
    .daplink_drive_name =       "MICROBIT",
    .daplink_target_url = "https://microbit.org/device/?id=@B&v=@V",
    .prerun_board_config = prerun_board_config,
    .target_cfg = &target_device,
};
