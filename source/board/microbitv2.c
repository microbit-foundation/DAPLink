/**
 * @file    microbit.c
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
#include "pwm.h"
#include "gpio.h"

const char * const board_id_mb_2_0 = "9902";

extern target_cfg_t target_device_nrf52;

typedef enum main_shutdown_state {
    MAIN_SHUTDOWN_WAITING = 0,
    MAIN_SHUTDOWN_PENDING,
    MAIN_SHUTDOWN_REACHED,
    MAIN_SHUTDOWN_REQUESTED,
    MAIN_SHUTDOWN_CANCEL
} main_shutdown_state_t;

// shutdown state
main_shutdown_state_t main_shutdown_state = MAIN_SHUTDOWN_WAITING;
uint8_t shutdown_led_dc = 100;

// Called in main_task() to init before USB and files are configured
static void prerun_board_config(void) {
    
    
    target_device = target_device_nrf52;
    target_device.rt_board_id = board_id_mb_2_0;
    
    pwm_init();
    pwm_init_pins();
    
    // Turn on the red LED
    pwm_set_dutycycle(100);
}

void board_30ms_hook()
{
                // need to define battery powered and set it someplace
            if (1) {
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
                    case MAIN_SHUTDOWN_REACHED:
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
                    case MAIN_SHUTDOWN_REQUESTED:
                        // Drive LOAD_DUMP and KILLME
                        gpio_set_loaddump(GPIO_ON);
                        gpio_set_killme(GPIO_ON);
                        main_shutdown_state = MAIN_SHUTDOWN_WAITING;
                        break;
                    case MAIN_SHUTDOWN_WAITING:
                    default:
                        break;
                }
                pwm_set_dutycycle(shutdown_led_dc);
            }
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
