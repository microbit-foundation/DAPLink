/**
 * @file    IO_Config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
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

// Override all defines if IO_CONFIG_OVERRIDE is defined
#ifdef IO_CONFIG_OVERRIDE
#include "IO_Config_Override.h"
#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__
#endif
#endif

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "MKL27Z4.h"
#include "compiler.h"
#include "daplink.h"

// This GPIO configuration is only valid for the KL27 HIC
COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_KL27Z);


// Debug Port I/O Pins

// SWCLK Pin                    PTC5(C5)
#define PIN_SWCLK_PORT          PORTC
#define PIN_SWCLK_GPIO          PTC
#define PIN_SWCLK_BIT           (5)
#define PIN_SWCLK               (1<<PIN_SWCLK_BIT)

// SWDIO Pin                    PTC6(C6)
#define PIN_SWDIO_PORT          PORTC
#define PIN_SWDIO_GPIO          PTC
#define PIN_SWDIO_BIT           (6)
#define PIN_SWDIO               (1<<PIN_SWDIO_BIT)

// nRESET Pin                   PTA20(A20)
#define PIN_nRESET_PORT         PORTA
#define PIN_nRESET_GPIO         PTA
#define PIN_nRESET_BIT          (20)
#define PIN_nRESET              (1<<PIN_nRESET_BIT)

// Debug Unit LEDs

// HID_LED PTB0
#define PIN_HID_LED_PORT        PORTB
#define PIN_HID_LED_GPIO        PTB
#define PIN_HID_LED_BIT         (0)
#define PIN_HID_LED             (1<<PIN_HID_LED_BIT)

// MSC_LED PTB0
#define PIN_MSC_LED_PORT        PORTB
#define PIN_MSC_LED_GPIO        PTB
#define PIN_MSC_LED_BIT         (0)
#define PIN_MSC_LED             (1<<PIN_MSC_LED_BIT)

// CDC_LED PTB0
#define PIN_CDC_LED_PORT        PORTB
#define PIN_CDC_LED_GPIO        PTB
#define PIN_CDC_LED_BIT         (0)
#define PIN_CDC_LED             (1<<PIN_CDC_LED_BIT)

// Volatge monitor pins

// USB Voltage monitor PTD5
#define PIN_VMON_USB_PORT      PORTD
#define PIN_VMON_USB_GPIO      PTD
#define PIN_VMON_USB_BIT       (5)
#define PIN_VMON_USB           (1<<PIN_VMON_USB_BIT)
#define PIN_VMON_USB_ALT_MODE  (0)
#define PIN_VMON_USB_ADC_CH    (6)
#define PIN_VMON_USB_ADC_MUX   (1)

// Battery Voltage monitor PTB1
#define PIN_VMON_BAT_PORT      PORTB
#define PIN_VMON_BAT_GPIO      PTB
#define PIN_VMON_BAT_BIT       (1)
#define PIN_VMON_BAT           (1<<PIN_VMON_BAT_BIT)
#define PIN_VMON_BAT_ALT_MODE  (0)
#define PIN_VMON_BAT_ADC_CH    (9)
#define PIN_VMON_BAT_ADC_MUX   (0)

// Enable Battery Voltage monitor PTC3
#define PIN_RUN_VMON_BAT_PORT  PORTC
#define PIN_RUN_VMON_BAT_GPIO  PTC
#define PIN_RUN_VMON_BAT_BIT   (3)
#define PIN_RUN_VMON_BAT       (1<<PIN_RUN_VMON_BAT_BIT)

// Reset pins

// SW RESET BUTTON PTD4
#define PIN_SW_RESET_PORT       PORTD
#define PIN_SW_RESET_GPIO       PTD
#define PIN_SW_RESET_BIT        (4)
#define PIN_SW_RESET            (1<<PIN_SW_RESET_BIT)
#define SW_RESET_PRESSED        (0)
#define SW_RESET_NOT_PRESSED    (1)
#define PIN_SW_RESET_LLWU_PIN   (14)
#define PIN_SW_RESET_LLWU_WAKEUP_TYPE   kLLWU_ExternalPinRisingEdge
#define PIN_SW_RESET_PORT_WAKEUP_TYPE   kPORT_InterruptRisingEdge

// KILLME PTC7
#define PIN_KILLME_PORT         PORTC
#define PIN_KILLME_GPIO         PTC
#define PIN_KILLME_BIT          (7)
#define PIN_KILLME              (1<<PIN_KILLME_BIT)

// LOADDUMP PTC4
#define PIN_LOADDUMP_PORT       PORTC
#define PIN_LOADDUMP_GPIO       PTC
#define PIN_LOADDUMP_BIT        (4)
#define PIN_LOADDUMP            (1<<PIN_LOADDUMP_BIT)

// Power LEDs

// Orange LED PTB0, configured as HID/MSC/CDC LEDs

// Red Spare LED PTA2
#define PIN_RED_LED_PORT         PORTA
#define PIN_RED_LED_GPIO         PTA
#define PIN_RED_LED_BIT          (2)
#define PIN_RED_LED              (1<<PIN_RED_LED_BIT)
#define PIN_RED_CH_NUMBER        1

#define RED_LED_PWM_FREQ_HZ      2000

// PWM settings for the board
#define BOARD_TPM_BASEADDR  TPM2
#define BOARD_TPM_CHANNEL   1
#define TPM_LED_ON_LEVEL    kTPM_HighTrue

// define the long and short reset button presses
#define RESET_SHORT_PRESS          8    // x 30ms debounce time =  240ms
#define RESET_LONG_PRESS           130  // x 30ms debounce time = 3900ms
#define RESET_2SEC_PRESS           66   // x 30ms debounce time = 1980ms
#define RESET_VERY_LONG_PRESS      (RESET_LONG_PRESS + RESET_2SEC_PRESS * 1)
#define RESET_MAX_LENGTH_PRESS     RESET_VERY_LONG_PRESS

// Connected LED                Not available

// Target Running LED           Not available

// UART
#define UART_PORT               PORTD
#define UART_NUM                (0)
// RX PTD6
#define PIN_UART_RX_GPIO        PTD
#define PIN_UART_RX_BIT         (6)
#define PIN_UART_RX             (1<<PIN_UART_RX_BIT)
#define PIN_UART_RX_MUX_ALT     (3)
// TX PTD7
#define PIN_UART_TX_GPIO        PTD
#define PIN_UART_TX_BIT         (7)
#define PIN_UART_TX             (1<<PIN_UART_TX_BIT)
#define PIN_UART_TX_MUX_ALT     (3)

#define UART                    LPUART0
#define UART_RX_TX_IRQn         LPUART0_IRQn
#define UART_RX_TX_IRQHandler   LPUART0_IRQHandler

#endif
