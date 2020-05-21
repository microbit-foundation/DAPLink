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

// SWCLK Pin                    PTC6
#define PIN_SWCLK_PORT          PORTC
#define PIN_SWCLK_GPIO          PTC
#define PIN_SWCLK_BIT           (6)
#define PIN_SWCLK               (1<<PIN_SWCLK_BIT)

// SWDIO Pin                    PTC5
#define PIN_SWDIO_PORT          PORTC
#define PIN_SWDIO_GPIO          PTC
#define PIN_SWDIO_BIT           (5)
#define PIN_SWDIO               (1<<PIN_SWDIO_BIT)

// nRESET Pin                   PTA20
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
#define PIN_HID_LED_MUX_ALT     (1)

// MSC_LED PTB0
#define PIN_MSC_LED_PORT        PORTB
#define PIN_MSC_LED_GPIO        PTB
#define PIN_MSC_LED_BIT         (0)
#define PIN_MSC_LED             (1<<PIN_MSC_LED_BIT)
#define PIN_MSC_LED_MUX_ALT     (1)

// CDC_LED PTB0
#define PIN_CDC_LED_PORT        PORTB
#define PIN_CDC_LED_GPIO        PTB
#define PIN_CDC_LED_BIT         (0)
#define PIN_CDC_LED             (1<<PIN_CDC_LED_BIT)
#define PIN_CDC_LED_MUX_ALT     (1)

// Reset pins

// SW RESET BUTTON PTD4
#define PIN_SW_RESET_PORT       PORTD
#define PIN_SW_RESET_GPIO       PTD
#define PIN_SW_RESET_BIT        (4)
#define PIN_SW_RESET            (1<<PIN_SW_RESET_BIT)
#define SW_RESET_PRESSED        (0)
#define SW_RESET_NOT_PRESSED    (1)

// UART
#define UART_PORT               PORTA
#define UART_NUM                (1)
// RX PTA18
#define PIN_UART_RX_GPIO        PTA
#define PIN_UART_RX_BIT         (18)
#define PIN_UART_RX             (1<<PIN_UART_RX_BIT)
#define PIN_UART_RX_MUX_ALT     (3)
// TX PTA19
#define PIN_UART_TX_GPIO        PTA
#define PIN_UART_TX_BIT         (19)
#define PIN_UART_TX             (1<<PIN_UART_TX_BIT)
#define PIN_UART_TX_MUX_ALT     (3)

#define UART                    LPUART1
#define UART_RX_TX_IRQn         LPUART1_IRQn
#define UART_RX_TX_IRQHandler   LPUART1_IRQHandler

#endif
