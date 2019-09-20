/**
 * @file    tpm.c
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

#include "fsl_tpm.h"
#include "pwm.h"

#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_McgIrc48MClk)

#define PIN_RED_CH_NUMBER        1

#define RED_LED_PWM_FREQ_HZ      2000

// PWM settings for the board
#define BOARD_TPM_BASEADDR  TPM2
#define BOARD_TPM_CHANNEL   1
#define TPM_LED_ON_LEVEL    kTPM_HighTrue

void pwm_set_mux(uint8_t mux);


void pwm_init(void)
{
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam;

    // setup the port output
    //PIN_RED_LED_PORT->PCR[PIN_RED_LED_BIT] = PORT_PCR_MUX(3);

    // setup the TPM clock
    CLOCK_SetTpmClock(1U);

    // Configure tpm params with frequency 24kHZ
    tpmParam.chnlNumber = (tpm_chnl_t)BOARD_TPM_CHANNEL;
    tpmParam.level = TPM_LED_ON_LEVEL;
    tpmParam.dutyCyclePercent = 100;

    // get the default config
    TPM_GetDefaultConfig(&tpmInfo);

    // initialize, setup PWM, and start TPM module
    TPM_Init(TPM2, &tpmInfo);
    TPM_SetupPwm(TPM2, &tpmParam, 1U, kTPM_CenterAlignedPwm, RED_LED_PWM_FREQ_HZ, TPM_SOURCE_CLOCK);
    TPM_StartTimer(BOARD_TPM_BASEADDR, kTPM_SystemClock);
}

void pwm_init_pins(void) {
    pwm_set_mux(3);
}

void pwm_deinit_pins(void) {
    pwm_set_mux(1);
}

void pwm_set_dutycycle(uint8_t updatedDutycycle)
{
    /* Disable channel output before updating the dutycycle */
    TPM_UpdateChnlEdgeLevelSelect(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL, 0U);

    /* Update PWM duty cycle */
    TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL, kTPM_CenterAlignedPwm,
                           updatedDutycycle);

    /* Start channel output with updated dutycycle */
    TPM_UpdateChnlEdgeLevelSelect(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL, TPM_LED_ON_LEVEL);
}

void pwm_set_mux(uint8_t mux)
{
    PIN_RED_LED_PORT->PCR[PIN_RED_LED_BIT] = PORT_PCR_MUX(mux);
}
