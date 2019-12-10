/**
 * @file adc.c
 * @brief
 */

#include "adc.h"



#define ADC_BASE    ADC0


void adc_init(void)
{
    adc16_config_t adc16ConfigStruct;
    adc16ConfigStruct.referenceVoltageSource     = kADC16_ReferenceVoltageSourceVref;
    adc16ConfigStruct.clockSource                = kADC16_ClockSourceAsynchronousClock;
    adc16ConfigStruct.enableAsynchronousClock    = true;
    adc16ConfigStruct.clockDivider               = kADC16_ClockDivider1;
    adc16ConfigStruct.resolution                 = kADC16_ResolutionSE12Bit;
    adc16ConfigStruct.longSampleMode             = kADC16_LongSampleCycle24;
    adc16ConfigStruct.enableHighSpeed            = false;
    adc16ConfigStruct.enableLowPower             = false;
    adc16ConfigStruct.enableContinuousConversion = false;
    //ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(ADC_BASE, &adc16ConfigStruct);
    ADC16_DoAutoCalibration(ADC_BASE);
}

void adc_init_pins()
{
    PIN_VMON_USB_PORT->PCR[PIN_VMON_USB_BIT] = PORT_PCR_MUX(PIN_VMON_USB_ALT_MODE);
    PIN_VMON_BAT_PORT->PCR[PIN_VMON_BAT_BIT] = PORT_PCR_MUX(PIN_VMON_BAT_ALT_MODE);
}

uint32_t adc_read_channel(uint32_t channelGroup, uint32_t channelNumber, uint32_t channelMux)
{
    adc16_channel_config_t adc16ChannelConfigStruct;
    adc16ChannelConfigStruct.channelNumber = channelNumber;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    adc16ChannelConfigStruct.enableDifferentialConversion = false;

    ADC16_SetChannelMuxMode(ADC_BASE, (adc16_channel_mux_mode_t)channelMux);
    ADC16_SetChannelConfig(ADC_BASE, channelGroup, &adc16ChannelConfigStruct);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC_BASE, channelGroup)));
    return ADC16_GetChannelConversionValue(ADC_BASE, channelGroup);
}
