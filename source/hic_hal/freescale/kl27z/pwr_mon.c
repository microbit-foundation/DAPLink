/**
 * @file power_monitor.c
 * @brief
 */

#include "pwr_mon.h"
#include "adc.h"
#include "gpio.h"
#include "fsl_pmc.h"


#define ADC_VBG_CHANNEL     27U
#define ADC_VBG_MUX         (kADC16_ChannelMuxA)
#define ADC_VREFH_CHANNEL   29U
#define ADC_VREFH_MUX       (kADC16_ChannelMuxA)


void pwr_mon_bandgap_init(void);
uint32_t pwr_mon_read_vbg(uint32_t channelGroup);
uint32_t pwr_mon_adc_to_mv(uint32_t raw_adc);


void pwr_mon_init(void)
{
    adc_init();
    adc_init_pins();
}

bool pwr_mon_battery_powered(void) {
    // Detect if device is battery powered
    gpio_set_run_vbat_sense(GPIO_ON);
    // Add a ~3ms delay to allow the 100nF capacitors to charge to about 3*RC.
    // 3 clock cycles per loop at -O2 ARMCC optimization
    for (uint32_t count = 48000; count > 0UL; count--); 
    volatile uint32_t bat_adc = adc_read_channel(0, PIN_VMON_BAT_ADC_CH, PIN_VMON_BAT_ADC_MUX);
    volatile uint32_t usb_adc = adc_read_channel(0, PIN_VMON_USB_ADC_CH, PIN_VMON_USB_ADC_MUX);
    gpio_set_run_vbat_sense(GPIO_OFF);

    // With USB power any battery voltage should be at least 35% lower than USB, but with the battery only they will be
    // around the same. So, add some marging (12% is easy to calculate) for the comparison
    uint32_t bat_adc_marging = bat_adc >> 3;
    return (bat_adc + bat_adc_marging) > usb_adc;
}

uint32_t pwr_mon_vcc_mv(void) {
    // ADC read the Vref (same as Vcc) and the Vbg 1V internal reference
    uint32_t vref_high = adc_read_channel(0, ADC_VREFH_CHANNEL, ADC_VREFH_MUX);
    uint32_t ref_volt = pwr_mon_read_vbg(0);

    // Reference voltage is 1V, scale to millivolts
    return vref_high * 1000 / ref_volt;
}


void pwr_mon_bandgap_init(void) {
    pmc_bandgap_buffer_config_t pmcBandgapConfig;
    pmcBandgapConfig.enable = true;
    pmcBandgapConfig.enableInLowPowerMode = true;
    PMC_ConfigureBandgapBuffer(PMC, &pmcBandgapConfig);
}

uint32_t pwr_mon_read_vbg(uint32_t channelGroup) {
    // Ensure the Vbg is enabled from first use only
    static bool bandgap_enabled = false;
    if (!bandgap_enabled) {
        pwr_mon_bandgap_init();
        bandgap_enabled = true;
    }
    return adc_read_channel(channelGroup, ADC_VBG_CHANNEL, ADC_VBG_MUX);
}

uint32_t pwr_mon_adc_to_mv(uint32_t raw_adc)
{
    // ADC read Vbg, a 1V internal reference
    uint32_t ref_volt_read = pwr_mon_read_vbg(0);

    // Scale ADC read out to millivolts
    return (raw_adc * 1000) / ref_volt_read;
}
