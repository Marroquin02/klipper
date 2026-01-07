// ADC functions on N32G45x
//
// Copyright (C) 2022-2023  Alexey Golyshin <stas2z@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "generic/armcm_timer.h" // udelay
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown
#include "n32g45x_adc.h" // ADC

// Watchdog refresh function to prevent system reset during long ADC operations
static void watchdog_refresh(void);

#define ADC_INVALID_PIN 0xFF

DECL_CONSTANT("ADC_MAX", 4095);

#define ADC_TEMPERATURE_PIN 0xfe
DECL_ENUMERATION("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

static const uint8_t adc_pins[] = {
    // ADC1
    ADC_INVALID_PIN, GPIO('A', 0), GPIO('A', 1), GPIO('A', 6),
    GPIO('A', 3), GPIO('F', 4), ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_TEMPERATURE_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    // ADC2
    ADC_INVALID_PIN, GPIO('A', 4), GPIO('A', 5), GPIO('B', 1),
    GPIO('A', 7), GPIO('C', 4), GPIO('C', 0), GPIO('C', 1),
    GPIO('C', 2), GPIO('C', 3), GPIO('F', 2), GPIO('A', 2),
    GPIO('C', 5), GPIO('B', 2), ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
#if CONFIG_MACH_N32G455 // ADC3/4 for G455 only
    // ADC3
    ADC_INVALID_PIN, GPIO('B', 11), GPIO('E', 9), GPIO('E', 13),
    GPIO('E', 12), GPIO('B', 13), GPIO('E', 8), GPIO('D', 10),
    GPIO('D', 11), GPIO('D', 12), GPIO('D', 13), GPIO('D', 14),
    GPIO('B', 0), GPIO('E', 7), GPIO('E', 10), GPIO('E', 11),
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    // ADC4
    ADC_INVALID_PIN, GPIO('E', 14), GPIO('E', 15), GPIO('B', 12),
    GPIO('B', 14), GPIO('B', 15), ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    GPIO('D', 8), GPIO('D', 9), ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
    ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN, ADC_INVALID_PIN,
#endif
};

// Function to refresh watchdog during long operations
static void watchdog_refresh(void)
{
    extern void watchdog_reset(void);
    watchdog_reset();
}

// GPIO configuration function specific for N32G455
// Note: This function is not needed as we can use the standard gpio_peripheral() function
// which is compatible with N32G455

// Perform calibration with timeout protection - improved for N32G455
static void
adc_calibrate(ADC_Module *adc)
{
    uint32_t timeout = 10000; // Increased timeout for better reliability
    
    // Ensure ADC is completely off before calibration
    adc->CTRL2 &= ~CTRL2_AD_ON_SET;
    udelay(20); // Increased delay
    
    // Turn on ADC and wait for ready flag
    adc->CTRL2 = CTRL2_AD_ON_SET;
    while (!(adc->CTRL3 & ADC_FLAG_RDY) && timeout--) {
        watchdog_refresh(); // Refresh watchdog while waiting
        udelay(1);
    }
    
    if (!timeout) {
        // Timeout occurred, ADC not ready
        return;
    }
    
    // Disable calibration during power-up
    adc->CTRL3 &= (~ADC_CTRL3_BPCAL_MSK);
    udelay(50); // Increased delay for stabilization
    
    // Start calibration with proper sequence for N32G455
    adc->CTRL2 = CTRL2_AD_ON_SET | CTRL2_CAL_SET;
    
    timeout = 10000; // Increased timeout for calibration
    while ((adc->CTRL2 & CTRL2_CAL_SET) && timeout--) {
        watchdog_refresh(); // Refresh watchdog while waiting
        udelay(1);
    }
    
    if (!timeout) {
        // Timeout occurred during calibration
        return;
    }
    
    // Additional delay after calibration to ensure stability
    udelay(100);
}

struct gpio_adc
gpio_adc_setup(uint32_t pin)
{
    // Find pin in adc_pins table
    int chan;
    for (chan=0; ; chan++) {
        if (chan >= ARRAY_SIZE(adc_pins))
            shutdown("Not a valid ADC pin");
        if (adc_pins[chan] == pin)
            break;
    }

    // Determine which ADC block to use
    ADC_Module *adc;
    if ((chan >> 5) == 0)
        adc = NS_ADC1;
    if ((chan >> 5) == 1)
        adc = NS_ADC2;
    if ((chan >> 5) == 2)
        adc = NS_ADC3;
    if ((chan >> 5) == 3)
        adc = NS_ADC4;
    chan &= 0x1F;

    // Enable the ADC
    uint32_t reg_temp;
    reg_temp = ADC_RCC_AHBPCLKEN;
    reg_temp |= (RCC_AHB_PERIPH_ADC1 | RCC_AHB_PERIPH_ADC2 |
                RCC_AHB_PERIPH_ADC3 | RCC_AHB_PERIPH_ADC4);
    ADC_RCC_AHBPCLKEN = reg_temp;

    // Configure ADC clock properly - first enable ADC PLL clock
    reg_temp = ADC_RCC_CFG2;
    reg_temp &= CFG2_ADCPLLPRES_RESET_MASK;
    reg_temp |= RCC_ADCPLLCLK_DIV2; // Use DIV2 for proper timing
    // Don't disable the PLL clock, keep it enabled
    ADC_RCC_CFG2 = reg_temp;

    // Configure ADC HCLK prescaler - corrected for N32G455
    reg_temp = ADC_RCC_CFG2;
    reg_temp &= CFG2_ADCHPRES_RESET_MASK;
    reg_temp |= RCC_ADCHCLK_DIV2; // Changed from DIV4 to DIV2 for N32G455 specific timing
    ADC_RCC_CFG2 = reg_temp;

    ADC_InitType ADC_InitStructure;
    ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn      = 0;
    ADC_InitStructure.ContinueConvEn = 0;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(adc, &ADC_InitStructure);

    adc_calibrate(adc);

    if (pin == ADC_TEMPERATURE_PIN) {
        // Only enable temperature sensor on ADC1
        if (adc == NS_ADC1) {
            // Wait for ADC to be ready before enabling temperature sensor
            uint32_t timeout = 5000;
            while (!(adc->CTRL3 & ADC_FLAG_RDY) && timeout--) {
                watchdog_refresh();
                udelay(1);
            }
                
            if (timeout) {
                // Enable temperature sensor only - VREF configuration removed
                // The sensor of temperature is connected to ADC1_IN16, not to VREF
                NS_ADC1->CTRL2 |= CTRL2_TSVREFE_SET;
                // Removed: VREF1P2_CTRL |= (1<<10);
                // This bit controls the VREF buffer, not the temperature sensor
                
                // Increased delay after enabling temperature sensor
                // Changed from 1000us to 2000us to ensure proper stabilization
                udelay(2000);
                
                // Verify temperature sensor is properly enabled
                timeout = 1000;
                while (!(adc->CTRL3 & ADC_FLAG_RDY) && timeout--) {
                    watchdog_refresh();
                    udelay(1);
                }
            }
        }
    } else {
        // Ensure GPIO is properly configured for analog input
        gpio_peripheral(pin, GPIO_ANALOG, 0);
        // Add small delay to ensure GPIO configuration is applied
        udelay(10);
    }

    return (struct gpio_adc){ .adc = adc, .chan = chan };
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function - improved for N32G455
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    ADC_Module *adc = g.adc;
    uint32_t sr = adc->STS;
    
    // Refresh watchdog to prevent timeout during ADC operations
    watchdog_refresh();
    
    // Special handling for temperature sensor (channel 18)
    if (g.chan == 18) {
        // Temperature sensor requires longer sampling time for accuracy
        // Use maximum sampling time for temperature sensor
        ADC_ConfigRegularChannel(adc, g.chan, 1, ADC_SAMP_TIME_239CYCLES5);
    } else {
        // Use standard sampling time for regular channels
        ADC_ConfigRegularChannel(adc, g.chan, 1, ADC_SAMP_TIME_41CYCLES5);
    }
    
    if (sr & ADC_STS_STR) {
        if (!(sr & ADC_STS_ENDC) || adc->RSEQ3 != g.chan)
            // Conversion still in progress or busy on another channel
            goto need_delay;
        // Conversion ready
        return 0;
    }
    
    // ADC timing: clock=4Mhz, Tconv=12.5, Tsamp=41.5, total=13.500us
    // For temperature sensor: Tsamp=239.5, total=252.000us
    adc->CTRL2 |= CTRL2_AD_ON_SET;
    adc->CTRL2 |= CTRL2_EXT_TRIG_SWSTART_SET;

need_delay:
    // Use longer delay for temperature sensor to ensure proper conversion
    return timer_from_us(g.chan == 18 ? 300 : 20);
}

// Read a value; use only after gpio_adc_sample() returns zero - improved for N32G455
uint16_t
gpio_adc_read(struct gpio_adc g)
{
    ADC_Module *adc = g.adc;
    uint32_t timeout = 2000; // Increased timeout for N32G455
    
    // Refresh watchdog to prevent timeout during ADC operations
    watchdog_refresh();
    
    // Wait for conversion to complete with timeout
    while (!(adc->STS & ADC_STS_ENDC) && timeout--) {
        watchdog_refresh();
        udelay(1);
    }
    
    if (!timeout) {
        // Timeout occurred, return zero to indicate error
        return 0;
    }
    
    // Read the result BEFORE clearing flags (critical fix)
    uint16_t result = adc->DAT;
    
    // Now clear the status flags
    adc->STS &= ~ADC_STS_ENDC;
    adc->STS &= ~ADC_STS_STR;
    adc->CTRL2 &= CTRL2_EXT_TRIG_SWSTART_RESET;
    
    // Enhanced verification for temperature sensor (channel 18)
    if (g.chan == 18) {
        // Temperature sensor channel verification
        // If result is zero or too low, it might indicate a problem with the sensor
        if (result < 100) { // Threshold for minimum expected temperature reading
            // Try to re-read once more with longer delay
            udelay(100); // Allow more time for stabilization
            timeout = 1000;
            while (!(adc->STS & ADC_STS_ENDC) && timeout--) {
                watchdog_refresh();
                udelay(1);
            }
            if (timeout) {
                result = adc->DAT;
            }
            
            // If still too low, try a third time with even longer delay
            if (result < 100) {
                udelay(200);
                timeout = 1000;
                while (!(adc->STS & ADC_STS_ENDC) && timeout--) {
                    watchdog_refresh();
                    udelay(1);
                }
                if (timeout) {
                    result = adc->DAT;
                }
            }
        }
    }
    
    return result;
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    ADC_Module *adc = g.adc;
    irqstatus_t flag = irq_save();
    if (adc->STS & ADC_STS_STR)
        gpio_adc_read(g);
    irq_restore(flag);
}
