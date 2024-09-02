/*
 * Copyright 2024 Marcus Alexander Tjomsaas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "battery.h"
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

static void update_ble_handler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(ble_send_work, update_ble_handler);


static int16_t sample_buffer[ADC_TOTAL_SAMPLES];

static struct adc_sequence_options options = {
    .extra_samplings = ADC_TOTAL_SAMPLES - 1,
    .interval_us = ADC_SAMPLE_INTERVAL_US,
};

static struct adc_sequence sequence = {
    .options = &options,
    .channels = BIT(ADC_CHANNEL),
    .buffer = sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION,
};

static struct adc_channel_cfg channel_7_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = ADC_PORT
#endif
};

// MCU peripherals for reading battery voltage
static const struct device *gpio_battery_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *adc_battery_dev = DEVICE_DT_GET(DT_NODELABEL(adc));

// Charging interrupt

static K_MUTEX_DEFINE(battery_mut);

// Assuming LiPo battery.
// For better accuracy, use your battery's datasheet.
static BatteryState battery_states[BATTERY_STATES_COUNT] = {
    {4200, 100},
    {4160, 99},
    {4090, 91},
    {4030, 78},
    {3890, 63},
    {3830, 53},
    {3680, 36},
    {3660, 35},
    {3480, 14},
    {3420, 11},
    {3150, 1}, // 3240
    {0000, 0}  // Below safe level
};

int battery_get_millivolt(uint16_t *battery_millivolt)
{

    int ret = 0;

    // Voltage divider circuit (Should tune R1 in software if possible)
    const uint16_t R1 = 1037; // Originally 1M ohm, calibrated after measuring actual voltage values. Can happen due to resistor tolerances, temperature ect..
    const uint16_t R2 = 510;  // 510K ohm
    
    int adc_mv = 0;
    
    ret |= k_mutex_lock(&battery_mut, K_FOREVER);
    ret |= gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, 1);
    ret |= adc_read(adc_battery_dev, &sequence);
    uint16_t adc_vref = adc_ref_internal(adc_battery_dev);

    // Get average sample value.
    for (uint8_t sample = 0; sample < ADC_TOTAL_SAMPLES; sample++)
    {
        adc_mv += sample_buffer[sample]; // ADC value, not millivolt yet.
    }
    adc_mv /= ADC_TOTAL_SAMPLES;

    // Convert ADC value to millivolts
    ret |= adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &adc_mv);

    // Calculate battery voltage.
    *battery_millivolt = adc_mv * ((R1 + R2) / R2);

    ret |= gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, 0);

    k_mutex_unlock(&battery_mut);

    LOG_DBG("%d mV", *battery_millivolt);
    return ret;
}

int battery_get_percentage(uint8_t *battery_percentage, uint16_t battery_millivolt)
{

    // Ensure voltage is within bounds
    if (battery_millivolt > battery_states[0].voltage)
        *battery_percentage = 100;
    if (battery_millivolt < battery_states[BATTERY_STATES_COUNT - 1].voltage)
        *battery_percentage = 0;

    for (uint16_t i = 0; i < BATTERY_STATES_COUNT - 1; i++)
    {
        // Find the two points battery_millivolt is between
        if (battery_states[i].voltage >= battery_millivolt && battery_millivolt >= battery_states[i + 1].voltage)
        {
            // Linear interpolation
            *battery_percentage = battery_states[i].percentage +
                                  ((float)(battery_millivolt - battery_states[i].voltage) *
                                   ((float)(battery_states[i + 1].percentage - battery_states[i].percentage) /
                                    (float)(battery_states[i + 1].voltage - battery_states[i].voltage)));

            LOG_DBG("%d %%", *battery_percentage);
            return 0;
        }
    }
    return -ESPIPE;
}


static void update_ble_handler(struct k_work *work) {
    
    uint8_t percentage = 0;
    uint16_t millivolt = 0;
    battery_get_millivolt(&millivolt);

    battery_get_percentage(&percentage, millivolt);
    bt_bas_set_battery_level(percentage);
    LOG_INF("update_ble mV %d, percentage %d", millivolt, percentage);
    k_work_schedule(&ble_send_work, K_MINUTES(BAS_UPDATE_FREQUENCY));
}

int battery_init()
{
    int ret = 0;

    ret |= adc_channel_setup(adc_battery_dev, &channel_7_cfg);        
    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    ret |= gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, 1); // FAST charge 100mA
    
    LOG_INF("Initialized");
    
    k_work_schedule(&ble_send_work, K_MINUTES(BAS_UPDATE_FREQUENCY));

    return ret;
}

SYS_INIT(battery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);