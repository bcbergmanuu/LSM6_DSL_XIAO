
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>


#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(LED0_NODE, gpios),
    GPIO_DT_SPEC_GET(LED1_NODE, gpios),    
};

static void switchof(struct k_work *work);

struct k_work_delayable dw_switchoffs [2];

static int led_init() {
    int ret = 0;
    for(int x = 0; x< 2; x++) {
        ret |= gpio_pin_configure_dt(&leds[x], GPIO_OUTPUT_ACTIVE);
        ret |= gpio_pin_set_dt(&leds[x], 0);
        k_work_init_delayable(&dw_switchoffs[x], switchof);  
    }    
    
    return ret;
}

//K_WORK_DELAYABLE_DEFINE(dw_switchof, switchof);

static void switchof(struct k_work *work) {    
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    int index = dwork - dw_switchoffs;    
    gpio_pin_set_dt(&leds[index], 0);    
}

void blink(uint8_t color) {    
    gpio_pin_set_dt(&leds[color], 1);
    k_work_schedule(&dw_switchoffs[color], K_MSEC(500));
}

SYS_INIT(led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);