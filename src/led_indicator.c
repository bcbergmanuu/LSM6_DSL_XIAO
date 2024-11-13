
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static int led_init() {
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led, 1);
    return ret;
}

static void startblink(struct k_work *work) {
    for(int x= 0; x< 4; x++) {
        gpio_pin_toggle_dt(&led);
        k_sleep(K_MSEC(20));
    }
}

K_WORK_DEFINE(blinky, startblink);

void blink() {
    k_work_submit(&blinky);
}

SYS_INIT(led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);