#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <stdio.h>
#include <zephyr/sys/util.h>
#include "storage_nvs.h"
#include "lsm6dsl_load.h"
#include "ble_load.h"
#include "battery.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>

K_HEAP_DEFINE(BUFFER,512);

LOG_MODULE_REGISTER(MAIN, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{		
	k_sleep(K_FOREVER);							
}