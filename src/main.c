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

LOG_MODULE_REGISTER(MOTION_SENSOR, CONFIG_LOG_DEFAULT_LEVEL);


int main(void)
{		
	struct storage_module *storage_m = (struct storage_module*)k_malloc(sizeof(struct storage_module));

	k_sleep(K_MSEC(5000));
    
	if(init_storage(storage_m)) {
		LOG_ERR("storage not working");
	};	
    k_sleep(K_MSEC(5000));

    if(battery_init()) {
		LOG_ERR("could not initialize battery");
	}
    k_sleep(K_MSEC(5000));

    if(ble_load()) {
		LOG_ERR("could not start BLE");
	}
    k_sleep(K_SECONDS(5));

  	if(lsm6dsl_init(storage_m)) {
		LOG_ERR("could not initialize motion sensor");
	}     

	k_sleep(K_FOREVER);							
}