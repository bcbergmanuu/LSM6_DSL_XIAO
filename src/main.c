#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include "storage_nvs.h"
#include "lsm6dsl_reg.h"
#include "lsm6dsl_load.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include "battery.h"
#include "ble_load.h"

K_HEAP_DEFINE(BUFFER,512);

LOG_MODULE_REGISTER(MOTION_SENSOR, CONFIG_LOG_DEFAULT_LEVEL);

void battery_work_handler(struct k_work *work_item);

struct k_work battery_work;
K_WORK_DEFINE(battery_work, battery_work_handler);

void battery_work_handler(struct k_work *work_item)
{
	uint16_t battery_millivolt = 0;
	uint8_t battery_percentage = 0;

	battery_get_millivolt(&battery_millivolt);
	battery_get_percentage(&battery_percentage, battery_millivolt);

	LOG_INF("Battery at %d mV (capacity %d%%)", battery_millivolt, battery_percentage);
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	__ASSERT(device_is_ready(dev), "console not ready");
	uint32_t dtr = 0, baudrate = 0;

	if (usb_enable(NULL)) {
		LOG_ERR("USB-not-working");
	}
	
	LOG_INF("Wait for DTR");
	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}
	LOG_INF("DTR set");

	if(uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate)) {	
		LOG_WRN("Failed to get baudrate");
	} else {
		LOG_INF("Baudrate detected: %d", baudrate);
	}
	
	struct storage_module *storage_m = (struct storage_module*)k_malloc(sizeof(struct storage_module));

	if(init_storage(storage_m)) {
		LOG_ERR("storage not working");
	};

  	if(lsm6dsl_init(storage_m)) {
		LOG_ERR("could not initialize motion sensor");
	}

	if(battery_init()) {
		LOG_ERR("error battery init");
	}

	if(ble_load()) {
		LOG_ERR("could not start BLE");
	}

	while (true) {
		k_sleep(K_MSEC(5000));	

		k_work_submit(&battery_work);
	}
}