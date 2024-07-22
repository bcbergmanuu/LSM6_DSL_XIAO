#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include "lsm6dsl_reg.h"
#include <string.h>
#include "lsm6dsl_load.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(MOTION_SENSOR, CONFIG_LOG_DEFAULT_LEVEL);

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

  	lsm6dsl_init();
	while (true) {
		k_sleep(K_MSEC(10000));				
	}
}