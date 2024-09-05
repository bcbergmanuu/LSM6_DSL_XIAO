/** @file
 *  @brief CTS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/posix/time.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/logging/log.h>
#include "cts.h"

//current time
static uint8_t ct[sizeof(time_t)];

LOG_MODULE_REGISTER(TIME_MODULE, CONFIG_LOG_DEFAULT_LEVEL);

void ble_update_time(struct k_work *work);

void timerElapsed(struct k_timer *timer_id);
K_WORK_DEFINE(notifyCTS, ble_update_time);

K_TIMER_DEFINE(update_time_timer, timerElapsed, NULL);

void timerElapsed(struct k_timer *timer_id) {
	k_work_submit(&notifyCTS);
}

static void ct_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{

	if(value) {
		k_timer_start(&update_time_timer, K_SECONDS(1), K_SECONDS(1));

	} else {
		k_timer_stop(&update_time_timer);
	}
}

static ssize_t read_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(ct));
}

int settime(time_t current_time) {
	int ret = 0;
	struct timespec ts = {0};
	ts.tv_sec = current_time;

    ret |= clock_settime(CLOCK_REALTIME, &ts);
	
	return ret;
}

static ssize_t write_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(ct)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
		
	for(uint16_t i = 0; i<len; i++){
		LOG_INF("received %i", value[i]);
	}
	time_t timeconstruct = 0;
	memcpy(&timeconstruct, value, len);
	
	settime(timeconstruct);
	print_current_time();

	return len;
}

int print_current_time() {
	
    struct timespec ts = {0};      
	
	clock_gettime(CLOCK_REALTIME, &ts);
	struct tm *current_time = gmtime(&ts.tv_sec);

	LOG_INF("Current time: %04d-%02d-%02d %02d:%02d:%02d",
			current_time->tm_year + 1900, current_time->tm_mon + 1, current_time->tm_mday,
			current_time->tm_hour, current_time->tm_min, current_time->tm_sec);
	
	return 0;
}

/* Current Time Service Declaration */
BT_GATT_SERVICE_DEFINE(cts_cvs,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME, BT_GATT_CHRC_READ |
			       BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_ct, write_ct, ct),
	BT_GATT_CCC(ct_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void ble_update_time(struct k_work *work)
{
    struct timespec ts = {
        .tv_sec = 0,
        .tv_nsec = 0,
    };

	clock_gettime(CLOCK_REALTIME, &ts);
	LOG_INF("update time, current: %" PRId64 ", size %d" , ts.tv_sec, sizeof(time_t));
	memcpy(&ct, &ts.tv_sec, sizeof(time_t));
	
	// struct bt_gatt_attr *cts_not_attr;
	// cts_not_attr = bt_gatt_find_by_uuid(cts_cvs.attrs, cts_cvs.attr_count, BT_UUID_CTS_CURRENT_TIME);					
	
	bt_gatt_notify(NULL, &cts_cvs.attrs[1], ct, sizeof(ct));
	
}


//SYS_INIT(cts_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);