/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
#include "storage_nvs.h"

LOG_MODULE_REGISTER(bluetooth_m, CONFIG_LOG_DEFAULT_LEVEL);
#define AES_KEY_SIZE 16 // AES-128
#define AES_BLOCK_SIZE 16

void ble_send_passcode(struct k_work *work);
K_WORK_DEFINE(ble_passcode_worker, ble_send_passcode);

//custom service
static struct bt_uuid_128 motiondata_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xa6c47c93, 0x8a09, 0x4ca5, 0x976e, 0xde7b871f11f0));

//ble key
static struct bt_uuid_128 ble_pw_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xd97b1503, 0xa25c, 0x4295, 0xbd21, 0xf96823d91552));

//unique device
static struct bt_uuid_128 ble_device_identifier_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x56a332cc, 0x0379, 0x497e, 0xbd2d, 0xe778c69badd6));

//current memory position
static struct bt_uuid_128 ble_memory_position_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x84b45135, 0x170b, 0x4d03, 0x92b6, 0x386d7bff160d));

//accelometer data
static struct bt_uuid_128 ble_accelometer_data_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x54ea5a09, 0x97d5, 0x48ab, 0xbaf4, 0x007e9d7a45ff));

// static uint8_t accelometer_result_buff[sizeof(struct fifo_pack)];
// static ssize_t read_accelometer_result(struct bt_conn *conn, const struct bt_gatt_attr *attr,
// 			void *buf, uint16_t len, uint16_t offset)
// {
// 	uint16_t mem_pos = 0;
// 	get_current_storage_id(&mem_pos);
// 	memcpy(&current_memoryPosition_buff, &mem_pos, sizeof(uint16_t));

// 	const uint8_t *value = attr->user_data;	

// 	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(current_memoryPosition_buff));
// }

static uint8_t current_memoryPosition_buff[2];
static ssize_t read_memory_position(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint16_t mem_pos = 0;
	get_current_storage_id(&mem_pos);
	memcpy(&current_memoryPosition_buff, &mem_pos, sizeof(uint16_t));

	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(current_memoryPosition_buff));
}

static ssize_t write_memory_position(struct bt_conn *conn, const struct bt_gatt_attr *attr,
 			 const void *buf, uint16_t len, uint16_t offset,
 			 uint8_t flags) {

 	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(current_memoryPosition_buff)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	memcpy(value + offset, buf, len);

	uint16_t memory_position = 0;
	memcpy(&memory_position, value, sizeof(uint16_t));
	write_uniqueIdentifier(memory_position);
	
 	return len;
}


static uint8_t ble_passcode_buff[4];

static ssize_t read_current_item(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(ble_passcode_buff));
}

static uint8_t uniqueIdentifier_value[sizeof(uint16_t)];

static ssize_t read_identifier(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset) {

	uint16_t uniqueId = 0;
	read_uniqueidentifier(&uniqueId);
	memcpy(&uniqueIdentifier_value, &uniqueId, sizeof(uint16_t));

	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(uniqueIdentifier_value));
}

static ssize_t write_identifier(struct bt_conn *conn, const struct bt_gatt_attr *attr,
 			 const void *buf, uint16_t len, uint16_t offset,
 			 uint8_t flags) {

 	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(uniqueIdentifier_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	memcpy(value + offset, buf, len);

	uint16_t uniqueId = 0;
	memcpy(&uniqueId, value, sizeof(uint16_t));
	write_uniqueIdentifier(uniqueId);
	
 	return len;
}

//static int signed_value;

// static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
// 			   void *buf, uint16_t len, uint16_t offset)
// {
// 	const char *value = attr->user_data;

// 	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
// 				 sizeof(signed_value));
// }

// static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
// 			    const void *buf, uint16_t len, uint16_t offset,
// 			    uint8_t flags)
// {
// 	uint8_t *value = attr->user_data;

// 	if (offset + len > sizeof(signed_value)) {
// 		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
// 	}

// 	memcpy(value + offset, buf, len);

// 	return len;
// }

static uint8_t notify_ble_passcode_on = 0;


static void ble_passcode_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	notify_ble_passcode_on = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

/* Motion readout primary Service Declaration */
BT_GATT_SERVICE_DEFINE(motion_svc,
	BT_GATT_PRIMARY_SERVICE(&motiondata_prop),
	BT_GATT_CHARACTERISTIC(&ble_pw_prop.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_current_item, NULL, ble_passcode_buff),	
	BT_GATT_CCC(ble_passcode_notify_changed,   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	BT_GATT_CHARACTERISTIC(&ble_device_identifier_prop.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN,
			       read_identifier, write_identifier, uniqueIdentifier_value),
	BT_GATT_CHARACTERISTIC(&ble_memory_position_prop.uuid, BT_GATT_CHRC_READ |
	 		       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE,
	 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN,	 		       
	 		       read_memory_position, write_memory_position, current_memoryPosition_buff),
	//BT_GATT_CEP(&vnd_long_cep),
	// BT_GATT_CHARACTERISTIC(&vnd_signed_uuid.uuid, BT_GATT_CHRC_READ |
	// 		       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_AUTH,
	// 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	// 		       read_signed, write_signed, &signed_value),
	// BT_GATT_CHARACTERISTIC(&vnd_write_cmd_uuid.uuid,
	// 		       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
	// 		       BT_GATT_PERM_WRITE, NULL,
	// 		       write_without_rsp_vnd, &vnd_wwr_value),
);



void ble_send_passcode(struct k_work *work) {				
	
	bt_gatt_notify(NULL, &motion_svc.attrs[1], ble_passcode_buff, sizeof(ble_passcode_buff));	
}


	const struct bt_data ad[] = {
			BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
			BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL), BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),	};
			//BT_DATA_BYTES(BT_DATA_UUID128_ALL, motiondata_prop),
// 			BT_DATA(BT_DATA_MANUFACTURER_DATA, uniqueId, sizeof(ui))
// 	};
// }

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err 0x%02x)\n", err);
	} else {
		LOG_INF("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected
};



#define BT_CONN_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
					    BT_LE_ADV_OPT_USE_NAME, \
					    1024, 2048, NULL) //2 seconds

static void bt_ready(void)
{
	int err;

	LOG_INF("Bluetooth initialized\n");	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_CONN_CUSTOM, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_INF("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u\n", addr, passkey);
	
	memcpy(ble_passcode_buff, (uint8_t*)&passkey, (sizeof(ble_passcode_buff)));	

	k_work_submit(&ble_passcode_worker);

	bt_conn_auth_passkey_confirm(conn);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s\n", addr);
}


static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
	.passkey_confirm = NULL,	
};


int ble_load()
{		
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);
	
	return err;
}


SYS_INIT(ble_load, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);