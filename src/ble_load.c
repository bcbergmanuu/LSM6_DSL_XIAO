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

#include <zephyr/bluetooth/services/ias.h>

#include <zephyr/logging/log.h>


#include "cts.h"

LOG_MODULE_REGISTER(bluetooth_m, CONFIG_LOG_DEFAULT_LEVEL);


/* Custom Service for readout motion */
#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xa6c47c93, 0x8a09, 0x4ca5, 0x976e, 0xde7b871f11f0)

static struct bt_uuid_128 motiondatauuid = BT_UUID_INIT_128(
	BT_UUID_CUSTOM_SERVICE_VAL);

//readout variable
static struct bt_uuid_128 current_item_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xd97b1503, 0xa25c, 0x4295, 0xbd21, 0xf96823d91552));

// static struct bt_uuid_128 vnd_auth_uuid = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

//#define VND_MAX_LEN 20

static uint8_t vnd_value[10 + 1] = { 'V', 'e', 'n', 'd', 'o', 'r'};
//static uint8_t vnd_auth_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r'};
//static uint8_t vnd_wwr_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r' };

static ssize_t read_current_item(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t write_current_item(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	//value[offset + len] = 0;
	LOG_INF("got value offset: %d, length: %d", offset, len);

	return len;
}

//static uint8_t simulate_vnd;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	//simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

// #define VND_LONG_MAX_LEN 74
// static uint8_t vnd_long_value[VND_LONG_MAX_LEN + 1] = {
// 		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '1',
// 		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '2',
// 		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '3',
// 		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '4',
// 		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '5',
// 		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '6',
// 		  '.', ' ' };

// static ssize_t write_long_vnd(struct bt_conn *conn,
// 			      const struct bt_gatt_attr *attr, const void *buf,
// 			      uint16_t len, uint16_t offset, uint8_t flags)
// {
// 	uint8_t *value = attr->user_data;

// 	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
// 		return 0;
// 	}

// 	if (0) {
// 		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
// 	}

// 	memcpy(value + offset, buf, len);
// 	value[offset + len] = 0;

// 	return len;
// }

// static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3));

// static struct bt_gatt_cep vnd_long_cep = {
// 	.properties = BT_GATT_CEP_RELIABLE_WRITE,
// };

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

// static const struct bt_uuid_128 vnd_signed_uuid = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x13345678, 0x1234, 0x5678, 0x1334, 0x56789abcdef3));

// static const struct bt_uuid_128 vnd_write_cmd_uuid = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4));

// static ssize_t write_without_rsp_vnd(struct bt_conn *conn,
// 				     const struct bt_gatt_attr *attr,
// 				     const void *buf, uint16_t len, uint16_t offset,
// 				     uint8_t flags)
// {
// 	uint8_t *value = attr->user_data;

// 	if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
// 		/* Write Request received. Reject it since this Characteristic
// 		 * only accepts Write Without Response.
// 		 */
// 		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
// 	}

// 	if (0) {
// 		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
// 	}

// 	memcpy(value + offset, buf, len);
// 	value[offset + len] = 0;

// 	return len;
// }

/* Motion readout primary Service Declaration */
BT_GATT_SERVICE_DEFINE(vnd_svc,
	BT_GATT_PRIMARY_SERVICE(&motiondatauuid),
	BT_GATT_CHARACTERISTIC(&current_item_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
			       BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       read_current_item, write_current_item, vnd_value),
	BT_GATT_CCC(vnd_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT)
	// BT_GATT_CHARACTERISTIC(&vnd_auth_uuid.uuid,
	// 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	// 		       BT_GATT_PERM_READ_AUTHEN |
	// 		       BT_GATT_PERM_WRITE_AUTHEN,
	// 		       read_vnd, write_vnd, vnd_auth_value),
	// BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid, BT_GATT_CHRC_READ |
	// 		       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
	// 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE |
	// 		       BT_GATT_PERM_PREPARE_WRITE,
	// 		       read_vnd, write_long_vnd, &vnd_long_value),
	// BT_GATT_CEP(&vnd_long_cep),
	// BT_GATT_CHARACTERISTIC(&vnd_signed_uuid.uuid, BT_GATT_CHRC_READ |
	// 		       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_AUTH,
	// 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	// 		       read_signed, write_signed, &signed_value),
	// BT_GATT_CHARACTERISTIC(&vnd_write_cmd_uuid.uuid,
	// 		       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
	// 		       BT_GATT_PERM_WRITE, NULL,
	// 		       write_without_rsp_vnd, &vnd_wwr_value),
);

//advertising data:
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

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

static void alert_stop(void)
{
	LOG_INF("Alert stopped\n");
}

static void alert_start(void)
{
	LOG_INF("Mild alert started\n");
}

static void alert_high_start(void)
{
	LOG_INF("High alert started\n");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

BT_IAS_CB_DEFINE(ias_callbacks) = {
	.no_alert = alert_stop,
	.mild_alert = alert_start,
	.high_alert = alert_high_start,
};

#define BT_CONN_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
					    BT_LE_ADV_OPT_USE_NAME, \
					    12288, 16384, NULL) //10.2 seconds max

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
};



int ble_load(void)
{
	struct bt_gatt_attr *vnd_ind_attr;
	char str[BT_UUID_STR_LEN];
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	vnd_ind_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count,
					    &current_item_uuid.uuid);
	bt_uuid_to_str(&current_item_uuid.uuid, str, sizeof(str));
	LOG_INF("Indicate VND attr %p (UUID %s)\n", vnd_ind_attr, str);	
	
	return 0;
}
