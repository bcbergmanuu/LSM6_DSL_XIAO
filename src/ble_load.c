#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "motion.pb.h"
#include "storage_nvs.h"


#define protobuf_chunk_size 64 //*13 = 247, actual 246 byte protobuf total/max mTU= 20byte
#define protobuf_buffer_size ROUND_UP(HourlyResult_size, protobuf_chunk_size)
//#define ble_chunks_to_send ROUND_UP(HourlyResult_size/protobuf_chunk_size, 1) //more than enough

LOG_MODULE_REGISTER(bluetooth_m, CONFIG_LOG_DEFAULT_LEVEL);

void ble_notify_motiondata_proc(struct k_work *ptrWorker);
void ble_send_passcode(struct k_work *work);

K_WORK_DEFINE(ble_passcode_worker, ble_send_passcode);
K_WORK_DEFINE(work_get_acc_data, ble_notify_motiondata_proc);

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

//accelometer data server (protobuf)
static struct bt_uuid_128 ble_accelometer_data_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x6f2648ed, 0xe73f, 0x4b9f, 0xa809, 0xb1686d18676c));

//accelometer requist data
static struct bt_uuid_128 ble_accelometer_requist_id_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x4e10fd71, 0x6cc1, 0x4fa7, 0xaa91, 0x82021582ca54));



static uint8_t ble_accelometer_data_buff[protobuf_chunk_size]; //size of packed protobuf
static uint8_t ble_requist_id_buff[2]; //uint16 flash location

//write flash ID to respond to
static ssize_t write_acc_data_position(struct bt_conn *conn, const struct bt_gatt_attr *attr,
 			 const void *buf, uint16_t len, uint16_t offset,
 			 uint8_t flags) {

 	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(ble_requist_id_buff)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	memcpy(value + offset, buf, len);	
	
	k_work_submit(&work_get_acc_data);
	
 	return len;
}
//read flash ID that has been responded to
static ssize_t read_acc_data_position(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(ble_requist_id_buff));
}

//read last chunk of protobuffed data 
static ssize_t read_acc_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{	
	
	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(ble_accelometer_data_buff));
}

//protobuf encode sensor data 
//todo: might want to use double?
static int encode_data(struct fifo_pack *stored_data, uint8_t *protobuf_packed, size_t *packed_size) {
	bool status;

	HourlyResult message = HourlyResult_init_zero;	
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(protobuf_packed, HourlyResult_size);
	
	message.UnixTime = stored_data->seconds_passed;

	BUILD_ASSERT(ARRAY_SIZE(message.AvgMinuteList) == FLASH_BUFFER_SIZE_NVS, "Error buffer not equal to array size");
	for(int i = 0; i<FLASH_BUFFER_SIZE_NVS; i++) {
		message.AvgMinuteList[i] = stored_data->vector_n[i];
	}	

	status = pb_encode(&stream, &HourlyResult_msg, &message);
	*packed_size = stream.bytes_written;

	LOG_INF("Encoded, actual protobuf message size was %d", stream.bytes_written); //PRIu64

	if (!status) {
		LOG_ERR("Encoding failed: %s\n", PB_GET_ERROR(&stream));
	}

	return status;
}


static uint8_t current_memoryPosition_buff[2];
static ssize_t read_memory_position(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint16_t mem_pos = 0;
	get_current_storage_id(&mem_pos);
	memcpy(current_memoryPosition_buff, &mem_pos, sizeof(uint16_t));

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
	memcpy(&memory_position, current_memoryPosition_buff, sizeof(uint16_t));
	set_current_storage_id(&memory_position);
	//memcpy(&memory_position, value, sizeof(uint16_t));	
	
 	return len;
}


static uint8_t ble_passcode_buff[4];
static ssize_t read_passcode(struct bt_conn *conn, const struct bt_gatt_attr *attr,
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
	memcpy(uniqueIdentifier_value, &uniqueId, sizeof(uint16_t));

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
	write_uniqueIdentifier(&uniqueId);
	
 	return len;
}

static uint8_t notify_ble_passcode_on = 0;
static uint8_t notify_ble_motiondata_on = 0;

static void ble_readmotion_data_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	notify_ble_motiondata_on = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static void ble_passcode_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	notify_ble_passcode_on = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

/* Motion readout primary Service Declaration */
BT_GATT_SERVICE_DEFINE(motion_svc,
	BT_GATT_PRIMARY_SERVICE(&motiondata_prop),
	BT_GATT_CHARACTERISTIC(&ble_pw_prop.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_passcode, NULL, ble_passcode_buff),	
	BT_GATT_CCC(ble_passcode_notify_changed,   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	BT_GATT_CHARACTERISTIC(&ble_device_identifier_prop.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_identifier, write_identifier, uniqueIdentifier_value),

	BT_GATT_CHARACTERISTIC(&ble_memory_position_prop.uuid, 
	 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,	 		       
	 		       read_memory_position, write_memory_position, current_memoryPosition_buff),

	BT_GATT_CHARACTERISTIC(&ble_accelometer_data_prop.uuid, 
	 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
	 		       BT_GATT_PERM_READ,
	 		       read_acc_data, NULL, ble_accelometer_data_buff),
	BT_GATT_CCC(ble_readmotion_data_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	BT_GATT_CHARACTERISTIC(&ble_accelometer_requist_id_prop.uuid, 
	 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	 		       read_acc_data_position, write_acc_data_position, ble_requist_id_buff),

);

void ble_send_passcode(struct k_work *work) {				
	
	if(notify_ble_passcode_on)
		bt_gatt_notify(NULL, &motion_svc.attrs[1], ble_passcode_buff, sizeof(ble_passcode_buff));	
}

void print_data(uint8_t *data, uint32_t len) {
	char string[255] = {0};
    
    
    LOG_INF("%s\n", string);
}

K_SEM_DEFINE(sem_notify, 0, 1);

void notify_complete_cb(struct bt_conn *conn, void *user_data)
{
	int err = 0;
    if (err != 0) {
        LOG_ERR("Notification failed with error %d", err);
        return;
    }
	k_sem_give(&sem_notify);
}


/// @brief Notify BLE with all chunks of data 
/// @param ptrWorker 
void ble_notify_motiondata_proc(struct k_work *ptrWorker) {					

	if(!notify_ble_motiondata_on) return;
	struct fifo_pack packet;
	uint16_t id_to_retreive;
	memcpy(&id_to_retreive, ble_requist_id_buff, sizeof(uint16_t));
	LOG_INF("NOTIFY data, reteive id %d", id_to_retreive);				
	load_tracked(id_to_retreive, &packet);
	
	uint8_t encoded_data_cache[protobuf_buffer_size] = {0};
	size_t packed_size = 0;
	encode_data(&packet, encoded_data_cache+2, &packed_size); //shift data 2 bytes
	memcpy(encoded_data_cache, &packed_size, sizeof(uint16_t)); //first 2 bytes for size

	struct bt_gatt_attr *notify_attr = bt_gatt_find_by_uuid(motion_svc.attrs, motion_svc.attr_count, &ble_accelometer_data_prop.uuid);
	
    struct bt_gatt_notify_params params = {
        .attr = notify_attr,
        .data = ble_accelometer_data_buff,
        .len  = sizeof(ble_accelometer_data_buff),
        .func = notify_complete_cb, 
    };

	for(size_t offset = 0; offset < packed_size; offset+=protobuf_chunk_size) {
		memcpy(ble_accelometer_data_buff, encoded_data_cache + offset, protobuf_chunk_size);	
				
		LOG_INF("sending offset %d", offset);
		
		int ret = bt_gatt_notify_cb(NULL, &params);
		int semret = k_sem_take(&sem_notify, K_MSEC(200));
		if(ret<0) {
			LOG_ERR("error notify data %d", ret);	
		}			
		if(semret == -EAGAIN) {			
			LOG_ERR("semaphone timed out");
			return;
		}
	}	
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

static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info) {
	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)\n", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_data_len_updated = le_data_length_updated,
};



#define BT_CONN_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
					    BT_LE_ADV_OPT_USE_NAME, \
					    768, 1024, NULL) //*1.25ms

static void bt_ready(void)
{
	int err;

	LOG_INF("Bluetooth initialized\n");	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_CONN_CUSTOM, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_INF("Advertising successfully started\n");
}
/*
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Passkey for %s: %06u\n", addr, passkey);
	memcpy(ble_passcode_buff, (uint8_t*)&passkey, (sizeof(ble_passcode_buff)));	
	k_work_submit(&ble_passcode_worker);
	bt_conn_auth_passkey_confirm(conn);
}
*/
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = NULL,// auth_passkey_display,
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

	//clean up bonds
	bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
	
	return err;
}

SYS_INIT(ble_load, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);