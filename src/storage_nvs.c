#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/posix/time.h>
#include <zephyr/sys/timeutil.h>

#include <zephyr/pm/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "storage_nvs.h"

static struct nvs_fs fs;

LOG_MODULE_REGISTER(STORAGE_MODULE, CONFIG_LOG_DEFAULT_LEVEL);

K_FIFO_DEFINE(storage_fifo);
const struct device *spi_flash_dev;



static int flash_set_suspend() {
	int ret = pm_device_action_run(spi_flash_dev, PM_DEVICE_ACTION_SUSPEND);
	if(ret) { 
		LOG_ERR("could not suspend qspi flash device %s", spi_flash_dev->name);
	}
	return ret;	
}

static int flash_set_resume() {
	int ret = pm_device_action_run(spi_flash_dev, PM_DEVICE_ACTION_RESUME);
	if(ret) { 
		LOG_ERR("could not suspend qspi flash device %s", spi_flash_dev->name);
	}
	return ret;
}

int init_internal_storage() {
	LOG_INF("init internal storage");
	const struct device *flash_dev = STORAGE_PARTITION_DEVICE;
	
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("%s: device not ready.\n", flash_dev->name);
		return ENODEV;
	}
	fs.flash_device = flash_dev;
    int rc = 0;	

    struct flash_pages_info info;

	LOG_INF("flash device found: %s", fs.flash_device->name);
	fs.offset = DT_REG_ADDR(DT_NODELABEL(storage_partition));;
	
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return -1;
	}
	fs.sector_size = info.size;
	fs.sector_count = (DT_REG_SIZE(DT_NODELABEL(storage_partition)) / info.size); 

	rc = nvs_mount(&fs);
	LOG_INF("fs sector size = %d", info.size);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return -1;
	}  
	
	uint16_t storage_id = 0;
	nvs_read(&fs, NVS_CURRENT_STORAGE_ID, &storage_id, sizeof(uint16_t));
	LOG_INF("last written to %d", storage_id);
	

	return rc;
}

int write_uniqueIdentifier(uint16_t *identifier) {
	nvs_write(&fs, NVS_DEVICE_ID, identifier, sizeof(uint16_t) );
	return 0;
}

int read_uniqueidentifier(uint16_t *identifier) {
	nvs_read(&fs, NVS_DEVICE_ID, identifier, sizeof(uint16_t) );
	return 0;
}

int init_storage(void) {	
	LOG_INF("init storage");
    int rc = 0;
	rc |= init_internal_storage();
	
	spi_flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));	
	if (!device_is_ready(spi_flash_dev)) {
		LOG_ERR("%s: device not ready.\n", spi_flash_dev->name);
		return rc;
	}
	
    struct flash_pages_info info;

	LOG_INF("flash device found: %s", spi_flash_dev->name);	
	
	rc = flash_get_page_info_by_offs(spi_flash_dev, 0, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return rc;
	}
		
	rc |= flash_set_suspend();	
	return rc;
}

//store one item in ram memory; for every 60 items flash will be written.
void store_tracked(float vector) {	
	struct fifo_data_t *data = k_malloc(sizeof(struct fifo_data_t));
	data->fifo_buffer = vector;
	k_fifo_put(&storage_fifo, data);
}

int set_current_storage_id(uint16_t *itemid) {
	int err = nvs_write(&fs, NVS_CURRENT_STORAGE_ID, itemid, sizeof(uint16_t));
	LOG_INF("nvs storage id set to: %d", *itemid);
	if (err < 0) return err; else return 0;
}

int get_current_storage_id(uint16_t *itemid) {	
	int err = nvs_read(&fs, NVS_CURRENT_STORAGE_ID, itemid, sizeof(uint16_t));
	if (err < 0) {
		LOG_ERR("error getting storageid");
		return err;
	}	
	LOG_INF("storage requisted current id: %d", *itemid);
	return 0;
}

int load_tracked(uint16_t itemid, struct fifo_pack *vector_list) {
	int ret = 0;	
	if(itemid > 8192) {
		LOG_ERR("invalid itemid request");
		return -EFAULT;
	}
	ret |= flash_set_resume();
	ret |= flash_read(spi_flash_dev, itemid * FLASH_PAGE_SIZE, vector_list, sizeof(struct fifo_pack));
	ret |= flash_set_suspend();
		
	
	LOG_INF("loaded item %d", itemid);
	
	return ret;
}

void fifo_handler(void *) {
	LOG_INF("fifo_handler thread initialized");
	struct fifo_data_t *rx_value = {0};
	
	static uint32_t fifo_counter = 0;		
	struct fifo_pack fifo_packer ={.seconds_passed =0, .vector_n = {0}};			
	
	while(true) {				
		rx_value = k_fifo_get(&storage_fifo, K_FOREVER);			
		k_free(rx_value);
	
		fifo_packer.vector_n[fifo_counter] = rx_value->fifo_buffer;						
		fifo_counter++;

		if(fifo_counter >= FLASH_BUFFER_SIZE_NVS) {
			int ret =0;			
			uint16_t flash_position;
			ret |= get_current_storage_id(&flash_position);
			flash_position++; //store in next position
			if(flash_position > 8192) flash_position = 0; //2MB page file 256 == 8192 items. When full start from beginning
			
			struct timespec ts = {0};
			
			//set time for next packet to store
			clock_gettime(CLOCK_REALTIME, &ts);		
			fifo_packer.seconds_passed = ts.tv_sec;
			
			ret |= flash_set_resume();
			ret |= flash_write(spi_flash_dev, flash_position * FLASH_PAGE_SIZE, &fifo_packer, sizeof(struct fifo_pack));
			ret |= flash_set_suspend();
			if(ret < 0) {
				LOG_ERR("could not write flash id: %d", flash_position);  	
			}
			else {
				LOG_INF("stored at position %d", flash_position);
				set_current_storage_id(&flash_position);
			}							
			fifo_counter = 0;  
		}		
	}
}

K_THREAD_DEFINE(fifohandler, 1024, fifo_handler, NULL, NULL, NULL, 8, 0, 0);


SYS_INIT(init_storage, APPLICATION, 64);