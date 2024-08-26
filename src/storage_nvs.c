#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/posix/time.h>
#include <zephyr/sys/timeutil.h>

#include <zephyr/pm/device.h>

#include "storage_nvs.h"

static struct nvs_fs fs;

LOG_MODULE_REGISTER(STORAGE_MODULE, CONFIG_LOG_DEFAULT_LEVEL);

#define FLASH_SIZE      (2 * 1024 * 1024)    // 2 MB in bytes
#define FLASH_SECTOR_SIZE 4096               // 4 KB in bytes
#define FLASH_SECTOR_COUNT (FLASH_SIZE / FLASH_SECTOR_SIZE)
#define FLASH_PAGE_SIZE 256
#define STORAGE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(storage_partition)
#define NVS_CURRENT_STORAGE_ID 4

K_FIFO_DEFINE(storage_fifo);
const struct device *spi_flash_dev;
 

int init_internal_storage() {
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

int set_time(time_t epoch_time) {
	
	 // Set the initial time (e.g., Jan 1, 2023, 00:00:00 UTC)
    // struct tm initial_time = {
    //     .tm_year = 2023 - 1900,
    //     .tm_mon = 0,
    //     .tm_mday = 1,
    //     .tm_hour = 0,
    //     .tm_min = 0,
    //     .tm_sec = 0,
    // };
    
	//time_t epoch_time = timeutil_timegm(&initial_time);

    struct timespec ts = {
        .tv_sec = epoch_time,
        .tv_nsec = 0,
    };

    // Set the initial time
    clock_settime(CLOCK_REALTIME, &ts);
	
	clock_gettime(CLOCK_REALTIME, &ts);
	struct tm *current_time = gmtime(&ts.tv_sec);

	LOG_INF("Current time: %04d-%02d-%02d %02d:%02d:%02d",
			current_time->tm_year + 1900, current_time->tm_mon + 1, current_time->tm_mday,
			current_time->tm_hour, current_time->tm_min, current_time->tm_sec);
	
	return 0;
}

int init_storage(struct storage_module *storage) {	
		
    int rc = 0;
	rc |= init_internal_storage();
	
	spi_flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));	
	if (!device_is_ready(spi_flash_dev)) {
		LOG_ERR("%s: device not ready.\n", spi_flash_dev->name);
		return ENODEV;
	}

	
    struct flash_pages_info info;

	LOG_INF("flash device found: %s", spi_flash_dev->name);	
	
	rc = flash_get_page_info_by_offs(spi_flash_dev, 0, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return -1;
	}
	
	//has bug, https://devzone.nordicsemi.com/f/nordic-q-a/114224/external-flash-to-sleep
	// if(pm_device_action_run(spi_flash_dev, PM_DEVICE_ACTION_SUSPEND)) { 
	// 	LOG_ERR("could not suspend qspi flash device %s", spi_flash_dev->name);
	// }

	storage->store_tracked = store_tracked;
	storage->load_tracked = load_tracked;
	

	return rc;
}

void store_tracked(float vector) {	
	struct fifo_data_t *data = k_malloc(sizeof(struct fifo_data_t));
	data->vector_n = vector;
	k_fifo_alloc_put(&storage_fifo, data);
}

void load_tracked(uint16_t id, float *vector_list) {
	nvs_read(&fs, id, vector_list, sizeof(float)*FLASH_BUFFER_SIZE_NVS);
}

struct fifo_pack {
	time_t seconds_passed;
	float fifo_buffer[FLASH_BUFFER_SIZE_NVS];	
};

BUILD_ASSERT(sizeof(struct fifo_pack) <= FLASH_PAGE_SIZE, "keep below FLASH_PAGE_SIZE");	


/// @brief test wether storage has been written
void read_storage() {
	
	uint16_t lastid = 0;
	struct fifo_pack buffer;
	nvs_read(&fs, NVS_CURRENT_STORAGE_ID, &lastid, sizeof(uint16_t));
	LOG_INF("nvs stored id: %d", lastid);
	flash_read(spi_flash_dev, lastid * FLASH_PAGE_SIZE, &buffer, sizeof(struct fifo_pack));
	LOG_INF("timestamp: %" PRId64, buffer.seconds_passed);
	for(int x = 0; x < FLASH_BUFFER_SIZE_NVS; x+=10) {		
		LOG_INF("datapoint %d: %4.2f", x, buffer.fifo_buffer[x]);
	}
}



void fifo_handler(void *) {
	LOG_INF("fifo_handler thread initialized");
	struct fifo_data_t *rx_value = {0};
	static uint16_t storage_id = 0;
	static uint32_t fifo_counter = 0;		
	struct fifo_pack fifo_packer ={.seconds_passed =0, .fifo_buffer = {0}};			
	
	while(true) {				
		
		rx_value = k_fifo_get(&storage_fifo, K_FOREVER);			
		k_free(rx_value);
	
		fifo_packer.fifo_buffer[fifo_counter] = rx_value->vector_n;
						
		fifo_counter++;

		if(fifo_counter >= FLASH_BUFFER_SIZE_NVS) {
			fifo_counter = 0;  			
			//set time for next packet to store
			struct timespec ts = {0};
			clock_gettime(CLOCK_REALTIME, &ts);		
			fifo_packer.seconds_passed = ts.tv_sec;
					
			ssize_t ret = flash_write(spi_flash_dev, storage_id * FLASH_PAGE_SIZE, &fifo_packer, sizeof(float)*FLASH_BUFFER_SIZE_NVS);
			if(ret < 0) {
				LOG_ERR("could not write flash id: %d", storage_id);  	
			}
			else {
				LOG_INF("stored at position %d", storage_id);
				nvs_write(&fs, NVS_CURRENT_STORAGE_ID, &storage_id, sizeof(uint16_t));
				storage_id++; 
			}
					
			read_storage();
		}		
	}
}

K_THREAD_DEFINE(fifohandler, 1024, fifo_handler, NULL, NULL, NULL, 8, 0, 0);