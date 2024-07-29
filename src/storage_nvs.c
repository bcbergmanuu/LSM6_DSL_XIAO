#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "storage_nvs.h"

static struct nvs_fs fs;

LOG_MODULE_REGISTER(NVS_INIT, CONFIG_LOG_DEFAULT_LEVEL);

#define FLASH_BUFFER_SIZE 10
#define TEST_BUF 512
float *fifo_buffer;


void testnvs() {
	float *fifo_test_buffer = k_malloc(TEST_BUF * sizeof(float));
	float *fifo_test_buffer_read = k_malloc(TEST_BUF * sizeof(float));
	
	fifo_test_buffer[0] = 0.1;
	fifo_test_buffer[TEST_BUF-1] = .2;

	LOG_INF("buffer size: %d", TEST_BUF * sizeof(float));
	for(uint16_t x = 0; x < 10; x ++) {

		nvs_write(&fs, x, fifo_test_buffer, TEST_BUF * sizeof(float));
		nvs_read(&fs, x, fifo_test_buffer_read, TEST_BUF * sizeof(float) );
		//if(x % 10 == 0)
			LOG_INF("read buffer: %d, nums: %4.2f, %4.2f", x, fifo_test_buffer_read[0],  fifo_test_buffer_read[TEST_BUF-1]);
	}
}

int init_storage(struct storage_module *storage) {	
    const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	fifo_buffer = k_malloc(FLASH_BUFFER_SIZE * sizeof(float));

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("%s: device not ready.\n", flash_dev->name);
		return ENODEV;
	}

	printf("\n%s SPI flash testing\n", flash_dev->name);

    int rc = 0;
    struct flash_pages_info info;

	fs.flash_device = flash_dev;
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
		return ENODEV;
	}
    LOG_INF("flash device found: %s", fs.flash_device->name);
	fs.offset = 0;
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return -1;
	}
	fs.sector_size = info.size;
	fs.sector_count = 512; //*4096 == 2MB?

	rc = nvs_mount(&fs);
	LOG_INF("fs sector size = %d", info.size);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return -1;
	}     

	storage->store_tracked = store_tracked;
	testnvs();
	return 0;
}




void testflash(uint16_t position) {
  LOG_INF("storage feedback");
  float buffer[FLASH_BUFFER_SIZE] = {0};
  ssize_t ret = nvs_read(&fs, position, buffer, FLASH_BUFFER_SIZE * sizeof(float));
  if (ret < 0) LOG_ERR("could not read flash");
  for(int x = 0; x< FLASH_BUFFER_SIZE; x++) {
    LOG_INF("value %d: %4.2f", x, buffer[x]);
  }
}



void store_tracked(float vector) {
  LOG_INF("storing information");
  static uint16_t storage_id = 0;  
  static uint32_t fifo_counter = 0;
  
  fifo_buffer[fifo_counter] = vector;
  fifo_counter++;
  //store 10 items each time
  if(fifo_counter > FLASH_BUFFER_SIZE) {
    fifo_counter = 0;  
    ssize_t ret = nvs_write(&fs, storage_id, fifo_buffer, FLASH_BUFFER_SIZE * sizeof(float));
	if(ret < 0) LOG_ERR("could not write flash");
    LOG_INF("stored at position %d", storage_id);
    testflash(storage_id);
    storage_id++;    
  }
}