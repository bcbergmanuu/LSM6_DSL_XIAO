#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

static struct nvs_fs fs;

LOG_MODULE_REGISTER(NVS_INIT, CONFIG_LOG_DEFAULT_LEVEL);

int init_storage() {

    const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

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
		return ;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return;
	}      
}