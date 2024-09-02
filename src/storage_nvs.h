#ifndef LSM6DSL_STORAGE_H
#define LSM6DSL_STORAGE_H

//store each 60 minutes
#define FLASH_BUFFER_SIZE_NVS 60
#define FLASH_SIZE      (2 * 1024 * 1024)    // 2 MB in bytes
#define FLASH_SECTOR_SIZE 4096               // 4 KB in bytes
#define FLASH_SECTOR_COUNT (FLASH_SIZE / FLASH_SECTOR_SIZE)
#define FLASH_PAGE_SIZE 256
#define STORAGE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(storage_partition)
#define NVS_CURRENT_STORAGE_ID 4
#define NVS_DEVICE_ID 5

/// @brief package containing all items 
struct fifo_pack {
	time_t seconds_passed;
	float vector_n[FLASH_BUFFER_SIZE_NVS];	
};
BUILD_ASSERT(sizeof(struct fifo_pack) <= FLASH_PAGE_SIZE, "keep below FLASH_PAGE_SIZE");	 

struct fifo_data_t {
    void *fifo_reserved;
    float fifo_buffer;
};


int read_uniqueidentifier(uint16_t *identifier);
int write_uniqueIdentifier(uint16_t identifier);

int load_tracked(uint16_t itemid, struct fifo_pack *vector_list);
void store_tracked(float vector);

int set_current_storage_id(uint16_t itemid);
int get_current_storage_id(uint16_t *itemid);


#endif