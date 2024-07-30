#ifndef LSM6DSL_STORAGE_H
#define LSM6DSL_STORAGE_H

//#define FLASH_BUFFER_SIZE 9 //test falue use --> 64 has to be deviceable by 3
#define FLASH_BUFFER_DIV 3
//BUILD_ASSERT(FLASH_BUFFER_SIZE % FLASH_BUFFER_DIV ==0, "FLASH_BUFFER_SIZE should be devisible by 3");
#define FLASH_BUFFER_SIZE_NVS 3 // FLASH_BUFFER_SIZE / FLASH_BUFFER_DIV

struct storage_module {
    void (* store_tracked)(float);
    void (* load_tracked)(uint16_t, float*);
};


struct fifo_data_t {
    void *fifo_reserved;
    float vector_n;
};


int init_storage(struct storage_module *storage);


void store_tracked(float vector);

void load_tracked(uint16_t id, float *vector_list);

#endif