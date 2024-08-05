#ifndef LSM6DSL_STORAGE_H
#define LSM6DSL_STORAGE_H

//buffer of 30sec, average of total 60s
#define FLASH_BUFFER_DIV 2
//store each 60 minutes
#define FLASH_BUFFER_SIZE_NVS 4

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