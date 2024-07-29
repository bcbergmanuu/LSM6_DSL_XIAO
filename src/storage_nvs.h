#ifndef LSM6DSL_STORAGE_H
#define LSM6DSL_STORAGE_H


struct storage_module {
    void (* store_tracked)(float);
};

int init_storage(struct storage_module *storage);


void store_tracked(float vector);



#endif