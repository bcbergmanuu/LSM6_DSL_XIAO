#ifndef LSM6DSL_LOAD_H
#define LSM6DSL_LOAD_H

#define TX_BUF_DIM          1000
#define BOOT_TIME        30 //ms 15ms minimum 
#define LSM6DSL_ADDR 0x6a
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define fifo_pattern 3 //3x 16 bit for only the accelometer
#define FIFO_BUFFER_LENGTH 675 //=3 byte per sample 12hz5 54sec
// int lsm6dsl_init(struct storage_module *storage);

#endif