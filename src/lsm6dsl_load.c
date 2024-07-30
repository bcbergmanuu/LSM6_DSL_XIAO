
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include "lsm6dsl_reg.h"
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "storage_nvs.h"

#define TX_BUF_DIM          1000
#define BOOT_TIME        30 //ms 15ms minimum 
#define LSM6DSL_ADDR 0x6a
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define fifo_pattern 3 //3x 16 bit for only the accelometer
#define FIFO_BUFFER_LENGTH 675 //2046max /3 byte per sample samples /26hz =27sec

BUILD_ASSERT(FIFO_BUFFER_LENGTH % fifo_pattern == 0, "FIFO_BUFFER_LENGTH should be devisible by fifo_pattern");

static const struct gpio_dt_spec signal = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, signal_gpios);

LOG_MODULE_REGISTER(LSM6DSL_LOAD, CONFIG_LOG_DEFAULT_LEVEL);

static uint8_t whoamI, rst;
const struct device *i2c;
static struct gpio_callback lsm6dsl_interrupt;
static stmdev_ctx_t dev_ctx;

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

static axis3bit16_t data_raw_acceleration;

void lsm6dsl_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void lsm6dsl_data_handler(struct k_work *work);
K_WORK_DEFINE(lsm6dsl_proces, lsm6dsl_data_handler);



void attachinterrupt(void) {
	
	__ASSERT(device_is_ready(signal.port), "custom device not ready");
	if(gpio_pin_configure_dt(&signal, GPIO_INPUT)) {
    LOG_ERR("error configure signalgpio");
  }
	if(gpio_pin_interrupt_configure_dt(&signal, GPIO_INT_EDGE_TO_ACTIVE)) {
    LOG_ERR("error configure interrupt signalgpio");
  };
	gpio_init_callback(&lsm6dsl_interrupt, lsm6dsl_isr, BIT(signal.pin));
	if(gpio_add_callback(signal.port, &lsm6dsl_interrupt)) {
    LOG_ERR("callback for interrupt not set");
  };	
}

/* Private functions ---------------------------------------------------------*/

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

static void platform_delay(uint32_t ms);
static void platform_init(void);

void lsm6dsl_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {  
  LOG_INF("isr given");
  k_work_submit(&lsm6dsl_proces);
}

struct xlData {
    float x;
    float y;
    float z;
};  

struct storage_module *storage_m;

int lsm6dsl_init(struct storage_module *storage)
{  
  storage_m = storage;
  
  attachinterrupt();
  /* Initialize mems driver interface */  
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  whoamI = 0;
  lsm6dsl_device_id_get(&dev_ctx, &whoamI);

  if ( whoamI != LSM6DSL_ID ) {
    LOG_ERR("device not found");
    return -1;
  }

  /* Restore default configuration */
  lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsl_reset_get(&dev_ctx, &rst);
  } while (rst);

  lsm6dsl_fifo_mode_set(&dev_ctx, LSM6DSL_STREAM_MODE);
  lsm6dsl_xl_power_mode_set(&dev_ctx, LSM6DSL_XL_NORMAL);
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  lsm6dsl_fifo_xl_batch_set(&dev_ctx, LSM6DSL_FIFO_XL_NO_DEC);
  lsm6dsl_fifo_gy_batch_set(&dev_ctx, LSM6DSL_FIFO_GY_DISABLE);
  lsm6dsl_fifo_watermark_set(&dev_ctx, FIFO_BUFFER_LENGTH*fifo_pattern);
  lsm6dsl_fifo_write_trigger_set(&dev_ctx, LSM6DSL_TRG_XL_GY_DRDY);
  lsm6dsl_den_mode_set(&dev_ctx, LSM6DSL_EDGE_TRIGGER);
  lsm6dsl_fifo_data_rate_set(&dev_ctx, LSM6DSL_FIFO_208Hz); //26
  lsm6dsl_rounding_mode_set(&dev_ctx, LSM6DSL_ROUND_XL);
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_833Hz); //104
  lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_4);  
  
  lsm6dsl_int1_route_t intset1 = { .int1_fth = 1};
  lsm6dsl_pin_int1_route_set(&dev_ctx, intset1); 
  
  LOG_INF("settigs completed");
  return 0;
}

static void lsm6dsl_data_handler(struct k_work *work)
{
  uint16_t num = 0;
  uint32_t counter = 0;
  
  struct xlData average_xl = {0};
  
  memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));

  //determine amount of entries in fifo
  lsm6dsl_fifo_data_level_get(&dev_ctx, &num);
  
  LOG_INF("numvalues %d", (int)num);
  
  if(!(num % fifo_pattern == 0))
    LOG_ERR("fifo pattern broken!");
  for(int x = 0; x <= num; x+=fifo_pattern) {        
    
    lsm6dsl_fifo_raw_data_get(&dev_ctx, data_raw_acceleration.u8bit, fifo_pattern * sizeof(int16_t));                                                          

    average_xl.x += lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
    average_xl.y += lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
    average_xl.z += lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
    counter++;                        
  }
  average_xl.x/= counter;
  average_xl.y/= counter;
  average_xl.z/= counter;
  LOG_INF("average: %4.2f\t%4.2f\t%4.2f", average_xl.x, average_xl.y, average_xl.z);

  float vector = sqrt(pow(average_xl.x,2)+pow(average_xl.y,2)+pow(average_xl.z,2));
  LOG_INF("average vector = sqrt(a^2+b^2+c^2) = %4.2f", vector);
  //todo: use zephyr fifo buffer?
  storage_m->store_tracked(vector);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{  

	int error = i2c_burst_write(i2c, LSM6DSL_ADDR, reg, bufp, len);
  if (error) {
    return -EIO;
  } else {
    return 0;
  };
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{  
	 if(i2c_burst_read(i2c, LSM6DSL_ADDR, reg, bufp, len)){    
    return -EIO;
   }   
   return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
    k_sleep(K_MSEC(ms));
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
	i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	__ASSERT(device_is_ready(i2c), "max30102 device not ready");

  
}