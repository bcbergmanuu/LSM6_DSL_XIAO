
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "storage_nvs.h"
#include "lsm6dsl_reg.h"
//#include <hal/nrf_gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define TX_BUF_DIM          1000
#define BOOT_TIME        30 //ms 15ms minimum 
#define LSM6DSL_ADDR 0x6a
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define fifo_pattern 3 //3x 16 bit for only the accelometer
#define FIFO_BUFFER_LENGTH 675 //2046max /3 byte per sample samples /12hz5 =54sec

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
K_WORK_DELAYABLE_DEFINE(lsm6dsl_proces, lsm6dsl_data_handler);


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
  //gpio_pin_interrupt_configure_dt(&signal, GPIO_INT_DISABLE);
  LOG_INF("isr given");

  k_work_schedule(&lsm6dsl_proces, K_NO_WAIT); 
}

struct xlData {
    float x;
    float y;
    float z;
};  

//struct storage_module *storage_m;

int lsm6dsl_init(/*struct storage_module *storage */)
{  
  //storage_m = storage;
  
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
  //lsm6dsl_fifo_write_trigger_set(&dev_ctx, LSM6DSL_TRG_XL_GY_DRDY);
  lsm6dsl_den_mode_set(&dev_ctx, LSM6DSL_LEVEL_LETCHED);
  lsm6dsl_fifo_data_rate_set(&dev_ctx, LSM6DSL_FIFO_12Hz5); //12Hz5
  lsm6dsl_rounding_mode_set(&dev_ctx, LSM6DSL_ROUND_XL);
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_52Hz);  //52Hz
  lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_4);    

//exp
  // lsm6dsl_tilt_sens_set(&dev_ctx, 1);  
  // uint8_t val = 0;
  // LOG_INF("trigger received:%d", val);
//end exp

  lsm6dsl_int1_route_t intset1 = { .int1_fth = 1, /* .int1_tilt = 1 */};
  lsm6dsl_pin_int1_route_set(&dev_ctx, intset1); 
  
  LOG_INF("settigs completed");
  return 0;
}

static uint8_t ble_acc_last_value_buff[4];
static ssize_t read_acc_last_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(ble_acc_last_value_buff));
}

static struct bt_uuid_128 ble_motion_sensor_result_prop = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x7c3beebb, 0xba2d, 0x4634, 0xa999, 0xf61b91c75f49));

static uint8_t notify_ble_motion_activity_value_on = 0;

static void ble_motion_activity_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	notify_ble_motion_activity_value_on = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}



/* Current Time Service Declaration */
BT_GATT_SERVICE_DEFINE(ble_motion_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0x170B)),
	BT_GATT_CHARACTERISTIC(&ble_motion_sensor_result_prop.uuid, 
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			      BT_GATT_PERM_READ,
			      read_acc_last_value, NULL, ble_acc_last_value_buff),
	BT_GATT_CCC(ble_motion_activity_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// static int32_t testtilt(const stmdev_ctx_t *ctx, uint8_t *val) {
//   uint32_t ret = 0;
//   lsm6dsl_func_src1_t result_trig;
//   ret |= lsm6dsl_read_reg(ctx, LSM6DSL_FUNC_SRC1, (uint8_t *)&result_trig, 1);
//   *val = result_trig.tilt_ia;
//   return ret;
// }

void ble_showmotion(struct k_work *work)
{
  if(notify_ble_motion_activity_value_on) 
      bt_gatt_notify(NULL, &ble_motion_service.attrs[1], ble_acc_last_value_buff, sizeof(ble_acc_last_value_buff));	
}

K_WORK_DEFINE(work_ble_showmotion, ble_showmotion);

static int fifo_threashold_handler(void) {
  int ret = 0;
  uint16_t num = 0;
  uint32_t counter = 0;
  
  struct xlData average_xl = {0};
  
  memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));

  ret |= lsm6dsl_fifo_data_level_get(&dev_ctx, &num);
  
  LOG_INF("numvalues %d", (int)num);
  if(num == 0){
    return -1;
    LOG_ERR("triggered but no data?!");
  }
  for(int x = 0; x <= (num-fifo_pattern); x+=fifo_pattern) {        
      
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
  LOG_INF("average) = %4.2f", vector);
  
  store_tracked(vector);
  memcpy(&ble_acc_last_value_buff, &vector, sizeof(float));
  k_work_submit(&work_ble_showmotion);
  return ret;
}

static void lsm6dsl_data_handler(struct k_work *work)
{  
  //uint8_t val = 0;
  //testtilt(&dev_ctx, &val);
  
  //if(val) {
  //  LOG_INF("TILT.IA trigger received:%d", val);    
  //} else {
    if(fifo_threashold_handler()) {
      LOG_ERR("error handling fifo");
    }
  //}
    
  //gpio_pin_interrupt_configure_dt(&signal, GPIO_INT_LEVEL_HIGH);
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
}

SYS_INIT(lsm6dsl_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);