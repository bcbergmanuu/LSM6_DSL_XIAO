
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "storage_nvs.h"
#include "lsm6dsl_reg.h"
#include "lsm6dsl_load.h"
#include "led_indicator.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>



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

int lsm6dsl_set_interrupts(bool withDoubletap) {
  int ret = 0;
  lsm6dsl_int1_route_t intset1 = { .int1_fth = 1, /*.int1_tilt = 1, */ .int1_sign_mot = 1, .int1_double_tap = withDoubletap};
  ret |= lsm6dsl_pin_int1_route_set(&dev_ctx, intset1); 
  return ret;
}

static bool sigMotionDetected = false;

/// @brief Enables significant motion detection
/// @return 0 if succeed
int lsm6dsl_config_doubletap(bool setEnabled) {
  int ret = 0;

  lsm6dsl_xl_data_rate_set(&dev_ctx, setEnabled ? LSM6DSL_XL_ODR_416Hz : LSM6DSL_XL_ODR_52Hz); 
  lsm6dsl_tap_cfg_t tap_cfg = {.tap_x_en = setEnabled, .tap_y_en = setEnabled, .tap_z_en = setEnabled, .interrupts_enable =setEnabled, .inact_en = 0, .slope_fds = 0};
  lsm6dsl_tap_ths_6d_t tab_ths = {.tap_ths =12, .d4d_en =1};
  lsm6dsl_int_dur2_t tap_dur = {.dur = 7, .quiet =3, .shock = 3};
  lsm6dsl_wake_up_ths_t tap_ths = {.single_double_tap = 1};
  //lsm6dsl_xl_hp_path_internal_set(&dev_ctx, &tap_cfg);
  ret |= lsm6dsl_write_reg(&dev_ctx, LSM6DSL_TAP_CFG, (uint8_t *)&tap_cfg, 1);
  ret |= lsm6dsl_write_reg(&dev_ctx, LSM6DSL_TAP_THS_6D, (uint8_t*)&tab_ths, 1);
  ret |= lsm6dsl_write_reg(&dev_ctx, LSM6DSL_INT_DUR2, (uint8_t *)&tap_dur, 1);
  ret |= lsm6dsl_write_reg(&dev_ctx, LSM6DSL_WAKE_UP_THS, (uint8_t *)&tap_ths, 1);

  lsm6dsl_set_interrupts(setEnabled);
  
  return ret;
}



int lsm6dsl_init(/*struct storage_module *storage */)
{  
  
  int ret = 0;
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
  ret |= lsm6dsl_device_id_get(&dev_ctx, &whoamI);
  

  if ( whoamI != LSM6DSL_ID ) {
    LOG_ERR("device not found");
    return -1;
  }

  /* Restore default configuration */
  ret |= lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    ret |= lsm6dsl_reset_get(&dev_ctx, &rst);
  } while (ret == 0 && rst);
//sig motion
  static uint8_t sig_motion_threasold  = 1;
  ret |= lsm6dsl_motion_sens_set(&dev_ctx, 1);  
  ret |= lsm6dsl_motion_threshold_set(&dev_ctx, &sig_motion_threasold);
//end sig motion

  ret |= lsm6dsl_fifo_mode_set(&dev_ctx, LSM6DSL_STREAM_MODE);
  ret |= lsm6dsl_xl_power_mode_set(&dev_ctx, LSM6DSL_XL_NORMAL);
  ret |= lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  ret |= lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  ret |= lsm6dsl_fifo_xl_batch_set(&dev_ctx, LSM6DSL_FIFO_XL_NO_DEC);
  ret |= lsm6dsl_fifo_gy_batch_set(&dev_ctx, LSM6DSL_FIFO_GY_DISABLE);  
  ret |= lsm6dsl_fifo_watermark_set(&dev_ctx, FIFO_BUFFER_LENGTH*fifo_pattern);
  //lsm6dsl_fifo_write_trigger_set(&dev_ctx, LSM6DSL_TRG_XL_GY_DRDY);
  ret |= lsm6dsl_den_mode_set(&dev_ctx, LSM6DSL_LEVEL_LETCHED);
  ret |= lsm6dsl_fifo_data_rate_set(&dev_ctx, LSM6DSL_FIFO_12Hz5); //12Hz5
  ret |= lsm6dsl_rounding_mode_set(&dev_ctx, LSM6DSL_ROUND_XL);
  ret |= lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_52Hz);  //52Hz
  ret |= lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_4);    //DIV_4
  ret |= lsm6dsl_set_interrupts(false);
//  lsm6dsl_tilt_sens_set(&dev_ctx, 1); 
  
  LOG_INF("settigs completed");
  return ret;
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

static int32_t tilt_has_triggered(const stmdev_ctx_t *ctx, lsm6dsl_func_src1_t *val) {
  uint32_t ret = 0;
  lsm6dsl_func_src1_t result_trig;
  //uint8_t result;
  ret |= lsm6dsl_read_reg(ctx, LSM6DSL_FUNC_SRC1, (uint8_t *)&result_trig, 1);
  *val = result_trig;
  return ret;
}

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
  
  ret |= lsm6dsl_fifo_data_level_get(&dev_ctx, &num);  
  LOG_INF("numvalues %d", (int)num);  
  if(num < 1920) {
    //wait for next interrupt to occur
    LOG_INF("buffer not filled");
    return 0;
  }
  
  if(sigMotionDetected) {
    sigMotionDetected = false;
  } else {
    lsm6dsl_config_doubletap(false);
  }

  memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));

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
  LOG_INF("average: %4.2f\t%4.2f\t%4.2f", (double) average_xl.x, (double)average_xl.y, (double)average_xl.z);

  float vector = sqrt(pow(average_xl.x,2)+pow(average_xl.y,2)+pow(average_xl.z,2));
  LOG_INF("average) = %4.2f", (double)vector);
  
  store_tracked(vector);
  memcpy(&ble_acc_last_value_buff, &vector, sizeof(float));
  k_work_submit(&work_ble_showmotion);
  return ret;
}

static void lsm6dsl_data_handler(struct k_work *work)
{  
  lsm6dsl_func_src1_t trigger_received = {0};
  int32_t ret = tilt_has_triggered(&dev_ctx, &trigger_received);
  if(ret) {
    LOG_ERR("unable to read trigger source");
    return;
  }
  // if(trigger_received.tilt_ia) {
  //   LOG_INF("TILT.IA trigger received");   
  //   blink(0);
  // }
  if(trigger_received.sign_motion_ia) {
    LOG_INF("sign_motion_ia trigger received");
    lsm6dsl_config_doubletap(true);
    sigMotionDetected = true;
    blink(1);
  }
  lsm6dsl_tap_src_t tab_detection = {0};
  ret |= lsm6dsl_tap_src_get(&dev_ctx, &tab_detection);
  if(tab_detection.tap_ia) {
    LOG_INF("doubletab detected");
    blink(2);
  }

  if(fifo_threashold_handler()) {
      LOG_ERR("error handling fifo");
  }      
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