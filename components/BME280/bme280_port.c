#include "bme280_port.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "bme280.h"
#include "bme280_defs.h"
#include "driver/i2c.h"
#include "esp32/rom/ets_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO \
    GPIO_NUM_22 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO \
    GPIO_NUM_21 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM                                                      \
    I2C_NUM_0 /*!< I2C master i2c port number, the number of i2c peripheral \
                 interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ        1000000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS     1000

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief This API reads the sensor temperature, pressure and humidity data in
 * forced mode.
 */
int8_t stream_sensor_data_forced_mode(struct bme280_dev  *dev,
                                      struct bme280_data *comp_data) {
    int8_t  rslt;
    uint8_t settings_sel;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h  = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p  = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t  = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL |
                   BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_OK) {
        fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);

        return rslt;
    }

    /* Continuously stream sensor data */

    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
    if (rslt != BME280_OK) {
        fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
    }

    /* Wait for the measurement to complete and print data @25Hz */
    dev->delay_us(40000, dev->intf_ptr);
    rslt = bme280_get_sensor_data(BME280_ALL, comp_data, dev);
    if (rslt != BME280_OK) {
        fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
    }
    dev->delay_us(1000000, dev->intf_ptr);

    return rslt;
}

/*!
 * @brief This API used to print the sensor temperature, pressure and humidity
 * data.
 */
void print_sensor_data(struct bme280_data *comp_data) {
    float temp, press, hum;
    printf("Temperature, Pressure, Humidity\n");
#ifdef BME280_FLOAT_ENABLE
    temp  = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum   = comp_data->humidity;
#else
#ifdef BME280_64BIT_ENABLE
    temp  = 0.01f * comp_data->temperature;
    press = 0.0001f * comp_data->pressure;
    hum   = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp  = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum   = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
}

/*!
 * I2C read function map to ESP32 platform
 */

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length,
                     void *intf_ptr) {
    int8_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (length > 1) {
        i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    return espRc;
}

/*!
 * I2C write function map to ESP32 platform
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                      uint32_t length, void *intf_ptr) {
    esp_err_t        espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, length, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    return espRc;
}

/*!
 * Delay function map to ESP32 platform
 */
void user_delay_us(uint32_t period, void *intf_ptr) { ets_delay_us(period); }

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bme280_I2C_init(struct bme280_dev *dev) {
    // struct bme280_dev dev;
    int8_t rslt = 1;
    dev_addr    = BME280_I2C_ADDR_SEC;

    dev->intf_ptr = &dev_addr;
    dev->intf     = BME280_I2C_INTF;
    dev->read     = user_i2c_read;
    dev->write    = user_i2c_write;
    dev->delay_us = user_delay_us;

    // init i2c
    int          i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf            = {
                   .mode             = I2C_MODE_MASTER,
                   .sda_io_num       = I2C_MASTER_SDA_IO,
                   .scl_io_num       = I2C_MASTER_SCL_IO,
                   .sda_pullup_en    = GPIO_PULLUP_ENABLE,
                   .scl_pullup_en    = GPIO_PULLUP_ENABLE,
                   .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t paramconfig  = i2c_param_config(i2c_master_port, &conf);
    esp_err_t driv_install = i2c_driver_install(i2c_master_port, conf.mode,
                                                I2C_MASTER_RX_BUF_DISABLE,
                                                I2C_MASTER_TX_BUF_DISABLE, 0);
    uint8_t   chip_id      = 0;

    bme280_soft_reset(dev);
    bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1, dev);

    rslt = bme280_init(dev);

    return rslt;
}

/*!
 *  @brief Function deinitializes I2C platform.
 */
void bme280_I2C_deinit(void) {
    // not implemented
}
