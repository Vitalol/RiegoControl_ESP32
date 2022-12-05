
#include "sensors.h"

#include <stdio.h>
#include <string.h>

#include "bme280.h"
#include "bme280_port.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "node_config.h"

// defines
#define TSK_SIZE_SENSOR (4 * 1024)

#define QUEUE_LENGTH_SENSOR 16
#define BME280_MEASURES_NUM 3

// Prototypes
void sensor_task(void *pvParameters);

void sensors_init_task(measure_handler_t *measure_handler) {
    xTaskCreate(&sensor_task, "tsk_sensor", TSK_SIZE_SENSOR, measure_handler, 5,
                NULL);
}

void sensor_task(void *pvParameters) {
    measure_handler_t  measure_handler = *(measure_handler_t *)pvParameters;
    struct bme280_dev  dev;
    struct bme280_data sensor_data;
    measure_t          bme280[BME280_MEASURES_NUM];

    bme280_I2C_init(&dev);

    while (true) {
        stream_sensor_data_forced_mode(&dev, &sensor_data);
        ESP_LOGI(pcTaskGetName(NULL), "Sensor data");

        bme280[0].value = sensor_data.humidity;
        bme280[0].type  = AIR_HUMIDITY;

        bme280[1].value = sensor_data.pressure;
        bme280[1].type  = AIR_PRESSURE;

        bme280[2].value = sensor_data.temperature;
        bme280[2].type  = AIR_TEMPERATURE;

        for (int i = 0; i < BME280_MEASURES_NUM; i++) {
            ESP_LOGI(pcTaskGetName(NULL), "Push value %f type %d",
                     bme280[i].value, bme280[i].type);
            measurements_add(&measure_handler, &bme280[i]);
        }
        measurements_notify(&measure_handler);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}