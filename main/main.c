#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"
#include "node_config.h"
#include "bme280.h"
#include "bme280_port.h"
#include "driver/i2c.h"

#define TSK_SIZE_SENSOR (2 * 1024)
#define TSK_SIZE_RX     (4 * 1024)
#define QUEUE_LENGTH_SENSOR 10

// prototypes
void tsk_sensor(void *pvParameters);
void task_rx(void *pvParameters);
void task_lora_comm(void *pvParameters);
// Queues
QueueHandle_t queue_sensor;


void app_main()
{

    ESP_LOGI(pcTaskGetName(NULL), "Project Version: %d", conf_get_version());

    // LoRa initialitation
    // Recognize module
    do{
        ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }while((lora_init() == 0));

    // Lora configuration
    lora_enable_crc();
    LoraConf_t lora_configuration;
    lora_configuration.frequency = FREQ_915MHz;
    lora_configuration.codingRate = 5;
    lora_configuration.bandwith = 7;
    lora_configuration.spreadingFactor = 7;
    lora_config(lora_configuration);
    conf_set_NodeMode(SensorNode);
    
    // Queues initialitation
    queue_sensor = xQueueCreate(QUEUE_LENGTH_SENSOR, sizeof(struct bme280_data));
    // Task initialitation

    switch (conf_get_NodeMode())
    {
        case SensorNode:
            xTaskCreate(&tsk_sensor, "tsk_sensor", TSK_SIZE_SENSOR, NULL, 5, NULL);
            break;
        case ActuatorNode:
            // ToDo:
            break;
        default:
            break;
    }
    // Lora task initialitation
    xTaskCreate(&task_lora_comm, "task_lora_comm", TSK_SIZE_RX, NULL, 5, NULL);
}

void tsk_sensor(void *pvParameters)
{
    struct bme280_dev dev;
    struct bme280_data sensor_data;
    bme280_I2C_init(&dev);

    while (true)
    {
        stream_sensor_data_forced_mode(&dev, &sensor_data);
        //print_sensor_data(&data);
        ESP_LOGI(pcTaskGetName(NULL), "Sensor data");
        xQueueSend(queue_sensor, (void *)&sensor_data, 1000 / portTICK_RATE_MS);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255
    while (1)
    {
        lora_receive(); // put into receive mode
        if (lora_received())
        {
            int receive_len = lora_receive_packet(buf, sizeof(buf));
            for (int i = 0; i < receive_len; i++)
            {
                printf("%.2X ", buf[i]);
            }
            printf("\n");
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]", receive_len, receive_len, buf);
        }              // end if
        vTaskDelay(2); // Avoid WatchDog alerts
    }                  // end while
}
void task_lora_comm(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255
    int count = 0;
    struct bme280_data sensor_data;
    while (1)
    {
        xQueueReceive(queue_sensor, &sensor_data, portMAX_DELAY);
        print_sensor_data(&sensor_data);
        int send_len = sprintf((char *)buf, "%.02f %.02f %.02f %d", sensor_data.humidity, sensor_data.pressure, sensor_data.temperature, count++);
        lora_send_packet(buf, send_len);
        ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
        vTaskDelay(4000 / portTICK_RATE_MS);
    } // end while
}