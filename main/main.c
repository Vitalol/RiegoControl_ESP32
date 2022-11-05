#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "sx127x.h"
#include "node_config.h"
#include "bme280.h"
#include "bme280_port.h"
#include "measurements.h"

// defines
#define TSK_SIZE_SENSOR (4 * 1024)
#define TSK_SIZE_RX     (4 * 1024)
#define QUEUE_LENGTH_SENSOR 16
#define BME280_MEASURES_NUM     3
#define MAX_MEASURES_NUM    3*16

// prototypes
void tsk_sensor(void *pvParameters);
void task_rx(void *pvParameters);
void task_lora_comm(void *pvParameters);
// Queues
QueueHandle_t queue_sensor;
measure_handler_t measure_handler;





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
    Node_handler_t node = {0};
    conf_set_NodeMode(&node, SensorNode);
    // Measures init
    measurements_init(&measure_handler);
    switch (conf_get_NodeMode(&node))
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
    measure_t measures_list[MAX_MEASURES_NUM];
    measure_t bme280[BME280_MEASURES_NUM];

    bme280_I2C_init(&dev);

    while (true)
    {
        stream_sensor_data_forced_mode(&dev, &sensor_data);
        print_sensor_data(&sensor_data);
        ESP_LOGI(pcTaskGetName(NULL), "Sensor data");

        bme280[0].value = sensor_data.humidity;
        bme280[0].type = AIR_HUMIDITY;

        bme280[1].value = sensor_data.pressure;
        bme280[1].type = AIR_PRESSURE;

        bme280[2].value = sensor_data.temperature;
        bme280[2].type = AIR_TEMPERATURE;
        
        for(int i = 0; i < BME280_MEASURES_NUM; i++){
            ESP_LOGI(pcTaskGetName(NULL), "Push value %f type %d", bme280[i].value, bme280[i].type);
            measurements_add(&measure_handler, &bme280[i]);
        }
        measurements_notify(&measure_handler);
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
    ESP_LOGI(pcTaskGetName(NULL), "Start task_lora_comm");
    
    measure_t measure;
    while (1)
    {   
        ESP_LOGI(pcTaskGetName(NULL), "Waiting measures ...");
        measurements_wait(&measure_handler, portMAX_DELAY);
        while(measurements_get(&measure_handler, &measure) != 0){
            ESP_LOGI(pcTaskGetName(NULL), "value %f  type %d", measure.value, measure.type);
        }
        //lora_send_packet((uint8_t *)(&measure), sizeof(measure_struct));
    } // end while
}