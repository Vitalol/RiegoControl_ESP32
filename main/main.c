#include <stdio.h>
#include <string.h>

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
#define TSK_SIZE_RX     (10 * 1024)
#define QUEUE_LENGTH_SENSOR 16
#define BME280_MEASURES_NUM     3
#define MAX_MEASURES_NUM    3*16
#define LORA_MAX_PKT_SIZE 255

#define LORA_HEADER_SIZE 4
#define LORA_SINK_ADDR    0
#define LORA_RECIVER_INDX 0
#define LORA_SENDER_INDX  1
#define LORA_MSG_PROTOCOL_INDX 2
#define LORA_SIZE_INDX 3

// prototypes
void tsk_sensor(void *pvParameters);
void task_rx(void *pvParameters);
void task_lora_comm(void *pvParameters);
// Queues
QueueHandle_t queue_sensor;
measure_handler_t measure_handler;

// enums

typedef enum msg_protocol_t{
    MSG_PROTOCOL_SEND_MEASURES
}msg_protocol_t;


Node_handler_t node = {0};  


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
    
    // conf Node
    //static Node_handler_t node = {0};   //static to avoid storing it in stack
    conf_set_NodeMode(&node, SensorNode);
    conf_set_node_addr(&node, 5);
    
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
    xTaskCreate(&task_lora_comm, "task_lora_comm", TSK_SIZE_RX, (void *) &node, 5, NULL);
}

void tsk_sensor(void *pvParameters)
{
    struct bme280_dev dev;
    struct bme280_data sensor_data;
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
    Node_handler_t node = *(Node_handler_t *)pvParameters;
    ESP_LOGI(pcTaskGetName(NULL), "Start task_lora_comm");
    measure_t measure;
    uint8_t lora_pkt[LORA_MAX_PKT_SIZE];
    int lora_pkt_indx = 0;
    char lora_pkt_string[1024] = {0};

    msg_protocol_t msg_to_send = MSG_PROTOCOL_SEND_MEASURES;
    while (1)
    {   
        ESP_LOGI(pcTaskGetName(NULL), "Waiting measures ...");
        lora_pkt_indx = 0;
        switch(msg_to_send){
            case MSG_PROTOCOL_SEND_MEASURES:
                // send measurements
                measurements_wait(&measure_handler, portMAX_DELAY);
                int measures_count = 0;
                measure_t measures_buff[MAX_MEASURES_NUM];
                // Header
                lora_pkt[LORA_RECIVER_INDX] = LORA_SINK_ADDR;
                lora_pkt_indx++;
                lora_pkt[LORA_SENDER_INDX] = conf_get_node_addr(node);
                lora_pkt_indx++;
                // Measures

                while(measurements_get(&measure_handler, &measure) != 0){
                    measures_buff[measures_count++] = measure;
                }
                int bytes_num = measures_count*sizeof(measure_t);
                lora_pkt[LORA_SIZE_INDX] = bytes_num;
                lora_pkt_indx++;
                lora_pkt[LORA_MSG_PROTOCOL_INDX] = MSG_PROTOCOL_SEND_MEASURES;
                lora_pkt_indx++;
                int offset = 0;

                memcpy((void *)&lora_pkt[lora_pkt_indx], (void *) &measures_buff, bytes_num);
                
                
                for(int i = 0; i<(bytes_num+LORA_HEADER_SIZE); i++){
                    offset += sprintf(&lora_pkt_string[offset], "%02X ", lora_pkt[i]);
                }

                ESP_LOGI(pcTaskGetName(NULL), "lora_pkt(4+%d) = %s",bytes_num, lora_pkt_string);
                lora_send_packet(lora_pkt, bytes_num+LORA_HEADER_SIZE);

            break;
        }

    } // end while
}