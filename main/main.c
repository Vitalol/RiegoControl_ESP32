#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "lora.h"
#include "sensors.h"
#include "node_config.h"
#include "bme280.h"
#include "bme280_port.h"
#include "measurements.h"

// define
void app_main()
{
    ESP_LOGI(pcTaskGetName(NULL), "Project Version: %d", conf_get_version());

    // Conf Node
    static node_handler_t node = {0};   //static to avoid storing it in stack
    conf_set_NodeMode(&node, SensorNode);
    conf_set_node_addr(&node, 5);
    
    
    // Measures init
    static measure_handler_t measure_handler;
    measurements_init(&measure_handler);
    
    // Node init
    switch (conf_get_NodeMode(&node))
    {
        case SensorNode:
            sensors_init_task(&measure_handler);
            break;
        case ActuatorNode:
            // ToDo:
            break;
        default:
            break;
    }
    
    // LoRa initialitation
    lora_init();
    lora_init_task(node, measure_handler);
    
}