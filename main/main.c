// Compiler libs
#include <stdio.h>
#include <string.h>
// Espressif and freertos libs
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// my libs
#include "actuators.h"
#include "bme280.h"
#include "bme280_port.h"
#include "lora.h"
#include "measurements.h"
#include "node_config.h"
#include "scheduler.h"
#include "sensors.h"

// define
void callback(void *parameters) {
    int args = *(int *)parameters;
    ESP_LOGW("CALLBACK", " CALLBAAAAAAAAAAAAAAACK %d \n", args);
}
void app_main() {
    ESP_LOGI(pcTaskGetName(NULL), "Project Version: %d", conf_get_version());

    // Conf Node
    static node_handler_t node = {0};  // static to avoid losing scope
    conf_set_NodeMode(&node, SensorNode);
    conf_set_node_addr(&node, 5);

    // Measures init
    static measure_handler_t measure_hndl = {
        0};  // static to avoid losing scope
    measurements_init(&measure_hndl);

    // Node init
    switch (conf_get_NodeMode(&node)) {
        case SensorNode:
            sensors_init_task(&measure_hndl);
            break;
        case ActuatorNode:
            // ToDo:
            break;
        default:
            break;
    }

    // LoRa initialitation
    lora_init();
    lora_init_task(node, measure_hndl);

    // Test scheduler

    static scheduler_handler_t scheduler = {0};
    scheduler.granularity                = 10;

    scheduler_init(&scheduler);

    scheduler_dates_t date1 = {.hour       = 0,
                               .minute     = 0,
                               .month_days = 1 << (1 - 1),
                               .week_days  = 1 << 6};
    scheduler_add_schedule(&scheduler, date1, 1, callback);

    scheduler_dates_t date2 = {.hour       = 0,
                               .minute     = 1,
                               .month_days = 1 << (1 - 1),
                               .week_days  = 1 << 6};
    scheduler_add_schedule(&scheduler, date2, 2, callback);

    scheduler_dates_t date3 = {.hour       = 0,
                               .minute     = 3,
                               .month_days = 2 << (1 - 1),
                               .week_days  = 1 << (4 - 1)};
    scheduler_add_schedule(&scheduler, date3, 3, callback);

    actuator_handler_t actuator = {.type                  = ACTUATOR_IRRIGATOR,
                                   .actuator.irrigator.id = 1};
    for (int i = 0; i < 5; i++) {
        actuator_irrigation(actuator, 1);
        vTaskDelay(500 / portTICK_RATE_MS);
        actuator_irrigation(actuator, 0);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}