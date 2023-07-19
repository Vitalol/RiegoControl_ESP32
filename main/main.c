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
    ESP_LOGW("CALLBACK", " CALLBAAAAAAAAAAAAAAACK %d", args);
}
void app_main() {
    ESP_LOGI(pcTaskGetName(NULL), "Project Version: %d", conf_get_version());

    // Conf Node
    static node_handler_t node = {0};  // static to avoid losing scope
    conf_set_NodeMode(&node, SensorNode);
    conf_set_node_addr(&node, 5);

    // Measures init
    static measure_handler_t measure_hndl = {0};  // static to avoid losing scope
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

    // Lora init
    lora_init_task(node, measure_hndl);

    // Test scheduler

    static scheduler_handler_t scheduler = {0};

    scheduler_init(&scheduler);

    // Add schedules
    int duration = 1;
    // turn on
    scheduler_dates_t turn_on = {.hour       = 0,
                                 .minute     = 1,
                                 .month_days = 1 << (1 - 1),
                                 .week_days  = 1 << 6};

    static actuator_handler_t turn_on_hndl = {.type                  = ACTUATOR_IRRIGATOR,
                                              .actuator.irrigator.id = 1,
                                              .actuator.irrigator.on = 1};

    scheduler_add_schedule(&scheduler, turn_on, 1, actuator_callback,
                           (void *)&turn_on_hndl);

    // turn off
    scheduler_dates_t turn_off = turn_on;
    turn_off.minute            = turn_on.minute + duration;

    static actuator_handler_t turn_off_hndl = {.type                  = ACTUATOR_IRRIGATOR,
                                               .actuator.irrigator.id = 1,
                                               .actuator.irrigator.on = 0};

    scheduler_add_schedule(&scheduler, turn_off, 2, actuator_callback,
                           (void *)&turn_off_hndl);
}