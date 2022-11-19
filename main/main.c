#include <stdio.h>
#include <string.h>

#include "bme280.h"
#include "bme280_port.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "measurements.h"
#include "node_config.h"
#include "scheduler.h"
#include "sensors.h"

// define
void callback(void) { ESP_LOGW("CALLBACK", " CALLBAAAAAAAAAAAAAAACK \n"); }
void app_main() {
    ESP_LOGI(pcTaskGetName(NULL), "Project Version: %d", conf_get_version());

    // Conf Node
    static node_handler_t node = {0};  // static to avoid storing it in stack
    conf_set_NodeMode(&node, SensorNode);
    conf_set_node_addr(&node, 5);
    /*
        // Measures init
        static measure_handler_t measure_handler;
        measurements_init(&measure_handler);

        // Node init
        switch (conf_get_NodeMode(&node)) {
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
    */
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
}