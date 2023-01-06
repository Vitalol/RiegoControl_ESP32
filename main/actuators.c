#include "actuators.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define NUMBER_OF_IRRIGATORS 1
#define LOG_TAG              "CALLBACK"
// This array purpouse is to extend the number of irrigators
// if it's needed in the future
gpio_num_t irrigators_map[NUMBER_OF_IRRIGATORS] = {GPIO_NUM_25};

gpio_num_t actuator_map(actuator_handler_t *actuator_hndlr) {
    switch (actuator_hndlr->type) {
        case ACTUATOR_IRRIGATOR:

            return irrigators_map[actuator_hndlr->actuator.irrigator.id - 1];
            break;

        default:
            return GPIO_NUM_NC;
            break;
    }
    return GPIO_NUM_NC;
}

int actuator_irrigation(actuator_handler_t *actuator_hndlr) {
    gpio_num_t gpio_pin = actuator_map(actuator_hndlr);
    if (gpio_pin == GPIO_NUM_NC) {
        return -1;
    }
    gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
    ESP_LOGW(LOG_TAG, " Led %d", actuator_hndlr->actuator.irrigator.on);
    return gpio_set_level(gpio_pin, actuator_hndlr->actuator.irrigator.on);
}

void actuator_callback_default(actuator_handler_t *actuator_hndlr) {
    ESP_LOGW(LOG_TAG, " No callback defined for actuator %d",
             actuator_hndlr->actuator.irrigator.id);
}
void actuator_callback_irrigation(actuator_handler_t *actuator_hndlr) {
    actuator_irrigation(actuator_hndlr);
}

void actuator_callback(actuator_handler_t *actuator_hndlr) {
    switch (actuator_hndlr->type) {
        case ACTUATOR_IRRIGATOR:
            actuator_callback_irrigation(actuator_hndlr);
            break;

        default:
            break;
    }
}