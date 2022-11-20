#include "actuators.h"

#include "driver/gpio.h"

#define IRRIGATORS_NUM 1

// This array purpouse is to extend the number of irrigators
// if it's needed in the future
gpio_num_t irrigators_map[IRRIGATORS_NUM] = {GPIO_NUM_25};

gpio_num_t actuator_map(actuator_handler_t actuator_hdlr) {
    switch (actuator_hdlr.type) {
        case ACTUATOR_IRRIGATOR:

            return irrigators_map[actuator_hdlr.actuator.irrigator.id - 1];
            break;

        default:
            return GPIO_NUM_NC;
            break;
    }
    return GPIO_NUM_NC;
}

int actuator_irrigation(actuator_handler_t actuator_hdlr, int on) {
    gpio_num_t gpio_pin = actuator_map(actuator_hdlr);
    if (gpio_pin == GPIO_NUM_NC) {
        return -1;
    }
    gpio_set_direction(actuator_map(actuator_hdlr), GPIO_MODE_OUTPUT);
    return gpio_set_level(GPIO_NUM_25, on);
}