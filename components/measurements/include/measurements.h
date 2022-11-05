#ifndef __MEASUREMENTS_H__
#define __MEASUREMENTS_H__
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/*********************************
 *
 *      Measurements file
 * 
 *********************************/



// Datatype
typedef struct measure_t{
    float value;
    uint8_t type;
}measure_t;

typedef struct measure_handler_t{
    EventGroupHandle_t flag;
    QueueHandle_t msg_queue;
}measure_handler_t;


typedef enum {
    AIR_TEMPERATURE,
    AIR_HUMIDITY,
    AIR_PRESSURE
} measure_type_t ; 


// Function headers

int measurements_init(measure_handler_t *handler);
void measurements_notify(measure_handler_t *handler);
int measurements_wait(measure_handler_t *handler, int wait);
int measurements_add(measure_handler_t *handler, measure_t *measure);
int measurements_get(measure_handler_t *handler, measure_t *measure);
int measurements_pending(measure_handler_t *handler);

#endif
