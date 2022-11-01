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



// Defines
#define MAX_MEASURES_NUM 255

// Datatype
typedef struct measure_t{
    float value;
    uint8_t type;
}measure_t;

typedef struct measure_buffer_t{
    measure_t *  buffer;
    SemaphoreHandle_t mux;
    EventGroupHandle_t flag;
    uint8_t head;
    uint8_t tail;
    uint8_t size;
    uint8_t count;
}measure_buffer_t;


typedef enum {
    AIR_TEMPERATURE,
    AIR_HUMIDITY,
    AIR_PRESSURE
} measure_type_t ; 


// Function headers
int measurements_init(measure_buffer_t *buffer, measure_t *ptr, uint8_t size);
int measurements_push_measure(measure_buffer_t *buffer, measure_t measure);
int measurements_pop_measure(measure_buffer_t *buffer, measure_t *measure);

void measurements_notify(measure_buffer_t *buffer);
int measurements_wait(measure_buffer_t *buffer, int wait);

// Macros

#define MEASUREMENTS_INIT_BUFFER(name, size) \
    measure_t name##_data[size];\
    measurements_init(&name, name##_data, size);


#endif
