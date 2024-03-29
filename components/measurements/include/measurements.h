#ifndef __MEASUREMENTS_H__
#define __MEASUREMENTS_H__
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Defines
#define MAX_MEASURES_NUM   (3 * 16)
#define MEASURE_READY_FLAG 0x01

// Datatype
typedef struct measure_t {
    float   value;
    uint8_t type;
} __attribute__((__packed__)) measure_t;

typedef struct measure_handler_t {
    EventGroupHandle_t flag;
    QueueHandle_t      msg_queue;
} measure_handler_t;

typedef enum { AIR_TEMPERATURE, AIR_HUMIDITY, AIR_PRESSURE } measure_type_t;

// Function headers

int  measurements_init(measure_handler_t *handler);
void measurements_notify(measure_handler_t *handler);
int  measurements_wait(measure_handler_t *handler, int wait);
int  measurements_add(measure_handler_t *handler, measure_t *measure);
int  measurements_get(measure_handler_t *handler, measure_t *measure);
int  measurements_pending(measure_handler_t *handler);

#endif
