#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
/*********************************
 *
 *      SCHEDULER file
 *
 *********************************/

// defines
#define SCHEDULES_MAX_NUM (32)

// datatypes
typedef struct scheduler_dates_t {
    uint32_t month_days;  // Wich days of the month
    uint8_t  week_days;   // Wich days of the week
    uint8_t  hour;        // At wich hour
    uint8_t  minute;      // At wich minute
} scheduler_dates_t;

typedef struct scheduler_schedule_t {
    bool              active;
    scheduler_dates_t date;
    void (*callback)(void *parameters);
    void *arguments;
} scheduler_schedule_t;

typedef struct scheduler_handler_t {
    scheduler_schedule_t schedules_list[SCHEDULES_MAX_NUM];
    SemaphoreHandle_t    mutex;
} scheduler_handler_t;

// prototypes
int scheduler_add_schedule(scheduler_handler_t *sch_hndl,
                           scheduler_dates_t date, int              id,
                           void (*callback)(void *arguments), void *arguments);
int scheduler_remove_schedule(scheduler_handler_t *sch_hndl, int id);
int scheduler_init(scheduler_handler_t *sch_hndl);
#endif
