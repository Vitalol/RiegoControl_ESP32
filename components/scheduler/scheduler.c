#include "scheduler.h"

#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_sntp.h"

#define SCHEDULER_TSK_SIZE    (10 * 1024)
#define SCHEDULER_GRANULARITY (30 * 1000)  // 30 seconds

void scheduler_mutex_take(scheduler_handler_t *sch_hndl, uint32_t wait) {
    xSemaphoreTake((sch_hndl->mutex), wait);
}

void scheduler_mutex_give(scheduler_handler_t *sch_hndl) {
    xSemaphoreGive((sch_hndl->mutex));
}
int scheduler_add_schedule(scheduler_handler_t *sch_hndl,
                           scheduler_dates_t date, int id,
                           void (*callback)(void *parameters)) {
    scheduler_mutex_take(sch_hndl, portMAX_DELAY);

    if (id > SCHEDULES_MAX_NUM) {
        return -1;
    }
    scheduler_schedule_t schedule = {
        .active = 1, .lock = 0, .date = date, .callback = callback};
    sch_hndl->schedules_list[id] = schedule;
    scheduler_mutex_give(sch_hndl);

    return 0;
}

int scheduler_remove_schedule(scheduler_handler_t *sch_hndl, int id) {
    scheduler_mutex_take(sch_hndl, portMAX_DELAY);

    if (id > SCHEDULES_MAX_NUM) {
        return -1;
    }
    scheduler_schedule_t schedule = {0};
    sch_hndl->schedules_list[id]  = schedule;
    scheduler_mutex_give(sch_hndl);

    return 0;
}

int scheduler_is_now(scheduler_dates_t date, struct tm *time_now) {
    return (time_now->tm_hour == date.hour && time_now->tm_min == date.minute &&
            (((1 << (time_now->tm_wday - 1)) & date.week_days) ||
             ((1 << (time_now->tm_mday - 1)) & date.month_days)));
}

void scheduler_task(void *pvParameters) {
    // init

    scheduler_handler_t *sch_hndl = (scheduler_handler_t *)pvParameters;
    bool                 scheduler_running = true;
    // loop
    while (scheduler_running) {
        scheduler_mutex_take(sch_hndl, portMAX_DELAY);

        for (int sch_indx = 0; sch_indx < SCHEDULES_MAX_NUM; sch_indx++) {
            if (sch_hndl->schedules_list[sch_indx].active == 0) {
                continue;
            }

            time_t     rawtime;
            struct tm *time_now;
            time(&rawtime);
            time_now = localtime(&rawtime);

            if (scheduler_is_now(sch_hndl->schedules_list[sch_indx].date,
                                 time_now)) {
                if (!sch_hndl->schedules_list[sch_indx].lock) {
                    sch_hndl->schedules_list[sch_indx].lock = 1;
                    sch_hndl->schedules_list[sch_indx].callback(&sch_indx);
                }
            }
        }
        scheduler_mutex_give(sch_hndl);

        vTaskDelay((sch_hndl->granularity * 1000) / portTICK_RATE_MS);
    }
}
/**
 * @brief
 *
 * @param sch_hndl must not exit scope
 * @return int
 */
int scheduler_init(scheduler_handler_t *sch_hndl) {
    sch_hndl->mutex = 0;
    sch_hndl->mutex = xSemaphoreCreateMutex();

    return xTaskCreate(&scheduler_task, "scheduler_task", SCHEDULER_TSK_SIZE,
                       (void *)sch_hndl, 5, NULL);
}
