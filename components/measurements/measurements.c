#include "measurements.h"

#include "esp_log.h"

#define MEASURE_QUEUE_LENGTH 16

int measurements_init(measure_handler_t *handler) {
    handler->flag      = xEventGroupCreate();
    handler->msg_queue = xQueueCreate(MEASURE_QUEUE_LENGTH, sizeof(measure_t));
    return 0;
}
int measurements_add(measure_handler_t *handler, measure_t *measure) {
    if (xQueueSend(handler->msg_queue, measure, 0) == pdTRUE) {
        return 0;
    }
    return -1;
}

int measurements_get(measure_handler_t *handler, measure_t *measure) {
    int pending = measurements_pending(handler);
    if (!pending) {
        return -1;
    }
    xQueueReceive(handler->msg_queue, measure, portMAX_DELAY);
    return pending;
}

int measurements_pending(measure_handler_t *handler) {
    return uxQueueMessagesWaiting(handler->msg_queue);
}

// measure notify
void measurements_notify(measure_handler_t *handler) {
    ESP_LOGI(pcTaskGetName(NULL), "measurements notify");
    xEventGroupSetBits(handler->flag, 0x01);
}
// measure wait
int measurements_wait(measure_handler_t *handler, int wait) {
    xEventGroupWaitBits(handler->flag, 0x01, pdTRUE, pdFALSE, wait);
    ESP_LOGI(pcTaskGetName(NULL), "measurements notified");
    return 0;
}