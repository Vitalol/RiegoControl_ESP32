#include "measurements.h"

#include "esp_log.h"

int measurements_init(measure_buffer_t *buffer, measure_t *ptr, uint8_t size){
   // Start mutex
   buffer->buffer = ptr;
   buffer->mux = xSemaphoreCreateMutex();
   buffer->flag = xEventGroupCreate();
   buffer->count = 0;
   buffer->head = 0;
   buffer->tail = 0;
   buffer->size = size;
   return 1;
}

int measurements_push_measure(measure_buffer_t *buffer, measure_t measure){
   
   xSemaphoreTake(buffer->mux, portMAX_DELAY);
   int next;
   next = buffer->head + 1;      // where is head pointed after the write
   if(next >= buffer->size){
      next = 0;
   }
   if  (next == buffer->tail){  // buffer full
      ESP_LOGE(pcTaskGetName(NULL), "Circular buffer is full");
      xSemaphoreGive(buffer->mux);
      return -1;
   }
   buffer->buffer[buffer->head] = measure;
   buffer->head = next;
   xSemaphoreGive(buffer->mux);
   return 1;
}

int measurements_pop_measure(measure_buffer_t *buffer, measure_t *measure){
   int next;
   xSemaphoreTake(buffer->mux, portMAX_DELAY);
   if (buffer->head == buffer->tail){  // not data
      ESP_LOGE(pcTaskGetName(NULL), "Circular buffer is empty");
      xSemaphoreGive(buffer->mux);
      return -1;
   }
   next = buffer->tail + 1; 
   if(next >= buffer->size){
      next = 0; // roll the buffer
   }
   *measure = buffer->buffer[buffer->tail];
   buffer->tail = next;
   xSemaphoreGive(buffer->mux);
   return 1;
}

// measure notify
void measurements_notify(measure_buffer_t *buffer){
   ESP_LOGI(pcTaskGetName(NULL), "measurements notify");
   xEventGroupSetBits(buffer->flag, 0x01);
}
// measure wait
int measurements_wait(measure_buffer_t *buffer, int wait){
   xEventGroupWaitBits(buffer->flag, 0x01, pdTRUE, pdFALSE, wait);
   ESP_LOGI(pcTaskGetName(NULL), "measurements notified");
   return 1;
}