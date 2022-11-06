#ifndef __LORA_H__
#define __LORA_H__

#include "measurements.h"
#include "node_config.h"

typedef enum msg_protocol_t{
    MSG_PROTOCOL_SEND_MEASURES
}msg_protocol_t;

void lora_init(void);
void lora_task_com(void *pvParameters);
void lora_init_task(node_handler_t node_h, measure_handler_t measure_h);
#endif