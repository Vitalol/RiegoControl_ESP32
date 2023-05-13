#ifndef __LORA_H__
#define __LORA_H__

#include "measurements.h"
#include "node_config.h"

void lora_init_task(node_handler_t node_h, measure_handler_t measure_h);
#endif