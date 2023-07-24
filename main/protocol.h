#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <time.h>

#include "measurements.h"
#include "scheduler.h"

#define PROTOCOL_NONE                 0
#define PROTOCOL_MSG_SET_TIME         1
#define PROTOCOL_MSG_DEFAULT_RULE     2
#define PROTOCOL_MSG_SET_SCHEDULER    3
#define PROTOCOL_MSG_SEND_MEASURE     4
#define PROTOCO_MSG_MANUAL_ACTIVATION 5
#define PROTOCOL_MSG_ACTUATION_RULE   6
#define PROTOCOL_MSG_SET_RULE         7

// Rule struct (move to another module?)
typedef struct rule_str {
    uint8_t type;
    float   value;
    uint8_t rule;  // 0 Larger or equal 1 Le
} __attribute__((__packed__)) rule_str;

typedef struct protocol_header_str {
    uint8_t destination;
    uint8_t origin;
    uint8_t type;
    uint8_t length;
} __attribute__((__packed__)) protocol_header_str;

// Set hour
// Size 8U
typedef struct protocol_set_hour_str {
    protocol_header_str header;
    time_t              time;
} __attribute__((__packed__)) protocol_set_hour_str;

// default rule and set scheduler
// Size 8U
typedef struct protocol_set_scheduler_str {
    protocol_header_str header;
    uint8_t             actuatorID;
    scheduler_dates_t   schedule;
} __attribute__((__packed__)) protocol_set_scheduler_str;

typedef struct protocol_set_rule_str {
    protocol_header_str header;
    uint8_t             actuatorID;
    rule_str            rule;
} __attribute__((__packed__)) protocol_set_rule_str;

// Send MEASURE

typedef struct protocol_send_measure_str {
    protocol_header_str header;
    uint8_t             measures_num;
    measure_t           measures[MAX_MEASURES_NUM];
} __attribute__((__packed__)) protocol_send_measure_str;

// Manual activation

typedef struct protocol_manual_activation {
    protocol_header_str header;
    uint8_t             actuatorID;
    uint8_t             duration;
} __attribute__((__packed__)) protocol_manual_activation;

#endif
