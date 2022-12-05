#ifndef __ACTUATORS_H_
#define __ACTUATORS_H_
// datatypes
typedef enum actuators_t {
    ACTUATOR_IRRIGATOR,
    ACTUATOR_UNKNOWN,
    ACTUATOR_TEST
} actuators_t;

typedef struct actuator_irrigator_t {
    int on;
    int id;
} actuator_irrigator_t;

typedef struct actuator_test_t {
    int on;
} actuator_test_t;

typedef struct actuator_handler_t {
    actuators_t type;
    int         id;
    union {
        actuator_irrigator_t irrigator;
        actuator_test_t      test;
    } actuator;

} actuator_handler_t;
// prototypes

void actuator_callback(actuator_handler_t *actuator_hndlr);
#endif
