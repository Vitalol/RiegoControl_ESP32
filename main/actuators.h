#ifndef __ACTUATORS_H_
#define __ACTUATORS_H_
// datatypes
typedef enum actuators_t {
    ACTUATOR_IRRIGATOR,
    ACTUATOR_UNKNOWN,
    ACTUATOR_TEST
} actuators_t;

typedef struct actuator_irrigator_t {
    int id;
} actuator_irrigator_t;

typedef struct actuator_test_t {
    int id;
} actuator_test_t;

typedef struct actuator_handler_t {
    actuators_t type;
    union {
        actuator_irrigator_t irrigator;
        actuator_test_t      test;
    } actuator;

} actuator_handler_t;
// prototypes
int actuator_irrigation(actuator_handler_t actuator_hdlr, int on);
#endif
