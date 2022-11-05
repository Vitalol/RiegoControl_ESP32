#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <stdint.h>
/*********************************
 *
 *      Configuration file
 * 
 *********************************/

/* Types */

typedef enum {
    SensorNode,
    ActuatorNode
}NodeMode_t;

/* Structs */

typedef struct Node_handler_t
{
    NodeMode_t mode;
}Node_handler_t;



/**
 * @brief Project version
 * 
 * @return int project version value
 */
int conf_get_version(void);
/**
 * @brief Return the node working mode 
 * 
 * @return NodeMode, sensor or actuaro mode
 */

/**
 * @brief Return the node working mode
 * 
 * @param NodeConfiguration node handler
 * @return NodeMode_t sensor or actuator
 */
NodeMode_t conf_get_NodeMode(Node_handler_t *NodeConfiguration);

void conf_set_NodeMode(Node_handler_t *NodeConfiguration, NodeMode_t mode);

#endif
