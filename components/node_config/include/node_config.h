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

typedef struct NodeConfig_t
{
    NodeMode_t mode;
}NodeConfig_t;



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
NodeMode_t conf_get_NodeMode(void);
/**
 * @brief Set the node working mode
 * 
 */
void conf_set_NodeMode(NodeMode_t mode);

#endif
