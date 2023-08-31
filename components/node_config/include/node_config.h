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

typedef struct node_handler_t
{
    NodeMode_t mode;
    uint8_t addr;
}node_handler_t;



/**
 * @brief Project version
 * 
 * @return int project version value
 */
int conf_get_version(void);


/**
 * @brief Return the node working mode
 * 
 * @param Node node handler
 * @return NodeMode_t sensor or actuator
 */
NodeMode_t conf_get_NodeMode(node_handler_t *Node);

void conf_set_NodeMode(node_handler_t *Node, NodeMode_t mode);

int conf_get_node_addr(node_handler_t Node);
void conf_set_node_addr(node_handler_t *Node, uint8_t addr);

#endif
