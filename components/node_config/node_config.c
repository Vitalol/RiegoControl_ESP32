#include "node_config.h"

/* Config variables */
static int project_version = 0;

int conf_get_version(void){
   return project_version;
}

NodeMode_t conf_get_NodeMode(node_handler_t *Node){
   return Node->mode;
}

void conf_set_NodeMode(node_handler_t *Node, NodeMode_t mode){
   Node->mode = mode;
};

int conf_get_node_addr(node_handler_t Node){
   return Node.addr;
}

void conf_set_node_addr(node_handler_t *Node, uint8_t addr){
   Node->addr = addr;
}