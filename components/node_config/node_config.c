#include "node_config.h"

/* Config variables */
static int project_version = 0;

int conf_get_version(void){
   return project_version;
}

NodeMode_t conf_get_NodeMode(Node_handler_t *NodeConfiguration){
   return NodeConfiguration->mode;
}

void conf_set_NodeMode(Node_handler_t *NodeConfiguration, NodeMode_t mode){
   NodeConfiguration->mode = mode;
};