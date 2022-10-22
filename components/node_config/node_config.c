#include "node_config.h"

/* Config variables */
int project_version = 0;

NodeConfig_t NodeConfiguration;

int conf_get_version(void){
   return project_version;
}

NodeMode_t conf_get_NodeMode(void){
   return NodeConfiguration.mode;
}

void conf_set_NodeMode(NodeMode_t mode){
   NodeConfiguration.mode = mode;
};