set(component_srcs "main.c")

idf_component_register(SRCS "actuators.c" "sensors.c" "lora.c" "${component_srcs}"
                       INCLUDE_DIRS ".")


