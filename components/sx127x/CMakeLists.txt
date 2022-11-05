set(component_srcs)

idf_component_register(SRCS "sx127x.c" "${component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "include")
