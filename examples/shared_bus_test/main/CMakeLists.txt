idf_component_register(
    SRCS "main.c" "twai_transport.c"
    INCLUDE_DIRS "."
    REQUIRES cybergear esp_driver_twai
)
