idf_component_register(SRCS main.c  
    INCLUDE_DIRS ""
    REQUIRES cybergear esp_driver_twai
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=cpp)
