idf_component_register(
    SRCS 
    "main.c" 
    "test_code.c"
    "../system/uart_bus.c"
    "../connect/uart_cmd.c"
    "../device/a7129.c"
    "../device/ybl.c"

    INCLUDE_DIRS 
    ""
    "../device"
    "../system"
    "../connect"
    
)
