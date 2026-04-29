idf_component_register(
    SRCS main.c
    INCLUDE_DIRS .
    REQUIRES json esp_driver_uart nvs_flash waveshare__esp32_p4_wifi6_touch_lcd_7b
)

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(
    ${LVGL_LIB} 
    PRIVATE 
        -DLV_LVGL_H_INCLUDE_SIMPLE
        -DLV_USE_DEMO_MUSIC
)