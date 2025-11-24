# main/CMakeLists.txt
# idf_component_register(SRCS "main.c"
#                        INCLUDE_DIRS "."
#                        PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_http_server esp_wifi nvs_flash
#                        REQUIRES esp32-camera esp_http_server esp_wifi nvs_flash)
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES 
                           esp_driver_gpio 
                           esp_driver_ledc 
                           esp_http_server 
                           esp_wifi 
                           nvs_flash
                           esp_netif
                           esp_event
                           esp_timer  # <-- ADD THIS LINE
)
