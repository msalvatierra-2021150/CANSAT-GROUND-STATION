idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES jgromes__radiolib esp_driver_gpio esp_driver_spi esp_timer)