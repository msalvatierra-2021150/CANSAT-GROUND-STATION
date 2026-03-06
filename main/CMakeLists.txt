idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES RadioLib esp_driver_gpio esp_timer) # This is the missing link!
