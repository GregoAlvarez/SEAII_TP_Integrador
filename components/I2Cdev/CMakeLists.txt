idf_component_register(SRCS "I2Cdev.cpp"
                       INCLUDE_DIRS "."
		       REQUIRES "driver" "esp_timer" "freertos" "log"
)