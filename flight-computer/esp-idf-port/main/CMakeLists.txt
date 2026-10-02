idf_component_register(SRCS "airbrakes/airbrakes.cpp" 
        "computer/computer.cpp"
        "control/control.cpp" "sd.cpp" "main.cpp"
        "gps/gps.cpp"
        "gps/pa1616d.cpp"
        "gps/parsing.cpp"
        "i2c/I2C.cpp"
        "i2c/TMP1075.cpp"
        "i2c/high_g_accel.cpp"
        "i2c/segment7.cpp"
        "i2c/BMP581.cpp"
        "i2c/BMI323.cpp"
        "i2c/MLX90395.cpp"
        "integration.cpp"
        "motor/motor.cpp"
        "rotation/rotation.cpp"
        "sd.cpp" 
        "staging/staging.cpp"
        PRIV_REQUIRES esp_driver_ledc spi_flash esp_driver_i2c esp_timer fatfs sdmmc esp_driver_sdspi esp_driver_mcpwm esp_driver_gpio esp_driver_uart
        INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=missing-field-initializers)