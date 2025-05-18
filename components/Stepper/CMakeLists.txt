idf_component_register(SRCS "AccelStepper.cpp" "MultiStepper.cpp"
                    PRIV_REQUIRES esp_driver_gpio esp_timer
                    INCLUDE_DIRS "include/" ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=uxTopUsedPriority")
set(CMAKE_CXX_STANDARD 17)
