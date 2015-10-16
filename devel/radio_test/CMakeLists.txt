include_directories(
     ${PROJECT_SOURCE_DIR}/drivers/cortex-m3
     ${PROJECT_SOURCE_DIR}/drivers/stm32
     ${PROJECT_SOURCE_DIR}/drivers/stm32f1xx
     )


# Build for open nodes M3
#if (PLATFORM STREQUAL "iotlab-m3")
    add_executable(radio_test main)
    target_link_libraries(radio_test platform)
#endif (PLATFORM STREQUAL "iotlab-m3")

