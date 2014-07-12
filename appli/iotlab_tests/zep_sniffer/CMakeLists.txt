if(${PLATFORM_HAS_PHY})
    add_executable(zep_sniffer main zep_sniffer)
    target_link_libraries(zep_sniffer platform)
endif(${PLATFORM_HAS_PHY})
