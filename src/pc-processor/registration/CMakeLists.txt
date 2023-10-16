add_executable(stitch_tool
    main.cpp
    ${REGISTRATION_SOURCES}
    ${VISUALIZER_SOURCES}
)
target_include_directories(stitch_tool
    PRIVATE ${PC_SOURCE_DIR}
    PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(stitch_tool
    PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(stitch_tool
    PRIVATE ${PCL_DEFINITIONS})

