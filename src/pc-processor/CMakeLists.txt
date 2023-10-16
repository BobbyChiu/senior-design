find_package(PCL REQUIRED)

set(PC_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(LEVELER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/leveler/simulated_leveler.cpp
)

set(REGISTRATION_SOURCES # a.k.a. stitching, ICP
    ${CMAKE_CURRENT_SOURCE_DIR}/registration/icp_strategy.cpp
)

set(VISUALIZER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/visualizer/visualizer.cpp
)

add_executable(lidar2pc
    main.cpp
    lidar_processor.cpp
    ${VISUALIZER_SOURCES}
)
target_include_directories(lidar2pc
    PRIVATE ${PC_SOURCE_DIR}
    PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar2pc
    PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(lidar2pc
    PRIVATE ${PCL_DEFINITIONS})

add_subdirectory(registration)
