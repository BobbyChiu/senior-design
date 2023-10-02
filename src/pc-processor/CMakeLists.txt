find_package(PCL REQUIRED)

set(LEVELER_SOURCES
    leveler/simulated_leveler.cpp)

add_executable(lidar2pc
    main.cpp
    lidar_processor.cpp
    visualizer.cpp)
target_include_directories(lidar2pc
    PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar2pc
    PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(lidar2pc
    PRIVATE ${PCL_DEFINITIONS})
