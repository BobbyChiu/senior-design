find_package(PCL REQUIRED)

add_executable(lidar2pc
    lidar_to_point_cloud.cpp)
target_include_directories(lidar2pc
    PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar2pc
    PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(lidar2pc
    PRIVATE ${PCL_DEFINITIONS})
