# Data acquisition software library
add_library(das lidar.cpp)
target_link_libraries(das PRIVATE rplidar_driver)
target_include_directories(das PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")


# Test executable that uses das
add_executable(das_test test/main.cpp)
target_link_libraries(das_test PRIVATE das)
