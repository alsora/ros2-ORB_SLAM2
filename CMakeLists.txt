
cmake_minimum_required(VERSION 3.5)
project(orbslam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Pangolin REQUIRED)
find_package(message_filters REQUIRED)


include_directories(/root/orbslam/ORB_SLAM2/include)
include_directories(/root/orbslam/ORB_SLAM2/)
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${message_filters_INCLUDE_DIRS})

#link_directories(/root/orbslam/ORB_SLAM2/lib/)


add_executable(mono 
    src/mono.cpp)

target_link_libraries(mono
ORB_SLAM2
)

ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM2 Pangolin)

add_executable(rgbd 
    src/rgbd.cpp)

target_link_libraries(rgbd
ORB_SLAM2
)

ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge ORB_SLAM2 Pangolin message_filters)




install(TARGETS mono rgbd
  DESTINATION lib/${PROJECT_NAME})

ament_package()