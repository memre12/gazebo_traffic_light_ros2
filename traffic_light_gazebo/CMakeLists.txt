cmake_minimum_required(VERSION 3.8)
project(traffic_light_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(traffic_light_plugin SHARED
  src/GazeboTrafficLight.cpp
)

ament_target_dependencies(traffic_light_plugin
  rclcpp
  gazebo_ros
  std_msgs
)

target_include_directories(traffic_light_plugin PUBLIC
  include
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(traffic_light_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS
    traffic_light_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    include/
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
