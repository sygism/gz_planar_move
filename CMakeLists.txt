cmake_minimum_required(VERSION 3.10)
project(gz_planar_move)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/PlanarMove.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}
  gz-sim8
  gz-transport13
  rclcpp
  nav_msgs
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION share/${PROJECT_NAME}/lib
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_package()
