cmake_minimum_required(VERSION 3.5)
project(image_mode_service)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_mode_service src/image_mode_service.cpp)
ament_target_dependencies(image_mode_service rclcpp std_srvs sensor_msgs cv_bridge OpenCV)

install(TARGETS
  image_mode_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
