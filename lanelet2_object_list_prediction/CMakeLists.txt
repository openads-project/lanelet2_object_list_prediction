cmake_minimum_required(VERSION 3.8)
project(lanelet2_object_list_prediction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_map_interface REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_perception_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(TARGET_NAME lanelet2_object_list_prediction)

add_executable(${TARGET_NAME} src/lanelet2_object_list_prediction.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

target_link_libraries(${TARGET_NAME} PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${perception_msgs_TARGETS}
  ${lanelet2_core_LIBRARIES}
  ${lanelet2_routing_LIBRARIES}
  ${lanelet2_traffic_rules_LIBRARIES}
  lanelet2_map_interface::lanelet2_map_interface
  perception_msgs_utils::perception_msgs_utils
  tf2::tf2
  tf2_perception_msgs::tf2_perception_msgs
  tf2_ros::tf2_ros
)

install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
