cmake_minimum_required(VERSION 3.8)
project(ros2_rs_crosswalk_signal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14->C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_rs_interfaces REQUIRED)

# 自作ライブラリの設定 (手動でライブラリとヘッダーパスを設定)
set(RS_COMMON_LIB "/usr/local/lib/librs_common.so")
set(RS_COMMON_INCLUDE_DIR "/usr/local/include/ryusei/common")

# ソースファイルの追加
add_executable(test_light_sub src/test_light_sub.cpp)  
ament_target_dependencies(test_light_sub rclcpp ros2_rs_interfaces)  # 依存関係の指定

# components
add_library(${PROJECT_NAME} SHARED
  src/crosswalk_signal.cpp
  src/test_img_pub.cpp
  src/test_img_sub.cpp
  src/check_hsv.cpp
  src/capture_cam_log.cpp
  src/log_img_pub.cpp
)

# Include directories (rs_common ライブラリのヘッダーファイルのパスを含める)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${RS_COMMON_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  rs_common
  stdc++fs
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp OpenCV 
  cv_bridge 
  sensor_msgs 
  rclcpp_components 
  ros2_rs_interfaces 
  fmt
)

rclcpp_components_register_nodes(${PROJECT_NAME} "crosswalk_signal::Recognition")
rclcpp_components_register_nodes(${PROJECT_NAME} "crosswalk_signal::ImagePublisher")
rclcpp_components_register_nodes(${PROJECT_NAME} "crosswalk_signal::ImageSubscriber")
rclcpp_components_register_nodes(${PROJECT_NAME} "crosswalk_signal::HSVChecker")
rclcpp_components_register_nodes(${PROJECT_NAME} "crosswalk_signal::CaptureCamLog")
rclcpp_components_register_nodes(${PROJECT_NAME} "crosswalk_signal::LogImgPublisher")

# Install targets
install(TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)

# install test_light_sub target
install(TARGETS test_light_sub 
  DESTINATION lib/${PROJECT_NAME}
)

#install configuration directory
install(
  DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/
)

# install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# includeディレクトリをincludeパスに追加
include_directories(
  include
)

ament_package()
