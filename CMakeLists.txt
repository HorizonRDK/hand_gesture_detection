cmake_minimum_required(VERSION 3.5)
project(hand_gesture_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)

include_directories(include
  ${PROJECT_SOURCE_DIR}
)

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
include_directories(
  ${CMAKE_SYSROOT}/usr/include/
  ${CMAKE_SYSROOT}/usr/include/opencv/
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/hand_gesture_det_node.cpp
  src/hand_gesture_output_parser.cpp
  src/gesture_preprocess.cpp
  src/lmks_process.cpp
  src/gesture_postprocess.cpp
  src/gesture_postprocess_util.cc
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  ai_msgs
)

include_directories(include
${PROJECT_SOURCE_DIR}
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
