cmake_minimum_required(VERSION 3.8)
project(ros2_shm_vision_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)

include_directories(include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(msg_files
  "msg/Image.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${typesupport_target} ${OpenCV_LIBS})
ament_target_dependencies(talker rclcpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${typesupport_target} ${OpenCV_LIBS})
ament_target_dependencies(listener rclcpp)

add_executable(filter src/filter_node.cpp)
target_link_libraries(filter ${typesupport_target} ${OpenCV_LIBS})
ament_target_dependencies(filter rclcpp)

add_executable(edge_detector src/edge_detector_node.cpp)
target_link_libraries(edge_detector ${typesupport_target} ${OpenCV_LIBS})
ament_target_dependencies(edge_detector rclcpp)

add_executable(optical_flow src/optical_flow_node.cpp)
target_link_libraries(optical_flow ${typesupport_target} ${OpenCV_LIBS})
ament_target_dependencies(optical_flow rclcpp)

add_executable(object_detector src/object_detector_node.cpp)
target_link_libraries(object_detector ${typesupport_target} ${OpenCV_LIBS})
ament_target_dependencies(object_detector rclcpp)


install(TARGETS talker listener filter edge_detector optical_flow object_detector
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
