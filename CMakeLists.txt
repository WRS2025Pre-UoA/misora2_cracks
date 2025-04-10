cmake_minimum_required(VERSION 3.8)
project(misora2_cracks)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

include_directories(
  include
)

set(dependencies
    rclcpp
    rclcpp_components
    cv_bridge
    OpenCV
    sensor_msgs
    std_msgs
)

add_library(cracks_library SHARED src/cracks_component.cpp)
ament_target_dependencies(cracks_library ${dependencies})
rclcpp_components_register_nodes(cracks_library "component_cracks::EvaluateCracks")

add_executable(cracks_node src/cracks_component.cpp src/cracks_node.cpp)
ament_target_dependencies(cracks_node ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  cracks_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cracks_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
