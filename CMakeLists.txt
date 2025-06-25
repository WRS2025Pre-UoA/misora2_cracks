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

# === ONNX Runtime ===
set(ONNXRUNTIME_DIR "onnxruntime-linux-x64-1.21.0")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

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

# *.cpp ファイルを明示的に集める
file(GLOB_RECURSE NN_SOURCES "src/nn/*.cpp")
file(GLOB_RECURSE UTILS_SOURCES "src/utils/*.cpp")

# ライブラリ
add_library(cracks_library SHARED src/cracks_component.cpp src/detection.cpp ${NN_SOURCES} ${UTILS_SOURCES})
ament_target_dependencies(cracks_library ${dependencies})
target_link_libraries(cracks_library onnxruntime) 
rclcpp_components_register_nodes(cracks_library "component_cracks::EvaluateCracks")
# ノード　
add_executable(cracks_node src/cracks_component.cpp src/cracks_node.cpp src/detection.cpp ${NN_SOURCES} ${UTILS_SOURCES})
ament_target_dependencies(cracks_node ${dependencies})
target_link_libraries(cracks_node onnxruntime) # ← 追加

# クラック自動検出 main関数から実行

add_executable(cracks_detection 
    src/detection_main.cpp 
    src/detection.cpp 
    ${NN_SOURCES}
    ${UTILS_SOURCES})
ament_target_dependencies(cracks_detection ${dependencies})
target_link_libraries(cracks_detection onnxruntime) # ← 追加

set_target_properties(cracks_detection PROPERTIES 
 INSTALL_RPATH "$(ONNXRUNTIME_DIR)"
 BUILD_RPATH "$(ONNXRUNTIME_DIR)"
)

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
  cracks_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
