cmake_minimum_required(VERSION 3.8)
project(misora2_cracks)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  # -Wpedantic を有効にしているため、C++17での std::make_pair の仕様変更に伴う警告が出ています。
  # これはC++14とC++17でのパラメータ渡し方法の違いによるもので、ビルドには致命的ではありません。
  # 警告を抑制するには -Wpedantic を外すか無効化してください。
  add_compile_options(-Wall -Wextra)
endif()

# === ONNX Runtime ===
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "onnxruntime-linux-x64-1.21.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "onnxruntime-linux-aarch64-1.19.0")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV 4.8 REQUIRED COMPONENTS core highgui imgproc)
find_package(misora2_custom_msg REQUIRED)

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
    misora2_custom_msg
)

# *.cpp ファイルを明示的に集める
file(GLOB_RECURSE NN_SOURCES "src/nn/*.cpp")
file(GLOB_RECURSE UTILS_SOURCES "src/utils/*.cpp")

# ライブラリ
add_library(cracks_library SHARED src/cracks_component.cpp src/detection.cpp src/size.cpp src/lsd.c ${NN_SOURCES} ${UTILS_SOURCES})
ament_target_dependencies(cracks_library ${dependencies})
target_link_libraries(cracks_library onnxruntime) # ONNX
rclcpp_components_register_nodes(cracks_library "component_cracks::EvaluateCracks")
# ノード　
add_executable(cracks_node src/cracks_component.cpp src/cracks_node.cpp src/detection.cpp src/size.cpp src/lsd.c ${NN_SOURCES} ${UTILS_SOURCES})
ament_target_dependencies(cracks_node ${dependencies})
target_link_libraries(cracks_node onnxruntime) #ONNX

# クラック自動検出 main関数から実行
add_executable(cracks_detection 
    src/detection_main.cpp 
    src/detection.cpp 
    ${NN_SOURCES}
    ${UTILS_SOURCES})
ament_target_dependencies(cracks_detection ${dependencies})
target_link_libraries(cracks_detection onnxruntime) # ONNX

# クラックサイズ測定 main関数から実行
add_executable(cracks_size
    src/size_main.cpp 
    src/size.cpp 
    src/lsd.c
)
ament_target_dependencies(cracks_size ${dependencies})

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
  cracks_size
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
