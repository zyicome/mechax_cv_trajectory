cmake_minimum_required(VERSION 3.8)
project(rm_rune)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置默认的 ONNX Runtime 根目录，如果用户没有手动提供
if(NOT ONNXRUNTIME_ROOTDIR)
    if(WIN32)
        # Windows 系统下设置默认的 ONNX Runtime 路径
        set(ONNXRUNTIME_ROOTDIR "C:/Program Files (x86)/onnxruntime")
    else()
        # 非 Windows 系统下设置默认路径
        set(ONNXRUNTIME_ROOTDIR "/home/qianli/onnxruntime")
    endif()
endif()

# 显示当前使用的 ONNXRUNTIME_ROOTDIR 路径
message(STATUS "Using ONNX Runtime directory: ${ONNXRUNTIME_ROOTDIR}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# 添加包含路径
include_directories(
  ${ONNXRUNTIME_ROOTDIR}/include
  ${ONNXRUNTIME_ROOTDIR}/include/onnxruntime/core/session
  ${CMAKE_SOURCE_DIR}/include
)

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Ceres REQUIRED)
 
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/local/include
    /usr/include/
    /usr/include/eigen3
)
 
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include/rm_rune)

add_library(${PROJECT_NAME} SHARED
  src/rm_rune_node.cpp
  src/power_rune.cpp
  src/image_processer.cpp
  src/contour_info.cpp
  src/configuration.cpp
  src/blade.cpp
  src/prediction.cpp
  src/augment.cpp
  src/autobackend.cpp
  src/common.cpp
  src/detect.cpp
  src/onnx_model_base.cpp
  src/ops.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components sensor_msgs geometry_msgs cv_bridge OpenCV tf2 tf2_ros image_transport tf2_geometry_msgs)

link_directories(${ONNXRUNTIME_ROOTDIR}/lib)

target_link_directories(${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} onnxruntime Ceres::ceres)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qianli_rm_rune::RuneNode
  EXECUTABLE ${PROJECT_NAME}_node
)




install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# install(FILES plugins.xml
#   DESTINATION share/${PROJECT_NAME}
# )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
