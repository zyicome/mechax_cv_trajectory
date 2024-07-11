cmake_minimum_required(VERSION 3.10)
project(left_armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
#set(OpenCV_DIR /home/mechax/git/opencv/install)
#set(OpenCV_INCLUDE_DIRS /home/mechax/build-opencv/install/include/opencv4)
#set(OpenCV_INCLUDE_DIRS /home/mechax/git/opencv/include/opencv2)
#find_package(OpenCV 4.5.4 REQUIRED PATHS /home/mechax/build-opencv/install/cmake)
find_package(OpenCV REQUIRED PATHS /home/mechax/git/opencv/install/cmake)
#find_package(InferenceEngine REQUIRED PATHS /home/mechax/git/opencv/install/cmake)
#set(OpenCV_LIBRARY_DIRS /home/mechax/build-opencv/install/lib)
#set(OpenCV_LIBS /home/mechax/git/opencv/install/lib)
message(WARNING "${OpenCV_DIR} ${OpenCV_VERSION}")
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_left_auto_aim::ArmorDetectorNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
