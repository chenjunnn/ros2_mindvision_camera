cmake_minimum_required(VERSION 3.10)
project(mindvision_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mv_camera_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC mvsdk/include)
target_link_directories(${PROJECT_NAME} PUBLIC mvsdk/lib)
target_link_libraries(${PROJECT_NAME} MVSDK)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN mindvision_camera::MVCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  set(_linter_excludes
    mvsdk/include/CameraApi.h
    mvsdk/include/CameraDefine.h
    mvsdk/include/CameraStatus.h
  )
  ament_cpplint(EXCLUDE ${_linter_excludes})

  # find_package(ament_cmake_gtest)
  # set(TEST_NAME test_kalman_filter)
  # ament_add_gtest(${TEST_NAME} test/${TEST_NAME}.cpp)
  # target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)