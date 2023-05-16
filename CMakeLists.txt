cmake_minimum_required(VERSION 3.10)
project(mindvision_camera)

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
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mv_camera_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC mvsdk/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC mvsdk/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC mvsdk/lib/arm64)
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

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
endif()

# ############
# # Install ##
# ############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
