cmake_minimum_required(VERSION 3.5)
project(cv_camera)

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


find_package(OpenCV 4 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(utils)
find_package(builtin_interfaces REQUIRED)

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -fPIC")

include_directories(
  include
  # ${Boost_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(cv_camera_driver SHARED src/capture.cpp src/driver.cpp )


## Declare a cpp executable
add_executable(cv_camera_node src/cv_camera_node.cpp)
# add_dependencies(cv_camera_node cv_camera_driver)

target_link_libraries(cv_camera_node 
  cv_camera_driver
  utils::utils
  )


set(CV_DRIVER_DEPENDENCIES
rclcpp
rclcpp_components
sensor_msgs
image_transport
cv_bridge
camera_info_manager
std_msgs
OpenCV
utils

)
## Specify libraries to link a library or executable target against
ament_target_dependencies(cv_camera_driver
  ${CV_DRIVER_DEPENDENCIES}
)

rclcpp_components_register_nodes(cv_camera_driver "cv_camera::Driver")
set(node_plugins "${node_plugins}cv_camera::Driver;$<TARGET_FILE:driver>\n")

ament_target_dependencies(cv_camera_node
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "OpenCV"
        "camera_info_manager"
        "image_transport"
        "cv_bridge"
)

ament_export_targets(export_cv_camera_driver HAS_LIBRARY_TARGET)

ament_export_dependencies(${CV_DRIVER_DEPENDENCIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS cv_camera_driver 
  EXPORT export_cv_camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

install(
  TARGETS cv_camera_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_export_include_directories(include)

ament_package()
