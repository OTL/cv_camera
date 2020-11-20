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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
# find_package(roslint REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# roslint_cpp()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(cv_camera src/capture.cpp src/driver.cpp)


## Declare a cpp executable
add_executable(cv_camera_node src/cv_camera_node.cpp)
add_dependencies(cv_camera_node cv_camera)

target_link_libraries(cv_camera_node ${Boost_LIBRARIES} cv_camera)

## Specify libraries to link a library or executable target against
ament_target_dependencies(cv_camera
        "rclcpp"
        "sensor_msgs"
        "image_transport"
        "cv_bridge"
        "camera_info_manager"
        "std_msgs"
        "OpenCV"
)

ament_target_dependencies(cv_camera_node
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "OpenCV"
        "camera_info_manager"
        "image_transport"
        "cv_bridge"
)

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
install(TARGETS cv_camera cv_camera_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
  )

ament_package()
