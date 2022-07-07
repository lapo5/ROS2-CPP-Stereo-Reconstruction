cmake_minimum_required(VERSION 3.5)
project(stereo_logger)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(
  ./ 
        /usr/include/opencv4
  ) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  rclcpp
  sensor_msgs
  cv_bridge
)

add_executable(stereo_logger
    src/stereo_logger_node.cpp )

ament_target_dependencies(stereo_logger
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries( stereo_logger
  ${OpenCV_LIBS}
)

install(TARGETS stereo_logger
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install params files.
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
