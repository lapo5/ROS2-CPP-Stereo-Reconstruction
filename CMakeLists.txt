cmake_minimum_required(VERSION 3.5)
project(stereo_reconstruction_cpp)


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

add_executable(stereo_reconstruction 
    src/stereo_reconstruction_node.cpp 
    src/stereo_reconstruction.cpp)

ament_target_dependencies(stereo_reconstruction
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries( stereo_reconstruction
  ${OpenCV_LIBS}
)

install(TARGETS stereo_reconstruction
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


# Install calibration files.
install(DIRECTORY
  calibration
  DESTINATION share/${PROJECT_NAME}/
)

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

# Install rviz files.
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
