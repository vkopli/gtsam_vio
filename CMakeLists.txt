# MIT License (modified)

# Copyright (c) 2019 The Trustees of the University of Pennsylvania
# Authors:
# Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>
# Vidula Kopli <vkopli@seas.upenn.edu>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this **file** (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Set which node to build to execute ISAM2
set(isam2_node "isam2_vio_zedpose")

cmake_minimum_required(VERSION 2.8.12)
project(gtsam_vio)

add_compile_options(-std=c++11)

# Modify cmake module path if new .cmake files are required
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  sensor_msgs
  geometry_msgs
  eigen_conversions
  tf_conversions
  random_numbers
  message_generation
  nodelet
  image_transport
  cv_bridge
  message_filters
  pcl_conversions
  pcl_ros
  std_srvs
)

## GTSAM stuff
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SuiteSparse REQUIRED)

##################
## ROS messages ##
##################
add_message_files(
  FILES
  FeatureMeasurement.msg
  CameraMeasurement.msg
  TrackingInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_processor
  CATKIN_DEPENDS
    roscpp std_msgs tf nav_msgs sensor_msgs geometry_msgs nav_msgs
    eigen_conversions tf_conversions random_numbers message_runtime
    image_transport cv_bridge message_filters pcl_conversions
    pcl_ros std_srvs
  DEPENDS Boost EIGEN3 OpenCV SUITESPARSE
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)

# Image processor
add_library(image_processor
  src/image_processor.cpp
  src/utils.cpp
)
add_dependencies(image_processor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(image_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Image processor nodelet
add_library(image_processor_nodelet
  src/image_processor_nodelet.cpp
)
add_dependencies(image_processor_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(image_processor_nodelet
  image_processor
  ${catkin_LIBRARIES}
)

# custom ISAM2 nodes

# ISAM2 combined vision and zed pose node
add_executable(isam2 src/${isam2_node}.cpp)
target_link_libraries(isam2 ${catkin_LIBRARIES} gtsam)
add_dependencies(isam2 gtsam_vio_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS
  image_processor image_processor_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*_nodelet.h" EXCLUDE
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # Feature initialization test
  catkin_add_gtest(test_feature_init
    test/feature_initialization_test.cpp
  )
  add_dependencies(test_feature_init
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_feature_init
    ${catkin_LIBRARIES}
  )

  # Math utils test
  catkin_add_gtest(test_math_utils
    test/math_utils_test.cpp
  )
endif()
