cmake_minimum_required(VERSION 2.8.3)
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0.2")
  if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
  endif (POLICY CMP0048)
endif()

project(image_undistort)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Set compile flags
if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")
else()
  set(CMAKE_CXX_FLAGS "-Ofast")
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  sensor_msgs
  cv_bridge
  image_transport
  tf_conversions
  nodelet
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
)

find_package(OpenCV REQUIRED COMPONENTS core calib3d)

# --- Only include relevant source files ---
add_library(${PROJECT_NAME}_nodelet 
  src/nodelets/image_undistort_nodelet.cpp 
  src/camera_parameters.cpp 
  src/image_undistort.cpp 
  src/undistorter.cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})

# ---- ONLY UNIDISTORT NODE EXECUTABLE ----
add_executable(image_undistort_node src/nodes/image_undistort_node.cpp)
target_link_libraries(image_undistort_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

# --- REMOVED (OR COMMENTED) OTHER EXECUTABLES ---
# add_executable(stereo_info_node src/nodes/stereo_info_node.cpp)
# add_executable(stereo_undistort_node src/nodes/stereo_undistort_node.cpp)
# add_executable(depth_node src/nodes/depth_node.cpp)
# add_executable(dense_stereo_node src/nodes/dense_stereo_node.cpp)
# add_executable(point_to_bearing_node src/nodes/point_to_bearing_node.cpp)

# --- INSTALLATION ---
install(TARGETS ${PROJECT_NAME}_nodelet
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)