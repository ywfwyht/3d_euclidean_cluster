cmake_minimum_required(VERSION 2.8.3)
project(lidar_euclidean_cluster)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
std_msgs
sensor_msgs
jsk_recognition_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros std_msgs jsk_recognition_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp src/euclidean_cluster.cpp)
# add_executable(lidar_euclidean_cluster src/euclidean_cluster_core.cpp
#                                         src/euclidean_cluster_node.cpp)

target_include_directories(lidar_euclidean_cluster PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
