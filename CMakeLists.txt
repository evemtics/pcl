cmake_minimum_required(VERSION 2.8.3)
project(chapter10_tutorials)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  rospy
)
 
find_package(PCL REQUIRED)
catkin_package()
 
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
 
link_directories(${PCL_LIBRARY_DIRS})
 
 
add_executable(pcl_create src/pcl_create.cpp)
target_link_libraries(pcl_create ${catkin_LIBRARIES} ${PCL_LIBRARIES})
 
add_executable(pcl_read src/pcl_read.cpp)
add_executable(pcl_write src/pcl_write.cpp)
add_executable(pcl_filter src/pcl_filter.cpp)

target_link_libraries(pcl_read ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_write ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_downSample src/pcl_downSample.cpp)

target_link_libraries(pcl_downSample ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_Radius src/pcl_Radius.cpp)

target_link_libraries(pcl_Radius ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(pcl_Gassian src/pcl_Gassian.cpp)


target_link_libraries(pcl_Gassian ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(sansac_line src/sansac_line.cpp)

target_link_libraries(sansac_line ${catkin_LIBRARIES} ${PCL_LIBRARIES})
