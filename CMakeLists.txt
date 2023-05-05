cmake_minimum_required(VERSION 3.0.2)
project(bilateral_filter)

# ======== dependencies =======
# catkin packages
set(CATKIN_PACKAGES
    roscpp
    std_msgs
    pcl_conversions
    pcl_ros
)
# catkin packages + catkin
find_package(
    catkin REQUIRED 
    COMPONENTS ${CATKIN_PACKAGES}
)

# ==== catkin package info ====
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS ${CATKIN_PACKAGES}
   DEPENDS
)

# =========== build ===========
# header locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# build executables
add_executable(         ${PROJECT_NAME}_node src/bilateral_filter_node.cpp src/bilateral_filter.cpp)
target_link_libraries(  ${PROJECT_NAME}_node ${catkin_LIBRARIES})