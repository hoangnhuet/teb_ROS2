cmake_minimum_required(VERSION 3.8)
project(teb_ROS2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread graph chrono)

set(EXTERNAL_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Boost_LIBRARIES}) # tempoparily not neccessary

add_executable(test_obs_ros src/test_obs.cpp src/teb_planner/obstacle/obstacle.cpp src/teb_planner/common/ultis.cpp)
ament_target_dependencies(test_obs_ros
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)
include_directories(
    ${EXTERNAL_INCLUDE_DIRS}
    include
    src/teb_planner/include/obstacle
    src/teb_planner/include/common 
)
target_link_libraries(test_obs_ros ${EXTERNAL_LIBS})
install(TARGETS test_obs_ros
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
