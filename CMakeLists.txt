cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(PythonLibs REQUIRED)

add_executable(elevation_map_node
  src/elevation_map_main.cpp
  src/elevation_map_node.cpp
)
ament_target_dependencies(elevation_map_node
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_include_directories(elevation_map_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(elevation_map_node ${PCL_LIBRARIES})

set(POINT_LIO_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/point_lio_ros2")
add_executable(autonomy_light_pointlio_mapping
  "${POINT_LIO_VENDOR_DIR}/src/laserMapping.cpp"
  "${POINT_LIO_VENDOR_DIR}/include/ikd-Tree/ikd_Tree.cpp"
  "${POINT_LIO_VENDOR_DIR}/src/parameters.cpp"
  "${POINT_LIO_VENDOR_DIR}/src/preprocess.cpp"
  "${POINT_LIO_VENDOR_DIR}/src/Estimator.cpp"
)
target_compile_definitions(autonomy_light_pointlio_mapping
  PRIVATE ROOT_DIR="${POINT_LIO_VENDOR_DIR}/"
)
target_compile_options(autonomy_light_pointlio_mapping PRIVATE -O3 -pthread -fexceptions)
target_include_directories(autonomy_light_pointlio_mapping PRIVATE
  "${POINT_LIO_VENDOR_DIR}/include"
  "${POINT_LIO_VENDOR_DIR}/src"
  ${EIGEN3_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
)
ament_target_dependencies(autonomy_light_pointlio_mapping
  geometry_msgs
  livox_ros_driver2
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(autonomy_light_pointlio_mapping
  ${PYTHON_LIBRARIES}
  OpenMP::OpenMP_CXX
)

install(TARGETS elevation_map_node autonomy_light_pointlio_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
