cmake_minimum_required(VERSION 3.16)
project(r3v_ros LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(assimp REQUIRED)

include(cmake/r3v_rust.cmake)
r3v_add_rust_library(renderer
  "${CMAKE_CURRENT_SOURCE_DIR}/r3v-renderer" r3v_renderer "cpp,multi-threaded")
# The exported functions are gated on `c`, `cpp` only generates the header.
r3v_add_rust_library(voxelizer
  "${CMAKE_CURRENT_SOURCE_DIR}/r3v-voxelizer" r3v_voxelizer "c,cpp,multi_threaded,wgpu")

set(R3V_ROS_DEPENDENCIES
  ament_index_cpp
  geometry_msgs
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  urdf)

add_library(r3v_ros_component SHARED
  src/camera.cpp
  src/cloud_processor.cpp
  src/mesh_loader.cpp
  src/primitives.cpp
  src/r3v_handles.cpp
  src/r3v_node.cpp
  src/robot_scene.cpp)
target_include_directories(r3v_ros_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(r3v_ros_component ${R3V_ROS_DEPENDENCIES})
if(TARGET assimp::assimp)
  set(R3V_ASSIMP_LIBRARIES assimp::assimp)
else()
  target_include_directories(r3v_ros_component PRIVATE ${ASSIMP_INCLUDE_DIRS})
  set(R3V_ASSIMP_LIBRARIES ${ASSIMP_LIBRARIES})
endif()
target_link_libraries(r3v_ros_component
  Eigen3::Eigen
  ${R3V_ASSIMP_LIBRARIES}
  r3v::renderer
  r3v::voxelizer)
add_dependencies(r3v_ros_component renderer_cargo voxelizer_cargo)
set_target_properties(r3v_ros_component PROPERTIES INSTALL_RPATH "$ORIGIN")
rclcpp_components_register_nodes(r3v_ros_component "r3v_ros::R3vNode")

add_executable(r3v_node src/main.cpp)
ament_target_dependencies(r3v_node ${R3V_ROS_DEPENDENCIES})
target_link_libraries(r3v_node r3v_ros_component Eigen3::Eigen)
set_target_properties(r3v_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS r3v_ros_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS r3v_node DESTINATION lib/${PROJECT_NAME})
install(FILES "${renderer_LIBRARY}" "${voxelizer_LIBRARY}" DESTINATION lib)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
