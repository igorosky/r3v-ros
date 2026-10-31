cmake_minimum_required(VERSION 3.16)
project(r3v_ros LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Only for our targets: the message headers (r3v_interfaces and the ROS ones)
# do not build with all of them.
set(R3V_ROS_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -pedantic-errors -Wdeprecated
  -Wc++17-compat -Wconversion -Wsign-conversion -Wpessimizing-move -Werror)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(r3v_interfaces REQUIRED)
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
# Ubuntu's libzstd-dev has no CMake package before 24.04.
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

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
  r3v_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  urdf)

# Shared by both nodes: the voxel stream and node helpers.
add_library(r3v_ros_common SHARED
  src/node_common.cpp
  src/voxel_map.cpp
  src/voxel_stream.cpp)
target_include_directories(r3v_ros_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(r3v_ros_common PRIVATE ${R3V_ROS_COMPILE_OPTIONS})
ament_target_dependencies(r3v_ros_common r3v_interfaces rclcpp std_msgs tf2 tf2_msgs tf2_ros)
target_link_libraries(r3v_ros_common PkgConfig::ZSTD)

add_library(r3v_voxelizer_component SHARED
  src/cloud_processor.cpp
  src/voxelizer_node.cpp)
target_compile_options(r3v_voxelizer_component PRIVATE ${R3V_ROS_COMPILE_OPTIONS})
ament_target_dependencies(r3v_voxelizer_component ${R3V_ROS_DEPENDENCIES})
target_link_libraries(r3v_voxelizer_component
  r3v_ros_common
  Eigen3::Eigen
  r3v::voxelizer)
add_dependencies(r3v_voxelizer_component voxelizer_cargo)
rclcpp_components_register_nodes(r3v_voxelizer_component "r3v_ros::VoxelizerNode")

add_library(r3v_renderer_component SHARED
  src/camera.cpp
  src/mesh_loader.cpp
  src/primitives.cpp
  src/renderer_handles.cpp
  src/renderer_node.cpp
  src/robot_scene.cpp)
target_compile_options(r3v_renderer_component PRIVATE ${R3V_ROS_COMPILE_OPTIONS})
ament_target_dependencies(r3v_renderer_component ${R3V_ROS_DEPENDENCIES})
if(TARGET assimp::assimp)
  set(R3V_ASSIMP_LIBRARIES assimp::assimp)
else()
  target_include_directories(r3v_renderer_component PRIVATE ${ASSIMP_INCLUDE_DIRS})
  set(R3V_ASSIMP_LIBRARIES ${ASSIMP_LIBRARIES})
endif()
target_link_libraries(r3v_renderer_component
  r3v_ros_common
  Eigen3::Eigen
  ${R3V_ASSIMP_LIBRARIES}
  r3v::renderer)
add_dependencies(r3v_renderer_component renderer_cargo)
rclcpp_components_register_nodes(r3v_renderer_component "r3v_ros::RendererNode")

set_target_properties(r3v_ros_common r3v_voxelizer_component r3v_renderer_component
  PROPERTIES INSTALL_RPATH "$ORIGIN")

add_executable(r3v_voxelizer_node src/voxelizer_main.cpp)
add_executable(r3v_renderer_node src/renderer_main.cpp)
foreach(node voxelizer renderer)
  target_compile_options(r3v_${node}_node PRIVATE ${R3V_ROS_COMPILE_OPTIONS})
  target_link_libraries(r3v_${node}_node r3v_${node}_component)
  set_target_properties(r3v_${node}_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
endforeach()

install(TARGETS r3v_ros_common r3v_voxelizer_component r3v_renderer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS r3v_voxelizer_node r3v_renderer_node DESTINATION lib/${PROJECT_NAME})
install(FILES "${renderer_LIBRARY}" "${voxelizer_LIBRARY}" DESTINATION lib)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Explicit paths: ament_lint_auto would also lint the submodules' generated
  # bindings.
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_cpplint(src include test)
  ament_uncrustify(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/uncrustify.cfg"
    src include test)
  ament_add_gtest(test_voxel_stream test/test_voxel_stream.cpp)
  target_compile_options(test_voxel_stream PRIVATE ${R3V_ROS_COMPILE_OPTIONS})
  target_link_libraries(test_voxel_stream r3v_ros_common)
endif()

ament_package()
