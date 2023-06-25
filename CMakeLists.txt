cmake_minimum_required(VERSION 3.5)
project(gl_depth_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(AMENT_PKGS
    rosidl_typesupport_cpp
    rosidl_default_generators
    rclcpp
    pcl_conversions
    tf2
    tf2_ros # Used for ROS example
    tf2_eigen # Used for ROS example
    pluginlib
    urdf
    visualization_msgs # Used for ROS example
    sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(ament_pkg IN LISTS AMENT_PKGS)
  find_package(${ament_pkg} REQUIRED)
endforeach()


# Required for core functionality
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Threads REQUIRED)

# Extensions
find_package(assimp REQUIRED) # Just used for loading models in mesh_loader.h
find_package(OpenCV REQUIRED) # Used for interface extension

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Primary rendering library
# Independent of ROS, but does need glfw3 and assimp for model loading
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/sim_depth_camera.cpp
  src/${PROJECT_NAME}/mesh_loader.cpp
  src/${PROJECT_NAME}/mesh.cpp
  src/${PROJECT_NAME}/renderable_mesh.cpp
  src/${PROJECT_NAME}/shader_program.cpp
  src/${PROJECT_NAME}/glad.c
  src/${PROJECT_NAME}/glfw_guard.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${OPENGL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  dl
  glfw
)

# Libaries for interfacing with opencv and pcl
add_library(${PROJECT_NAME}_interfaces SHARED
  src/interfaces/pcl_interface.cpp
  src/interfaces/opencv_interface.cpp
)
#target_include_directories(${PROJECT_NAME}_interfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_interfaces PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_interfaces
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Simulated laser scanner library
add_library(${PROJECT_NAME}_sim_laser_scanner SHARED
  src/${PROJECT_NAME}/sim_laser_scanner.cpp
)
#add_dependencies(${PROJECT_NAME}_interfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_sim_laser_scanner PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_sim_laser_scanner
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${PCL_LIBRARIES}
)

## Example showing basic usage
add_executable(${PROJECT_NAME}_test src/usage_example.cpp)
#set_target_properties(${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME depth_example PREFIX "")
target_include_directories(${PROJECT_NAME}_test PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Example showing an orbiting camera
add_executable(${PROJECT_NAME}_orbit src/camera_orbit_example.cpp)
#set_target_properties(${PROJECT_NAME}_orbit PROPERTIES OUTPUT_NAME orbit_example PREFIX "")
target_include_directories(${PROJECT_NAME}_orbit PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_orbit
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  Threads::Threads
  ${rclcpp_LIBRARIES}
)

## Example showing orbiting camera with point cloud publishing in ROS
##add_executable(${PROJECT_NAME}_ros_orbit src/camera_ros_example.cpp)
##set_target_properties(${PROJECT_NAME}_ros_orbit PROPERTIES OUTPUT_NAME ros_example PREFIX "")
##target_link_libraries(${PROJECT_NAME}_ros_orbit
##  ${PROJECT_NAME}
##  ${PROJECT_NAME}_interfaces
##  ${catkin_LIBRARIES}
##)

## Example showing sweeping laser scan with point cloud publishing in ROS
##add_executable(${PROJECT_NAME}_laser_example src/laser_example.cpp)
##set_target_properties(${PROJECT_NAME}_laser_example PROPERTIES OUTPUT_NAME laser_example PREFIX "")
##target_link_libraries(${PROJECT_NAME}_laser_example
##  ${PROJECT_NAME}_sim_laser_scanner
##  ${catkin_LIBRARIES}
##)

add_library(${PROJECT_NAME}_simulator_plugins SHARED
  src/simulator/ros_urdf_scene_updater.cpp
  src/simulator/depth_camera_plugin.cpp
  src/simulator/laser_scanner_plugin.cpp
)
target_include_directories(${PROJECT_NAME}_simulator_plugins PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_simulator_plugins
  ${rclcpp_LIBRARIES}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${PROJECT_NAME}_sim_laser_scanner
  ${PCL_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  yaml-cpp
  ${urdf_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

add_executable(${PROJECT_NAME}_simulator
  src/simulator/simulator.cpp
)
target_include_directories(${PROJECT_NAME}_simulator PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_simulator PUBLIC
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${PROJECT_NAME}_sim_laser_scanner
  ${rclcpp_LIBRARIES}
  ${TinyXML2_LIBRARIES}
  ${pluginlib_LIBRARIES}
  yaml-cpp
)
#target_compile_definitions(${PROJECT_NAME}_simulator PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin_description.xml)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_interfaces
    ${PROJECT_NAME}_sim_laser_scanner
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}_orbit
##    ${PROJECT_NAME}_ros_orbit
##    ${PROJECT_NAME}_laser_example
    ${PROJECT_NAME}_simulator_plugins
    ${PROJECT_NAME}_simulator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME}_simulator_plugins)
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
    rosidl_default_runtime
    pluginlib
    )
ament_package()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  catkin_add_gtest(utest_node test/utest.cpp)
#  target_link_libraries(utest_node
#    ${catkin_LIBRARIES}
#    ${PROJECT_NAME}_sim_laser_scanner
#  )
#endif()
