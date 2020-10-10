cmake_minimum_required(VERSION 2.8.3)
project(segway_sim)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	std_msgs
	sensor_msgs
	geometry_msgs
	visualization_msgs
	nav_msgs
	tf
	pcl_conversions
	pcl_ros
  ambercortex_ros
	)

# Chose osqp version
# if(OSQP_EMBEDDED)
#   message(STATUS "Using embedded version of osqp.")
#   set(OSQP_MODE "osqp_embedded")
#   set(OSQP_MODE_LIB "osqp_embedded")
# else()
#   set(OSQP_MODE "osqp")
#   set(OSQP_MODE_LIB "osqpstatic")
# endif()
set(OSQP_MODE "osqp_embedded")
set(OSQP_MODE_LIB "osqp_embedded")
# Find OSQP library and headers
find_package(${OSQP_MODE} REQUIRED CONFIG)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED CONFIG)
find_package(mpc REQUIRED CONFIG)
find_package(Threads REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  input.msg
  state.msg
  cmd.msg
  sensor.msg
  valFunCnst.msg
  goalSetAndState.msg
  optSol.msg
  linearMatrices.msg
  lowLevelLog.msg
  highLevelBelief.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ui.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp
	             message_generation
	             std_msgs
	             sensor_msgs
	             geometry_msgs
	             visualization_msgs
	             nav_msgs
	             tf
	             pcl_conversions
	             pcl_ros
               ambercortex_ros)

###########
## Build ##
###########
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(integrator_node src/integrator_node.cpp)
add_executable(controller_node src/controller_node.cpp)
add_executable(manual_teleop_node src/manual_teleop_node.cpp src/keyboard.cpp)
add_executable(sensor_node src/sensor_node.cpp)
add_executable(ekf_node src/ekf_node.cpp src/ekf.cpp)
add_executable(mpc_node src/mpc_node.cpp)
add_executable(low_level_node src/low_level_node.cpp)

# Ensure messages built first
add_dependencies(integrator_node segway_sim_generate_messages_cpp)
add_dependencies(controller_node segway_sim_generate_messages_cpp)
add_dependencies(manual_teleop_node segway_sim_generate_messages_cpp)
add_dependencies(sensor_node segway_sim_generate_messages_cpp)
add_dependencies(ekf_node segway_sim_generate_messages_cpp)
add_dependencies(mpc_node segway_sim_generate_messages_cpp)
add_dependencies(low_level_node segway_sim_generate_messages_cpp)


target_link_libraries(integrator_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)

target_link_libraries(controller_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)

target_link_libraries(manual_teleop_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
                      Threads::Threads
)

target_link_libraries(sensor_node
                      ${catkin_LIBRARIES}
)

target_link_libraries(ekf_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
)

target_link_libraries(mpc_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
                      ${OSQP_MODE}::${OSQP_MODE_LIB}
                      mpc::mpc
                      -ldl
)

target_link_libraries(low_level_node
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
                      ${OSQP_MODE}::${OSQP_MODE_LIB}
                      mpc::mpc
                      -ldl
)