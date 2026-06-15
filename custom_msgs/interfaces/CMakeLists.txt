cmake_minimum_required(VERSION 3.8)
project(interfaces)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(shared_interfaces REQUIRED)

# Message definitions
set(msg_files
  "msg/ClippingShape.msg"
  "msg/ClippingShapes.msg"
  "msg/FrictionMap.msg"
  "msg/FrictionPoint.msg"
  "msg/Path.msg"
  "msg/Trajectory.msg"
  "msg/RacingLine.msg"
  "msg/VehicleState.msg"
  "msg/MpcPrediction.msg"
  "msg/MuScaling.msg"
  "msg/StabilityCriteria.msg"
  "msg/RequestMuScaling.msg"
  "msg/HardwareStatus.msg"
  "msg/MiddleLine.msg"
  "msg/RuntimeProfile.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Create a different target to export files on include/
add_library(${PROJECT_NAME}_headers INTERFACE)

target_include_directories(${PROJECT_NAME}_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_headers INTERFACE 
  Eigen3::Eigen 
  shared_interfaces::shared_interfaces__rosidl_typesupport_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_headers
  EXPORT ${PROJECT_NAME}_headers
)

ament_export_targets(${PROJECT_NAME}_headers HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(Eigen3)
ament_export_dependencies(shared_interfaces)

ament_package()
