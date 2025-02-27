cmake_minimum_required(VERSION 3.8)
project(rbf_gnss_ins_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Find GeographicLib
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
        PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
        NAMES Geographic
)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# List of message files
set(msg_files
        "msg/Heading.msg"
        "msg/GnssStatus.msg"
        "msg/GnssVel.msg"
        "msg/ImuStatus.msg"
        "msg/Ins.msg"
        "msg/ECEF.msg"
        "msg/RTCMStatus.msg"
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
          ${msg_files}
          DEPENDENCIES
            std_msgs
        )

# Add rbf_ntrip_driver.cpp as a rclcpp_component target
add_library(rbf_gnss_ins_component SHARED
src/binary_parser.cpp
src/ll_to_utm_transform.cpp
src/serial_port.cpp
src/rbf_gnss_ins_driver.cpp
src/converter.cpp
)

ament_target_dependencies(rbf_gnss_ins_component
  rclcpp
  rclcpp_components
  sensor_msgs
  mavros_msgs
  nav_msgs
  tf2
  tf2_ros
  diagnostic_updater
)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rbf_gnss_ins_component "${cpp_typesupport_target}") 

rclcpp_components_register_node(
rbf_gnss_ins_component
PLUGIN "rbf_gnss_ins_driver::GnssInsDriver"
EXECUTABLE rbf_gnss_ins_component_driver
)

ament_export_targets(export_rbf_gnss_ins_component)
install(TARGETS rbf_gnss_ins_component
        EXPORT export_rbf_gnss_ins_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)


# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
