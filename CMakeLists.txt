cmake_minimum_required(VERSION 3.8)
project(rbf_gnss_ins_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add rbf_ntrip_driver.cpp as a rclcpp_component target
ament_auto_add_library(rbf_gnss_ins_component SHARED
src/parser.cpp
src/ll_to_utm_transform.cpp
src/serial_port.cpp
src/rbf_gnss_ins_driver.cpp
src/converter.cpp
)

# Find GeographicLib
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
        PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
        NAMES Geographic
)



# List of message files
set(msg_files
        "msg/Heading.msg"
        "msg/GnssStatus.msg"
        "msg/GnssVel.msg"
        "msg/ImuStatus.msg"
        "msg/Ins.msg"
        "msg/ECEF.msg"
        "msg/RTCMStatus.msg"
        "msg/Gpnav.msg"
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
          ${msg_files}
          DEPENDENCIES
            std_msgs
        )



        
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rbf_gnss_ins_component "${cpp_typesupport_target}") 

rclcpp_components_register_node(
rbf_gnss_ins_component
PLUGIN "rbf_gnss_ins_driver::GnssInsDriver"
EXECUTABLE rbf_gnss_ins_component_driver
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
INSTALL_TO_SHARE
launch
config
)
