cmake_minimum_required(VERSION 3.5)
project(etasl_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(etasl_controller SHARED src/etasl_controller.cpp)
target_include_directories(etasl_controller PRIVATE include)

ament_target_dependencies(etasl_controller
                          "builtin_interfaces"
                          "controller_interface"
                          "pluginlib"
                          "rclcpp_lifecycle"
                          "rcutils"
                          "sensor_msgs")
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(etasl_controller PRIVATE
                           "ETASL_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface
                                         etasl_controller.xml)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS etasl_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(controller_interface rclcpp_lifecycle sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(etasl_controller)
ament_package()