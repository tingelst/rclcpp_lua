cmake_minimum_required(VERSION 3.5)
project(rclcpp_lua)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp)
find_package(rclcpp_lifecycle)
find_package(rclcpp_components)
find_package(class_loader)
find_package(std_msgs)
find_package(sensor_msgs)
find_package(controller_interface)
find_package(controller_manager)
find_package(hardware_interface)
find_package(kuka_rsi_hw_interface)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)
find_package(hardware_manager REQUIRED)
find_package(etasl_controller REQUIRED)

find_package(Lua 5.3 REQUIRED)
find_package(sol2 REQUIRED)

add_library(rclcpp_lua src/rclcpp_lua.cpp)
target_include_directories(
  rclcpp_lua
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${LUA_INCLUDE_DIR})
target_link_libraries(rclcpp_lua sol2::sol2 ${LUA_LIBRARIES})
ament_target_dependencies(rclcpp_lua
                          rclcpp
                          rclcpp_lifecycle
                          rclcpp_components
                          class_loader
                          std_msgs
                          sensor_msgs
                          controller_interface
                          controller_manager
                          hardware_interface
                          hardware_manager
                          kuka_rsi_hw_interface
                          etasl_controller)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(rclcpp_lua PRIVATE "RCLCPP_LUA_BUILDING_LIBRARY")

add_executable(load_script src/rclcpp_lua2.cpp src/load_script.cpp)
target_include_directories(
  load_script
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${LUA_INCLUDE_DIR})
target_link_libraries(load_script
                      sol2::sol2
                      ${LUA_LIBRARIES}
                      ${TinyXML_LIBRARY})
ament_target_dependencies(load_script
                          rclcpp
                          rclcpp_lifecycle
                          rclcpp_components
                          class_loader
                          std_msgs
                          sensor_msgs
                          controller_interface
                          controller_manager
                          hardware_interface
                          hardware_manager
                          kuka_rsi_hw_interface
                          etasl_controller)

ament_environment_hooks(env_hook/rclcpp_lua.sh.in)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS rclcpp_lua
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS load_script DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION share/scripts)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_interfaces(export_${PROJECT_NAME})
ament_export_libraries(rclcpp_lua)

ament_package()
