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
find_package(std_msgs)
find_package(Lua 5.3 REQUIRED)
find_package(sol2 REQUIRED)

add_library(rclcpp_lua src/rclcpp_lua.cpp)
target_include_directories(
  rclcpp_lua
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${LUA_INCLUDE_DIR})
target_link_libraries(rclcpp_lua sol2::sol2 ${LUA_LIBRARIES})
ament_target_dependencies(rclcpp_lua rclcpp std_msgs)
ament_environment_hooks(env_hook/rclcpp_lua.sh.in)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(rclcpp_lua PRIVATE "RCLCPP_LUA_BUILDING_LIBRARY")

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rclcpp_lua
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

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