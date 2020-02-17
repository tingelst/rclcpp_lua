cmake_minimum_required(VERSION 3.5)
project(hardware_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake)
find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

add_library(hardware_manager SHARED src/hardware_manager.cpp
                                    src/hardware_loader_pluginlib.cpp)
target_include_directories(hardware_manager PRIVATE include)
ament_target_dependencies(hardware_manager
                          "ament_index_cpp"
                          "hardware_interface"
                          "pluginlib"
                          "rclcpp"
                          "rcpputils")
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(hardware_manager PRIVATE
                           "HARDWARE_MANAGER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(hardware_manager PUBLIC
                           "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS hardware_manager
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_libraries(hardware_manager)
ament_export_include_directories(include)
ament_export_dependencies(pluginlib)
ament_package()
