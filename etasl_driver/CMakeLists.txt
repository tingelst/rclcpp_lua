cmake_minimum_required(VERSION 3.5)
project(etasl_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(expressiongraph REQUIRED)
find_package(expressiongraph_context REQUIRED)
find_package(expressiongraph_context_lua REQUIRED)
find_package(expressiongraph_solver_qpoases REQUIRED)

find_package(Lua 5.3 REQUIRED)
find_package(sol2 REQUIRED)

add_library(etasl_driver src/etasl_driver.cpp src/lua_context.cpp)
target_include_directories(
  etasl_driver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
         ${LUA_INCLUDE_DIR}
         )

target_link_libraries(etasl_driver sol2::sol2 ${LUA_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
                          "expressiongraph"
                          "expressiongraph_context"
                          "expressiongraph_context_lua"
                          "expressiongraph_solver_qpoases")

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(etasl_driver PRIVATE "ETASL_DRIVER_BUILDING_LIBRARY")

# add_executable(test_etasl_driver test/test_etasl_driver.cpp)
# target_include_directories(
#   test_etasl_driver
#   PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#          $<INSTALL_INTERFACE:include>
#          ${LUA_INCLUDE_DIR}
#          )
# target_link_libraries(test_etasl_driver etasl_driver sol2::sol2 ${LUA_LIBRARIES})
# ament_target_dependencies(test_etasl_driver
#                           "expressiongraph"
#                           "expressiongraph_context"
#                           "expressiongraph_context_lua"
#                           "expressiongraph_solver_qpoases")

install(DIRECTORY include/ DESTINATION include)
install(TARGETS etasl_driver
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

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
ament_export_libraries(etasl_driver)
ament_export_dependencies("expressiongraph"
                          "expressiongraph_context"
                          "expressiongraph_context_lua"
                          "expressiongraph_solver_qpoases"
                          "sol2")

ament_package()
