#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "rclcpp_lua/visibility_control.h"

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe) {
  exe->spin();
}

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  sol::state lua;
  // open some common libraries
  lua.open_libraries(sol::lib::base, sol::lib::package, sol::lib::string);

  rclcpp::init(argc, argv);

  lua.script_file(
      "/home/lars/etasl_ros2_control_ws/install/rclcpp_lua/share/rclcpp_lua/"
      "scripts/rclcpp.lua");

  rclcpp::shutdown();

  return 0;
}