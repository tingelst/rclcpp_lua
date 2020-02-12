#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "rclcpp_lua/visibility_control.h"

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  sol::state lua;
  // open some common libraries
  lua.open_libraries(sol::lib::base, sol::lib::package);

  rclcpp::init(argc, argv);

  sol::table etasl = lua.create_table();
  etasl["exec"] = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  lua["etasl"] = etasl;

  lua.script_file(
      "/home/lars/rclcpp_lua_ws/install/rclcpp_lua/share/rclcpp_lua/scripts/"
      "rclcpp.lua");

  //   rclcpp::executors::SingleThreadedExecutor executor;
  //   auto producer = std::make_shared<Producer>("producer", "number");
  //   auto consumer = std::make_shared<Consumer>("consumer", "number");
  //   executor.add_node(producer);
  //   executor.add_node(consumer);
  //   executor.spin();
  rclcpp::shutdown();

  return 0;
}