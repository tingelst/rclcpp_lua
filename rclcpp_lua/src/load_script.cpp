#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "rclcpp_lua/visibility_control.h"

#include "controller_manager/controller_manager.hpp"
#include <kuka_rsi_hardware/kuka_rsi_hardware.hpp>
#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe) {
  exe->spin();
}

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  sol::state lua;
  // open some common libraries
  lua.open_libraries(sol::lib::base, sol::lib::package, sol::lib::string);

  rclcpp::init(argc, argv);

  auto executor =
  std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  lua["exec"] = std::move(executor);

  // start the controller manager with the robot hardware
  // controller_manager::ControllerManager cm(my_robot, executor);

  // lua.set_function("get_controller_manager", [&cm]() { return cm; });

  lua.script_file(
      "/home/lars/etasl_ros2_control_ws/install/rclcpp_lua/share/rclcpp_lua/"
      "scripts/rclcpp.lua");

  //   rclcpp::executors::SingleThreadedExecutor executor;
  //   auto producer = std::make_shared<Producer>("producer", "number");
  //   auto consumer = std::make_shared<Consumer>("consumer", "number");
  //   executor.add_node(producer);
  //   executor.add_node(consumer);
  //   executor.spin();


  std::shared_ptr<kuka_rsi_hardware::KukaRsiHardware> robot = lua["robot"];

  // hardware_interface::RobotHardware* robot = lua["robot"];
  // kuka_rsi_hardware::KukaRsiHardware& robot = lua["robot"];
  // std::shared_ptr<controller_manager::ControllerManager> cm = lua["cm"];;

  // std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec =
  // lua["exec"];

  // rclcpp::Rate r(100);
  // hardware_interface::hardware_interface_ret_t ret;
  // while (rclcpp::ok()) {
  //   ret = robot->read();
  //   if (ret != hardware_interface::HW_RET_OK) {
  //     fprintf(stderr, "read failed!\n");
  //   }
  //   cm->update();
  //   ret = robot->write();
  //   if (ret != hardware_interface::HW_RET_OK) {
  //     fprintf(stderr, "write failed!\n");
  //   }
  //   r.sleep();
  // }

  // exec->cancel();

  // rclcpp::shutdown();

  return 0;
}