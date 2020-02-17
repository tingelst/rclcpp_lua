#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/robot_hardware.hpp"

#include "rclcpp_lua/rclcpp_lua.hpp"

#include "rclcpp_lua/visibility_control.h"

class AsyncSpinner {
 public:
  explicit AsyncSpinner(
      std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
      : executor_(executor) {}
  void async_spin() {
    handle_ = std::async(std::launch::async, spin, executor_);
  }

 private:
  static void spin(
      std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe) {
    exe->spin();
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::future<void> handle_;
};

class RobotManager {
 public:
  void set_robot(std::shared_ptr<hardware_interface::RobotHardware> robot) {
    std::cout << "Setting robot" << std::endl;
    robot_ = robot;
  }

  std::shared_ptr<hardware_interface::RobotHardware> get_robot() {
    return robot_;
  }

 private:
  std::shared_ptr<hardware_interface::RobotHardware> robot_;
};

namespace rclcpp_lua {
void register_rclcpp_lua(sol::state_view L);
}

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  sol::state lua;
  lua.open_libraries(sol::lib::base, sol::lib::package, sol::lib::string);

  rclcpp_lua::register_rclcpp_lua(lua);

  lua.new_usertype<RobotManager>("RobotManager", sol::factories([]() {
                                   return std::make_shared<RobotManager>();
                                 }),
                                 "set_robot", &RobotManager::set_robot,
                                 "get_robot", &RobotManager::get_robot);

  rclcpp::init(argc, argv);

  auto robot_manager = std::make_shared<RobotManager>();
  lua.set_function("get_robot_manager", [&]() { return robot_manager; });

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  lua.set_function("get_executor", [&]() { return executor; });

  auto spinner = AsyncSpinner(executor);

  lua.script_file(
      "/home/lars/etasl_ros2_control_ws/install/rclcpp_lua/share/rclcpp_lua/"
      "scripts/rclcpp.lua");

  auto robot = robot_manager->get_robot();

  std::shared_ptr<controller_manager::ControllerManager> cm = lua["cm"];

  spinner.async_spin();

  rclcpp::Rate r(100);
  hardware_interface::hardware_interface_ret_t ret;
  while (rclcpp::ok()) {
    ret = robot->read();

    if (ret != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "read failed!\n");
    }

    cm->update();

    ret = robot->write();
    if (ret != hardware_interface::HW_RET_OK) {
      fprintf(stderr, "write failed!\n");
    }

    r.sleep();
  }

  executor->cancel();

  rclcpp::shutdown();

  return 0;
}