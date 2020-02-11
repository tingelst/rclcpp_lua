#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lua/rclcpp_lua.hpp>

#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

namespace test {
using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
 public:
  RCLCPP_LUA_PUBLIC
  explicit Talker(const rclcpp::NodeOptions& options)
      : Node("talker", options) {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message = [this]() -> void {
      msg_ = std::make_unique<std_msgs::msg::String>();
      msg_->data = "Hello World: " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
      // Put the message into a queue to be processed by the middleware.
      // This call is non-blocking.
      pub_->publish(std::move(msg_));
    };
    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

 private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace test

// SOL_BASE_CLASSES(rclcpp::Node);
SOL_BASE_CLASSES(test::Talker, rclcpp::Node);
SOL_DERIVED_CLASSES(rclcpp::Node, test::Talker);

namespace rclcpp_lua {

sol::table register_rclcpp_lua(sol::this_state L) {
  sol::state_view lua(L);
  sol::table module = lua.create_table();

  module.set_function("init", []() { rclcpp::init(0, nullptr); });
  module.set_function("shutdown", []() { rclcpp::shutdown(); });

  sol::usertype<rclcpp::Node> node = module.new_usertype<rclcpp::Node>("Node");
  node["five"] = [](const rclcpp::Node& node) { return 5; };

  sol::usertype<rclcpp::NodeOptions> node_options =
      module.new_usertype<rclcpp::NodeOptions>("NodeOptions");

  sol::table executors = lua.create_table();
  sol::usertype<rclcpp::executors::SingleThreadedExecutor>
      single_threaded_executor =
          executors.new_usertype<rclcpp::executors::SingleThreadedExecutor>(
              "SingleThreadedExecutor");
  single_threaded_executor["spin"] =
      &rclcpp::executors::SingleThreadedExecutor::spin;
  // [](rclcpp::executors::SingleThreadedExecutor& exec) { exec.spin(); };

  single_threaded_executor["add_node"] =
      [](rclcpp::executors::SingleThreadedExecutor& exec,
         std::shared_ptr<rclcpp::Node> node_ptr,
         bool notify = true) { exec.add_node(node_ptr, notify); };

  module["executors"] = executors;

  sol::table test = lua.create_table();
  sol::usertype<test::Talker> talker = test.new_usertype<test::Talker>(
      "Talker",
      //  sol::constructors<test::Talker(const rclcpp::NodeOptions&)>(),
      sol::factories([](rclcpp::NodeOptions& options) {
        std::cout << "factory" << std::endl;
        return std::make_shared<test::Talker>(options);
      }),
      sol::base_classes, sol::bases<rclcpp::Node>());
  module["test"] = test;

  return module;
}

}  // namespace rclcpp_lua

extern "C" int luaopen_librclcpp_lua(lua_State* L) {
  // pass the lua_State,
  // the index to start grabbing arguments from,
  // and the function itself
  // optionally, you can pass extra arguments to the function if that's
  // necessary, but that's advanced usage and is generally reserved for
  // internals only
  return sol::stack::call_lua(L, 1, rclcpp_lua::register_rclcpp_lua);
}
