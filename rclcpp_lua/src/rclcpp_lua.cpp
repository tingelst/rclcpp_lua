#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lua/rclcpp_lua.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cinttypes>

#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

#include <cmath>

namespace test {
using namespace std::chrono_literals;

class LifecycleController : public rclcpp_lifecycle::LifecycleNode {
 public:
  RCLCPP_LUA_PUBLIC
  explicit LifecycleController(const rclcpp::NodeOptions& options)
      : LifecycleNode("LifecycleController",
                      rclcpp::NodeOptions().use_intra_process_comms(true)) {
    output_topic_ = this->declare_parameter("output_topic", "command");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state) {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Float64>(output_topic_, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type>
        captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub, this]() -> void {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr) {
        return;
      }
      static double command{0.0};
      std_msgs::msg::Float64::UniquePtr msg =
          std::make_unique<std_msgs::msg::Float64>();
      msg->data = 0.1 * std::sin(this->get_clock()->now().nanoseconds());
      printf("Published message with value: %f, and address: 0x%" PRIXPTR "\n",
             msg->data, reinterpret_cast<uintptr_t>(msg.get()));
      pub_ptr->publish(std::move(msg));
    };
    timer_ = this->create_wall_timer(1s, callback);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state) {
    pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& state) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  std::string output_topic_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Node that produces messages.
struct Controller : public rclcpp::Node {
  Controller(const std::string& name, const std::string& output)
      : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Float64>(output, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type>
        captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub, this]() -> void {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr) {
        return;
      }
      static double command{0.0};
      std_msgs::msg::Float64::UniquePtr msg =
          std::make_unique<std_msgs::msg::Float64>();
      msg->data = 0.1 * std::sin(this->get_clock()->now().nanoseconds());
      printf("Published message with value: %f, and address: 0x%" PRIXPTR "\n",
             msg->data, reinterpret_cast<uintptr_t>(msg.get()));
      pub_ptr->publish(std::move(msg));
    };
    timer_ = this->create_wall_timer(4ms, callback);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Node that consumes messages.
struct Consumer : public rclcpp::Node {
  Consumer(const std::string& name, const std::string& input)
      : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    joint_states_node_ =
        rclcpp::Node::make_shared("joint_states_node", rclcpp::NodeOptions());

    joint_states_ = std::make_unique<sensor_msgs::msg::JointState>();
    joint_states_->name.push_back("joint1");
    joint_states_->position.push_back(0.0);

    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
    pub_ = joint_states_node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states2", qos);

    // Create a subscription on the input topic which prints on receipt of new
    // messages.
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
        input, 10, [this](std_msgs::msg::Float64::UniquePtr msg) {
          printf(" Received message with value: %f, and address: 0x%" PRIXPTR
                 "\n",
                 msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));

          sensor_msgs::msg::JointState out;
          out.header.stamp = this->get_clock()->now();
          out.name.push_back("joint1");
          out.position.push_back(msg->data);
          pub_->publish(out);
        });
  }

  double counter_{0.0};
  double joint_value_{0.0};
  sensor_msgs::msg::JointState::UniquePtr joint_states_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Node::SharedPtr joint_states_node_;
};

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

SOL_BASE_CLASSES(test::Talker, rclcpp::Node);
SOL_BASE_CLASSES(test::Consumer, rclcpp::Node);
SOL_BASE_CLASSES(test::Controller, rclcpp::Node);
SOL_DERIVED_CLASSES(rclcpp::Node, test::Talker, test::Consumer,
                    test::Controller);

namespace rclcpp_lua {

class Loader {
 public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr load_library(
      const std::string& library_path, const std::string& class_name,
      const rclcpp::NodeOptions& options) {
    auto logger = rclcpp::get_logger("Loader");
    auto loader = std::make_shared<class_loader::ClassLoader>(library_path);
    loaders_.push_back(loader);
    auto classes =
        loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (auto clazz : classes) {
      RCLCPP_INFO(logger, "Instantiate class %s", clazz.c_str());
      auto node_factory =
          loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      auto wrapper = node_factory->create_node_instance(options);
      node_wrappers_.push_back(wrapper);
      auto node = wrapper.get_node_base_interface();
      return node;
    }

    // auto node_factory =
    //     loader->createInstance<rclcpp_components::NodeFactory>(class_name);

    // auto wrapper = node_factory->create_node_instance(options);
    // node_wrappers_.push_back(wrapper);
    // auto node = wrapper.get_node_base_interface();
    // return node;
  }

 private:
  std::vector<std::shared_ptr<class_loader::ClassLoader>> loaders_;
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers_;
};

sol::table register_rclcpp_lua(sol::this_state L) {
  sol::state_view lua(L);
  sol::table module = lua.create_table();

  module.set_function("init", []() { rclcpp::init(0, nullptr); });
  module.set_function("shutdown", []() { rclcpp::shutdown(); });

  module.new_usertype<test::LifecycleController>(
      "LifecycleController",
      sol::factories([](const rclcpp::NodeOptions& options) {
        return std::make_shared<test::LifecycleController>(options);
      }),
      "configure", [](test::LifecycleController& self) { self.configure(); },
      "activate", [](test::LifecycleController& self) { self.activate(); },
      "get_node_base_interface",
      &test::LifecycleController::get_node_base_interface);

  module.new_usertype<test::Controller>(
      "Controller",
      sol::factories([](const std::string& name, const std::string& output) {
        return std::make_shared<test::Controller>(name, output);
      }),
      sol::base_classes, sol::bases<rclcpp::Node>());

  module.new_usertype<test::Consumer>(
      "Consumer",
      sol::factories([](const std::string& name, const std::string& input) {
        return std::make_shared<test::Consumer>(name, input);
      }),
      sol::base_classes, sol::bases<rclcpp::Node>());

  sol::usertype<Loader> loader = module.new_usertype<Loader>(
      "Loader", "load_library", &Loader::load_library);

  sol::usertype<rclcpp::node_interfaces::NodeBaseInterface>
      node_base_interface =
          module.new_usertype<rclcpp::node_interfaces::NodeBaseInterface>(
              "NodeBaseInterface", sol::no_constructor);

  sol::usertype<rclcpp::Node> node =
      module.new_usertype<rclcpp::Node>("Node", "get_node_base_interface",
                                        &rclcpp::Node::get_node_base_interface);

  sol::usertype<rclcpp::NodeOptions> node_options =
      module.new_usertype<rclcpp::NodeOptions>(
          "NodeOptions", "use_intra_process_comms",
          sol::overload(
              [](rclcpp::NodeOptions& options) {
                return options.use_intra_process_comms();
              },
              [](rclcpp::NodeOptions& options, bool use_intra_process_comms) {
                return options.use_intra_process_comms(use_intra_process_comms);
              }));

  sol::table executors = lua.create_table();
  sol::usertype<rclcpp::executors::SingleThreadedExecutor>
      single_threaded_executor =
          executors.new_usertype<rclcpp::executors::SingleThreadedExecutor>(
              "SingleThreadedExecutor");
  single_threaded_executor["spin"] =
      &rclcpp::executors::SingleThreadedExecutor::spin;
  // [](rclcpp::executors::SingleThreadedExecutor& exec) { exec.spin(); };

  single_threaded_executor["add_node"] = sol::overload(
      [](rclcpp::executors::SingleThreadedExecutor& exec,
         std::shared_ptr<rclcpp::Node> node_ptr,
         bool notify = true) { exec.add_node(node_ptr, notify); },
      [](rclcpp::executors::SingleThreadedExecutor& exec,
         std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_ptr,
         bool notify = true) { exec.add_node(node_ptr, notify); });

  module["executors"] = executors;

  sol::table test = lua.create_table();
  sol::usertype<test::Talker> talker = test.new_usertype<test::Talker>(
      "Talker", sol::factories([](rclcpp::NodeOptions& options) {
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
