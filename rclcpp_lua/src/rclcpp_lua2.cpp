#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lua/rclcpp_lua.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "std_msgs/msg/string.hpp"

#include "hardware_manager/hardware_manager.hpp"
#include "controller_manager/controller_manager.hpp"
#include "kuka_rsi_hardware/kuka_rsi_hardware.hpp"

#include "etasl_controller/etasl_controller.hpp"

#include <chrono>
#include <cinttypes>

#include <cstdio>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include <cmath>

namespace test
{
using namespace std::chrono_literals;

class LifecycleController : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_LUA_PUBLIC
  explicit LifecycleController(const rclcpp::NodeOptions& options)
    : LifecycleNode("LifecycleController", rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    output_topic_ = this->declare_parameter("output_topic", "command");
    input_topic_ = this->declare_parameter("input_topic", "state");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state)
  {
    // Create a publisher on the output topic.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::Float64>(output_topic_, qos);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub, this]() -> void {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr)
      {
        return;
      }
      static double command{ 0.0 };
      std_msgs::msg::Float64::UniquePtr msg = std::make_unique<std_msgs::msg::Float64>();
      msg->data = 0.1 * std::sin(this->get_clock()->now().nanoseconds());
      RCLCPP_INFO(this->get_logger(), "Published command message with and address: 0x%" PRIXPTR,
                  reinterpret_cast<uintptr_t>(msg.get()));
      pub_ptr->publish(std::move(msg));
    };
    timer_ = this->create_wall_timer(1s, callback);
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Received joint state message with address: 0x%" PRIXPTR,
                      reinterpret_cast<std::uintptr_t>(msg.get()));
        });
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Node that produces messages.
struct Controller : public rclcpp::Node
{
  Controller(const std::string& name, const std::string& output)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a publisher on the output topic.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::Float64>(output, qos);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub, this]() -> void {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr)
      {
        return;
      }
      static double command{ 0.0 };
      std_msgs::msg::Float64::UniquePtr msg = std::make_unique<std_msgs::msg::Float64>();
      msg->data = 0.1 * std::sin(this->get_clock()->now().nanoseconds());
      RCLCPP_INFO(this->get_logger(), "Published message with and address: 0x%" PRIXPTR,
                  reinterpret_cast<uintptr_t>(msg.get()));
      pub_ptr->publish(std::move(msg));
    };
    timer_ = this->create_wall_timer(4ms, callback);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

struct JointStatePublisher : public rclcpp::Node
{
  JointStatePublisher(const std::string& name, const std::string& input, const std::string& output)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output, qos);

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          sensor_msgs::msg::JointState out = *msg;
          pub_->publish(out);
          RCLCPP_INFO(this->get_logger(), "Received joint state message with address: 0x%" PRIXPTR,
                      reinterpret_cast<std::uintptr_t>(msg.get()));
        });
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

struct Driver : public rclcpp::Node
{
  Driver(const std::string& name, const std::string& input, const std::string& output)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output, qos);
    sub_ = this->create_subscription<std_msgs::msg::Float64>(input, 10, [this](std_msgs::msg::Float64::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received command message with address: 0x%" PRIXPTR,
                  reinterpret_cast<std::uintptr_t>(msg.get()));

      joint_states_ = std::make_unique<sensor_msgs::msg::JointState>();
      joint_states_->header.stamp = this->get_clock()->now();
      joint_states_->name.push_back("joint1");
      joint_states_->position.push_back(msg->data);
      RCLCPP_INFO(this->get_logger(), "Published joint state message with address: 0x%" PRIXPTR,
                  reinterpret_cast<uintptr_t>(joint_states_.get()));
      pub_->publish(std::move(joint_states_));
    });
  }

  double counter_{ 0.0 };
  double joint_value_{ 0.0 };
  sensor_msgs::msg::JointState::UniquePtr joint_states_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Node::SharedPtr joint_states_node_;
};

class Talker : public rclcpp::Node
{
public:
  RCLCPP_LUA_PUBLIC
  explicit Talker(const rclcpp::NodeOptions& options) : Node("talker", options)
  {
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

SOL_BASE_CLASSES(kuka_rsi_hardware::KukaRsiHardware, hardware_interface::RobotHardware);

SOL_DERIVED_CLASSES(hardware_interface::RobotHardware, kuka_rsi_hardware::KukaRsiHardware);

SOL_BASE_CLASSES(rclcpp::executors::SingleThreadedExecutor, rclcpp::executor::Executor);

SOL_BASE_CLASSES(rclcpp::executors::MultiThreadedExecutor, rclcpp::executor::Executor);

SOL_DERIVED_CLASSES(rclcpp::executor::Executor, rclcpp::executors::SingleThreadedExecutor,
                    rclcpp::executors::MultiThreadedExecutor);

SOL_BASE_CLASSES(test::Talker, rclcpp::Node);
SOL_BASE_CLASSES(test::JointStatePublisher, rclcpp::Node);
SOL_BASE_CLASSES(test::Driver, rclcpp::Node);

SOL_BASE_CLASSES(test::Controller, rclcpp::Node);
SOL_DERIVED_CLASSES(rclcpp::Node, test::Talker, test::Driver, test::Controller, test::JointStatePublisher);

namespace rclcpp_lua
{
class Loader
{
public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr load_library(const std::string& library_path,
                                                                     const std::string& class_name,
                                                                     const rclcpp::NodeOptions& options)
  {
    auto logger = rclcpp::get_logger("Loader");
    auto loader = std::make_shared<class_loader::ClassLoader>(library_path);
    loaders_.push_back(loader);
    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (auto clazz : classes)
    {
      RCLCPP_INFO(logger, "Instantiate class %s", clazz.c_str());
      auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
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

void register_rclcpp_lua(sol::state_view lua)
{
  sol::table module = lua.create_table();

  module.set_function("init", []() { rclcpp::init(0, nullptr); });
  module.set_function("shutdown", []() { rclcpp::shutdown(); });

  module.new_usertype<hardware_interface::RobotHardware>(
      "RobotHardware", "init", &hardware_interface::RobotHardware::init, "read",
      &hardware_interface::RobotHardware::read, "write", &hardware_interface::RobotHardware::write);

  module.new_usertype<etasl_controller::EtaslController>(
      "EtaslController", sol::factories([]() { return std::make_shared<etasl_controller::EtaslController>(); }),
      "get_controller_interface",
      [](std::shared_ptr<etasl_controller::EtaslController> etasl_controller) {
        return std::static_pointer_cast<controller_interface::ControllerInterface>(etasl_controller);
      },
      "add_input_scalar", &etasl_controller::EtaslController::add_input_scalar,
      "add_output_scalar", &etasl_controller::EtaslController::add_output_scalar
      );

  module.new_usertype<kuka_rsi_hardware::KukaRsiHardware>(
      "KukaRsiHardware", sol::factories([]() { return std::make_shared<kuka_rsi_hardware::KukaRsiHardware>(); }),
      "init", &kuka_rsi_hardware::KukaRsiHardware::init, "read", &kuka_rsi_hardware::KukaRsiHardware::read, "write",
      &kuka_rsi_hardware::KukaRsiHardware::write, "get_robot_hardware",
      [](std::shared_ptr<kuka_rsi_hardware::KukaRsiHardware> hw) {
        return std::static_pointer_cast<hardware_interface::RobotHardware>(hw);
      });

  module.new_usertype<hardware_manager::RobotHardwareManager>(
      "RobotHardwareManager", sol::factories([](const std::string& name) {
        return std::make_shared<hardware_manager::RobotHardwareManager>(name);
      }),
      "load_hardware", &hardware_manager::RobotHardwareManager::load_hardware);

  module.new_usertype<controller_manager::ControllerManager>(
      "ControllerManager",
      sol::factories([](std::shared_ptr<hardware_interface::RobotHardware> hw,
                        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor, const std::string& name) {
        return std::make_shared<controller_manager::ControllerManager>(hw, executor, name);
      }),
      "load_controller", &controller_manager::ControllerManager::load_controller, "configure",
      &controller_manager::ControllerManager::configure, "activate", &controller_manager::ControllerManager::activate,
      "add_controller",
      [](std::shared_ptr<controller_manager::ControllerManager> cm,
         std::shared_ptr<controller_interface::ControllerInterface> controller,
         std::string controller_name) { cm->add_controller(controller, controller_name); });

  module.new_usertype<test::LifecycleController>(
      "LifecycleController", sol::factories([](const rclcpp::NodeOptions& options) {
        return std::make_shared<test::LifecycleController>(options);
      }),
      "configure", [](test::LifecycleController& self) { self.configure(); }, "activate",
      [](test::LifecycleController& self) { self.activate(); }, "get_node_base_interface",
      &test::LifecycleController::get_node_base_interface);

  module.new_usertype<test::Controller>("Controller",
                                        sol::factories([](const std::string& name, const std::string& output) {
                                          return std::make_shared<test::Controller>(name, output);
                                        }),
                                        sol::base_classes, sol::bases<rclcpp::Node>());

  module.new_usertype<test::JointStatePublisher>(
      "JointStatePublisher",
      sol::factories([](const std::string& name, const std::string& input, const std::string& output) {
        return std::make_shared<test::JointStatePublisher>(name, input, output);
      }),
      sol::base_classes, sol::bases<rclcpp::Node>());

  module.new_usertype<test::Driver>(
      "Driver", sol::factories([](const std::string& name, const std::string& input, const std::string& output) {
        return std::make_shared<test::Driver>(name, input, output);
      }),
      sol::base_classes, sol::bases<rclcpp::Node>());

  sol::usertype<Loader> loader = module.new_usertype<Loader>("Loader", "load_library", &Loader::load_library);

  sol::usertype<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface =
      module.new_usertype<rclcpp::node_interfaces::NodeBaseInterface>("NodeBaseInterface", sol::no_constructor);

  sol::usertype<rclcpp::Node> node =
      module.new_usertype<rclcpp::Node>("Node", "get_node_base_interface", &rclcpp::Node::get_node_base_interface);

  sol::usertype<rclcpp::NodeOptions> node_options = module.new_usertype<rclcpp::NodeOptions>(
      "NodeOptions", "use_intra_process_comms",
      sol::overload([](rclcpp::NodeOptions& options) { return options.use_intra_process_comms(); },
                    [](rclcpp::NodeOptions& options, bool use_intra_process_comms) {
                      return options.use_intra_process_comms(use_intra_process_comms);
                    }));

  sol::table executors = lua.create_table();
  sol::usertype<rclcpp::executor::Executor> executor = executors.new_usertype<rclcpp::executor::Executor>("Executor");

  sol::usertype<rclcpp::executors::SingleThreadedExecutor> single_threaded_executor =
      executors.new_usertype<rclcpp::executors::SingleThreadedExecutor>(
          "SingleThreadedExecutor",
          sol::factories([]() { return std::make_shared<rclcpp::executors::SingleThreadedExecutor>(); }));
  single_threaded_executor["spin"] = &rclcpp::executors::SingleThreadedExecutor::spin;
  // [](rclcpp::executors::SingleThreadedExecutor& exec) { exec.spin(); };
  single_threaded_executor["add_node"] =
      sol::overload([](rclcpp::executors::SingleThreadedExecutor& exec, std::shared_ptr<rclcpp::Node> node_ptr,
                       bool notify = true) { exec.add_node(node_ptr, notify); },
                    [](rclcpp::executors::SingleThreadedExecutor& exec,
                       std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_ptr,
                       bool notify = true) { exec.add_node(node_ptr, notify); });

  sol::usertype<rclcpp::executors::MultiThreadedExecutor> multi_threaded_executor =
      executors.new_usertype<rclcpp::executors::MultiThreadedExecutor>(
          "MultiThreadedExecutor",
          sol::factories([]() { return std::make_shared<rclcpp::executors::MultiThreadedExecutor>(); }));
  multi_threaded_executor["spin"] = &rclcpp::executors::MultiThreadedExecutor::spin;

  multi_threaded_executor["add_node"] =
      sol::overload([](rclcpp::executors::MultiThreadedExecutor& exec, std::shared_ptr<rclcpp::Node> node_ptr,
                       bool notify = true) { exec.add_node(node_ptr, notify); },
                    [](rclcpp::executors::MultiThreadedExecutor& exec,
                       std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_ptr,
                       bool notify = true) { exec.add_node(node_ptr, notify); });

  module["executors"] = executors;

  sol::table test = lua.create_table();
  sol::usertype<test::Talker> talker = test.new_usertype<test::Talker>(
      "Talker", sol::factories([](rclcpp::NodeOptions& options) { return std::make_shared<test::Talker>(options); }),
      sol::base_classes, sol::bases<rclcpp::Node>());
  module["test"] = test;

  lua["rclcpp"] = module;

  // return module;
}

}  // namespace rclcpp_lua