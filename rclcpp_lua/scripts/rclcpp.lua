local rclcpp = require('librclcpp_lua')

-- local exec = get_executor()


-- print(exec)

-- local loader = rclcpp.Loader.new()

robot = rclcpp.KukaRsiHardware.new()
robot:init()

-- exec = rclcpp.executors.MultiThreadedExecutor.new()

print(exec)
-- print(robot)

cm = rclcpp.ControllerManager.new(robot, exec, "controller_manager");
cm:load_controller("ros_controllers", "ros_controllers::JointStateController", "joint_state_controller");
-- -- etasl_ros_controller = cm:load_controller(
-- --       "etasl_ros2_controllers", "etasl_ros2_controllers::EtaslRos2Controller", "etasl_ros2_controller");

-- -- start exec async spin

-- cm:configure()
-- cm:activate()





-- local exec = etasl.exec

-- local options = rclcpp.NodeOptions.new()

-- local talker = loader:load_library('/opt/ros/eloquent/lib/libtalker_component.so', 'composition::Talker', options)
-- local listener = loader:load_library('/opt/ros/eloquent/lib/liblistener_component.so', 'composition::Listener', options)

-- local lifecycle_options = rclcpp.NodeOptions.new():use_intra_process_comms(true)
-- local lifecycle_controller = rclcpp.LifecycleController.new(lifecycle_options)

-- local topic = 'command'
-- local controller = rclcpp.Controller.new("controller", topic)

-- local driver = rclcpp.Consumer.new("consumer", topic)

-- exec:add_node(lifecycle_controller:get_node_base_interface(), true)
-- exec:add_node(controller:get_node_base_interface(), true)
-- exec:add_node(driver:get_node_base_interface(), true)

-- print("added nodes to exec")

-- lifecycle_controller:configure()
-- lifecycle_controller:activate()


-- exec:add_node(talker, true)
-- exec:add_node(listener, true)



-- exec:spin()

-- rclcpp.shutdown()
