-- local rclcpp2 = require('librclcpp_lua')
print(rclcpp)

exec = get_executor()

robot = rclcpp.KukaRsiHardware.new()
local ret = robot:init()
print(ret)

robot_manager = get_robot_manager()
robot_manager:set_robot(robot:get_robot_hardware())

cm = rclcpp.ControllerManager.new(robot:get_robot_hardware(), exec, "controller_manager")
cm:load_controller("joint_state_controller", "joint_state_controller")
-- cm:load_controller("test_controller_01", "test_controller")
-- cm:load_controller("etasl_ros2_controllers", "etasl_ros2_controllers::EtaslRos2Controller", "etasl_ros2_controller")

cm:configure()
cm:activate()

-- local options = rclcpp.NodeOptions.new()

-- -- local talker = loader:load_library('/opt/ros/eloquent/lib/libtalker_component.so', 'composition::Talker', options)
-- -- local listener = loader:load_library('/opt/ros/eloquent/lib/liblistener_component.so', 'composition::Listener', options)

-- local lifecycle_options = rclcpp.NodeOptions.new():use_intra_process_comms(true)
-- local lifecycle_controller = rclcpp.LifecycleController.new(lifecycle_options)

-- local topic = 'command'
-- local driver = rclcpp.Driver.new("driver", "command", "state")
-- local joint_state_publisher = rclcpp.JointStatePublisher.new("joint_state_publisher", "state", "joint_states")

-- exec:add_node(lifecycle_controller:get_node_base_interface(), true)
-- exec:add_node(driver:get_node_base_interface(), true)
-- -- exec:add_node(joint_state_publisher:get_node_base_interface(), true)

-- lifecycle_controller:configure()
-- lifecycle_controller:activate()


