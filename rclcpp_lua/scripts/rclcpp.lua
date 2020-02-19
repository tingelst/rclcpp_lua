-- local rclcpp2 = require('librclcpp_lua')
-- print(rclcpp)

exec = get_executor()


rm = rclcpp.RobotHardwareManager.new("robot_hardware_manager")
robot = rm:load_hardware("kuka_rsi_hardware", "kuka_rsi_hardware")
robot:init()


etasl_controller = rclcpp.EtaslController.new()

cm = rclcpp.ControllerManager.new(robot, exec, "controller_manager")
cm:load_controller("joint_state_controller", "joint_state_controller")
cm:add_controller(etasl_controller:get_controller_interface(), "etasl_controller")


-- etasl_controller:set_joint_names("input_scalar")
etasl_controller:add_input_scalar("input_scalar")
etasl_controller:add_output_scalar("output_scalar")

print("main")
etasl_controller:read_task_specification_string("e = require('expressiongraph')")
etasl_controller:read_task_specification_string("v = e.Vector(1,2,3)")
etasl_controller:read_task_specification_string("print(v)")
etasl_controller:read_task_specification_string("print(ctx)")
etasl_controller:read_task_specification_string("print(Frame.new(v))")
etasl_controller:read_task_specification_string("constant = e.constant")
etasl_controller:read_task_specification_string("print(constant(v))")



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


