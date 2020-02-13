require("libexpressiongraph_context_lua")
ctx=Context()
ctx:addType("robot")
ctx:addType("feature")
time = ctx:getScalarExpr("time")


local rclcpp = require('librclcpp_lua')

-- rclcpp.init()

local loader = rclcpp.Loader.new()

-- local exec = rclcpp.executors.SingleThreadedExecutor.new()

local exec = etasl.exec

local options = rclcpp.NodeOptions.new()

local talker = loader:load_library('/opt/ros/eloquent/lib/libtalker_component.so', 'composition::Talker', options)
local listener = loader:load_library('/opt/ros/eloquent/lib/liblistener_component.so', 'composition::Listener', options)

local lifecycle_options = rclcpp.NodeOptions.new():use_intra_process_comms(true)
local lifecycle_controller = rclcpp.LifecycleController.new(lifecycle_options)

local topic = 'command'
-- local controller = rclcpp.Controller.new("controller", topic)

local driver = rclcpp.Consumer.new("consumer", topic)

exec:add_node(lifecycle_controller:get_node_base_interface(), true)
-- exec:add_node(controller:get_node_base_interface(), true)
exec:add_node(driver:get_node_base_interface(), true)

print("added nodes to exec")

lifecycle_controller:configure()
lifecycle_controller:activate()


-- exec:add_node(talker, true)
-- exec:add_node(listener, true)



exec:spin()

-- rclcpp.shutdown()
