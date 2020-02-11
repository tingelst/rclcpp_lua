local rclcpp = require('librclcpp_lua')

rclcpp.init()

local loader = rclcpp.Loader.new()

local exec = rclcpp.executors.SingleThreadedExecutor.new()

local options = rclcpp.NodeOptions.new()

local talker = loader:load_library('/opt/ros/eloquent/lib/libtalker_component.so', 'composition::Talker', options)
local listener = loader:load_library('/opt/ros/eloquent/lib/liblistener_component.so', 'composition::Listener', options)

exec:add_node(talker, true)
exec:add_node(listener, true)

exec:spin()

rclcpp.shutdown()
