local rclcpp = require('librclcpp_lua')

rclcpp.init()

local exec = rclcpp.executors.SingleThreadedExecutor.new()
print(exec)

local options = rclcpp.NodeOptions.new()

local talker = rclcpp.test.Talker.new(options)

print(talker:five())

exec:add_node(talker, true)

exec:spin()

rclcpp.shutdown()
