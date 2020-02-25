-- require('expressiongraph')
local constraints = require("constraints.constraints")

local BoxConstraint2 = constraints.BoxConstraint


-- print(b.getLower())

local u = UrdfExpr()
u:readFromFile("/home/lars/etasl_ros2_control_ws/install/kuka_kr6_support/share/kuka_kr6_support/urdf/kr6r900sixx.urdf")
u:addTransform("ee", "tool0", "base_link")

local r = u:getExpressions(ctx)

-- print(r.ee)

-- The transformation of the robot mounting plate frame with respect to the robot base frame
robot_ee = r.ee
-- The name of the robot joints
robot_joints = {
    "joint_a1",
    "joint_a2",
    "joint_a3",
    "joint_a4",
    "joint_a5",
    "joint_a6"
}

j1 = ctx:getScalarExpr(robot_joints[1])
j2 = ctx:getScalarExpr(robot_joints[2])
j3 = ctx:getScalarExpr(robot_joints[3])
j4 = ctx:getScalarExpr(robot_joints[4])
j5 = ctx:getScalarExpr(robot_joints[5])
j6 = ctx:getScalarExpr(robot_joints[6])

unames = u:getAllJointNames()
print(unames)

print(u:getAllJointExpressions())

print(u.poslimits)
u.poslimits = false
print(u.poslimits)

maxvel = 0.6
for i = 1, #robot_joints do
    BoxConstraint {
        context = ctx,
        var_name = robot_joints[i],
        lower = -maxvel,
        upper = maxvel
    }
end

-- Constraint{
--     context = ctx,
--     name = "c1",
--     expr = j1,
--     target = tgt1 * deg2rad,
--     priority = 2,
--     K = 4
-- }

-- deg2rad = math.pi / 180.0
tgt1 = ctx:createInputChannelScalar("input_scalar", 1.23456)

local meas = j1
local model = j1
local controller = "proportional"
target = 0
local priority = 2
local K = constant(4)
-- ctx:addInequalityConstraint('c1', model, meas, target, target, controller, controller, 2, K)

ctx:addConstraint("c2", j2 + tgt1, 4, 1, 2)
-- ctx:addInequalityConstraint('c3', j3, 0.0, 4, 1.234, 4, 1.0, 2)

-- inline int addInequalityConstraint(Context::Ptr            ctx,
--                                    const std::string&      name,
--                                    Expression<double>::Ptr expr,
--                                    double                  target_lower,
--                                    double                  K_lower,
--                                    double                  target_upper,
--                                    double                  K_upper,
--                                    double                  weight,
--                                    int                     priority) {
