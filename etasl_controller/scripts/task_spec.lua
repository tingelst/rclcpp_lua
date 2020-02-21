-- require('expressiongraph')


local u = UrdfExpr()
u:readFromFile(
    "/home/lars/etasl_ros2_control_ws/install/kuka_kr6_support/share/kuka_kr6_support/urdf/kr6r900sixx.urdf")
u:addTransform("ee", "tool0", "base_link")

local r = u:getExpressions(ctx)

print(r.ee)

-- The transformation of the robot mounting plate frame with respect to the robot base frame
robot_ee = r.ee
-- The name of the robot joints
robot_joints = {
    "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"
}

j1 = ctx:getScalarExpr(robot_joints[1])
j2 = ctx:getScalarExpr(robot_joints[2])
j3 = ctx:getScalarExpr(robot_joints[3])
j4 = ctx:getScalarExpr(robot_joints[4])
j5 = ctx:getScalarExpr(robot_joints[5])
j6 = ctx:getScalarExpr(robot_joints[6])

function BoxConstraint(arg)
    local HUGE_VALUE = 1E20
    local msg=  "   BoxConstraint{\n"..
                "       context=...,   [context to use]\n"..
                "       var_name=..,    [variable of which you want to constrain the velocity]\n"..
                "       lower=..,       [optional, default -large_value][lower velocity bound]\n".. 
                "       upper=..,       [optional, default +large_value][upper velocity bound]\n".. 
                "       tags=..         [optional, default ''][tags for this constraint]\n"..
                "   }\n"
    if arg=='help' then
        print(msg);
        return
    end
    -- namedcheck({"context","var_name","lower","upper","tags"},
    --            {arg.context, arg.var_name, arg.lower, arg.upper,arg.tags},
    --            {"Context","string","?number","?number","?string"},msg)
    if arg.lower == nil and arg.upper == 0 then
        error("lower and upper cannot be both unspecified\n\n"..msg,2)
    end
    if arg.lower == nil then
        arg.lower = -HUGE_VALUE
    end
    if arg.upper == nil then
        arg.upper = HUGE_VALUE
    end
    if arg.tags == nil then
        arg.tags = ''
    end
    local ndx = ctx:getScalarNdx(arg.var_name)
    if ndx==-1 then
        error("unknown variable name("..arg.var_name.." given\n\b"..msg,2)
    end
    local retval
    retval = ctx:addBoxConstraint(arg.var_name, ndx, arg.lower, arg.upper,arg.tags)
    if retval~=0 then
        error("addBoxConstraint failed: could not add the constraint\n\n"..msg)
    end
end


maxvel = 0.6
for i = 1, #robot_joints do
    BoxConstraint{
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

local meas = j1
local model = j1
local controller = 'proportional'
local target = 0.5
local priority = 2
local K = constant(4)


ctx:addInequalityConstraint('c1', model, meas, target, target, controller, controller, 2, K)

print(ctx)