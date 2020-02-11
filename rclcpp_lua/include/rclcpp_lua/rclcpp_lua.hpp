#ifndef RCLCPP_LUA__RCLCPP_LUA_HPP_
#define RCLCPP_LUA__RCLCPP_LUA_HPP_

#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "rclcpp_lua/visibility_control.h"

namespace rclcpp_lua {
sol::table register_rclcpp_lua(sol::this_state L);

} 

extern "C" RCLCPP_LUA_PUBLIC int luaopen_librclcpp_lua(lua_State* L);

#endif  // RCLCPP_LUA__RCLCPP_LUA_HPP_
