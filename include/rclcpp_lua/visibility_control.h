#ifndef RCLCPP_LUA__VISIBILITY_CONTROL_H_
#define RCLCPP_LUA__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RCLCPP_LUA_EXPORT __attribute__ ((dllexport))
    #define RCLCPP_LUA_IMPORT __attribute__ ((dllimport))
  #else
    #define RCLCPP_LUA_EXPORT __declspec(dllexport)
    #define RCLCPP_LUA_IMPORT __declspec(dllimport)
  #endif
  #ifdef RCLCPP_LUA_BUILDING_LIBRARY
    #define RCLCPP_LUA_PUBLIC RCLCPP_LUA_EXPORT
  #else
    #define RCLCPP_LUA_PUBLIC RCLCPP_LUA_IMPORT
  #endif
  #define RCLCPP_LUA_PUBLIC_TYPE RCLCPP_LUA_PUBLIC
  #define RCLCPP_LUA_LOCAL
#else
  #define RCLCPP_LUA_EXPORT __attribute__ ((visibility("default")))
  #define RCLCPP_LUA_IMPORT
  #if __GNUC__ >= 4
    #define RCLCPP_LUA_PUBLIC __attribute__ ((visibility("default")))
    #define RCLCPP_LUA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RCLCPP_LUA_PUBLIC
    #define RCLCPP_LUA_LOCAL
  #endif
  #define RCLCPP_LUA_PUBLIC_TYPE
#endif

#endif  // RCLCPP_LUA__VISIBILITY_CONTROL_H_
