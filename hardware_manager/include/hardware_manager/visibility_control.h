#ifndef HARDWARE_MANAGER__VISIBILITY_CONTROL_H_
#define HARDWARE_MANAGER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HARDWARE_MANAGER_EXPORT __attribute__ ((dllexport))
    #define HARDWARE_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define HARDWARE_MANAGER_EXPORT __declspec(dllexport)
    #define HARDWARE_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef HARDWARE_MANAGER_BUILDING_LIBRARY
    #define HARDWARE_MANAGER_PUBLIC HARDWARE_MANAGER_EXPORT
  #else
    #define HARDWARE_MANAGER_PUBLIC HARDWARE_MANAGER_IMPORT
  #endif
  #define HARDWARE_MANAGER_PUBLIC_TYPE HARDWARE_MANAGER_PUBLIC
  #define HARDWARE_MANAGER_LOCAL
#else
  #define HARDWARE_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define HARDWARE_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define HARDWARE_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define HARDWARE_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HARDWARE_MANAGER_PUBLIC
    #define HARDWARE_MANAGER_LOCAL
  #endif
  #define HARDWARE_MANAGER_PUBLIC_TYPE
#endif

#endif  // HARDWARE_MANAGER__VISIBILITY_CONTROL_H_