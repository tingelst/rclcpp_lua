#ifndef ETASL_DRIVER__VISIBILITY_CONTROL_H_
#define ETASL_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ETASL_DRIVER_EXPORT __attribute__ ((dllexport))
    #define ETASL_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define ETASL_DRIVER_EXPORT __declspec(dllexport)
    #define ETASL_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ETASL_DRIVER_BUILDING_LIBRARY
    #define ETASL_DRIVER_PUBLIC ETASL_DRIVER_EXPORT
  #else
    #define ETASL_DRIVER_PUBLIC ETASL_DRIVER_IMPORT
  #endif
  #define ETASL_DRIVER_PUBLIC_TYPE ETASL_DRIVER_PUBLIC
  #define ETASL_DRIVER_LOCAL
#else
  #define ETASL_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define ETASL_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define ETASL_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define ETASL_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ETASL_DRIVER_PUBLIC
    #define ETASL_DRIVER_LOCAL
  #endif
  #define ETASL_DRIVER_PUBLIC_TYPE
#endif

#endif  // ETASL_DRIVER__VISIBILITY_CONTROL_H_
