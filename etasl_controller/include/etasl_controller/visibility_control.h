#ifndef ETASL_CONTROLLER__VISIBILITY_CONTROL_H_
#define ETASL_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ETASL_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define ETASL_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define ETASL_CONTROLLER_EXPORT __declspec(dllexport)
    #define ETASL_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ETASL_CONTROLLER_BUILDING_LIBRARY
    #define ETASL_CONTROLLER_PUBLIC ETASL_CONTROLLER_EXPORT
  #else
    #define ETASL_CONTROLLER_PUBLIC ETASL_CONTROLLER_IMPORT
  #endif
  #define ETASL_CONTROLLER_PUBLIC_TYPE ETASL_CONTROLLER_PUBLIC
  #define ETASL_CONTROLLER_LOCAL
#else
  #define ETASL_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define ETASL_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define ETASL_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define ETASL_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ETASL_CONTROLLER_PUBLIC
    #define ETASL_CONTROLLER_LOCAL
  #endif
  #define ETASL_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // ETASL_CONTROLLER__VISIBILITY_CONTROL_H_
