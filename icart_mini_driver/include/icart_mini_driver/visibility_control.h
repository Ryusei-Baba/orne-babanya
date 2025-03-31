#ifndef ICART_MINI_DRIVER__VISIBILITY_CONTROL_H_
#define ICART_MINI_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ICART_MINI_DRIVER_EXPORT __attribute__ ((dllexport))
    #define ICART_MINI_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define ICART_MINI_DRIVER_EXPORT __declspec(dllexport)
    #define ICART_MINI_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ICART_MINI_DRIVER_BUILDING_LIBRARY
    #define ICART_MINI_DRIVER_PUBLIC ICART_MINI_DRIVER_EXPORT
  #else
    #define ICART_MINI_DRIVER_PUBLIC ICART_MINI_DRIVER_IMPORT
  #endif
  #define ICART_MINI_DRIVER_PUBLIC_TYPE ICART_MINI_DRIVER_PUBLIC
  #define ICART_MINI_DRIVER_LOCAL
#else
  #define ICART_MINI_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define ICART_MINI_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define ICART_MINI_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define ICART_MINI_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ICART_MINI_DRIVER_PUBLIC
    #define ICART_MINI_DRIVER_LOCAL
  #endif
  #define ICART_MINI_DRIVER_PUBLIC_TYPE
#endif

#endif  // ICART_MINI_DRIVER__VISIBILITY_CONTROL_H_