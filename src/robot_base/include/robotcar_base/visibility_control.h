#ifndef ROBOTCAR_BASE__VISIBILITY_CONTROL_H_
#define ROBOTCAR_BASE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTCAR_BASE_EXPORT __attribute__ ((dllexport))
    #define ROBOTCAR_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTCAR_BASE_EXPORT __declspec(dllexport)
    #define ROBOTCAR_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTCAR_BASE_BUILDING_DLL
    #define ROBOTCAR_BASE_PUBLIC ROBOTCAR_BASE_EXPORT
  #else
    #define ROBOTCAR_BASE_PUBLIC ROBOTCAR_BASE_IMPORT
  #endif
  #define ROBOTCAR_BASE_PUBLIC_TYPE ROBOTCAR_BASE_PUBLIC
  #define ROBOTCAR_BASE_LOCAL
#else
  #define ROBOTCAR_BASE_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTCAR_BASE_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTCAR_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTCAR_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTCAR_BASE_PUBLIC
    #define ROBOTCAR_BASE_LOCAL
  #endif
  #define ROBOTCAR_BASE_PUBLIC_TYPE
#endif

#endif  // ROBOTCAR_BASE__VISIBILITY_CONTROL_H_ 