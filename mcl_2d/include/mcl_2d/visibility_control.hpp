#ifndef MCL2D_VISIBILITY_CONTROL_HPP_
#define MCL2D_VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MCL2D_CORE_EXPORT __attribute__ ((dllexport))
    #define MCL2D_CORE_IMPORT __attribute__ ((dllimport))
  #else
    #define MCL2D_CORE_EXPORT __declspec(dllexport)
    #define MCL2D_CORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MCL2D_CORE_BUILDING_LIBRARY
    #define MCL2D_CORE_PUBLIC MCL2D_CORE_EXPORT
  #else
    #define MCL2D_CORE_PUBLIC MCL2D_CORE_IMPORT
  #endif
  #define MCL2D_CORE_PUBLIC_TYPE MCL2D_CORE_PUBLIC
  #define MCL2D_CORE_LOCAL
#else
  #define MCL2D_CORE_EXPORT __attribute__ ((visibility("default")))
  #define MCL2D_CORE_IMPORT
  #if __GNUC__ >= 4
    #define MCL2D_CORE_PUBLIC __attribute__ ((visibility("default")))
    #define MCL2D_CORE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MCL2D_CORE_PUBLIC
    #define MCL2D_CORE_LOCAL
  #endif
  #define MCL2D_CORE_PUBLIC_TYPE
#endif

#endif  // MCL2D_VISIBILITY_CONTROL_HPP_