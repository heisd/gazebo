#ifndef AGV_CORRIDOR_LAYER__VISIBILITY_CONTROL_H_
#define AGV_CORRIDOR_LAYER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AGV_CORRIDOR_LAYER_EXPORT __attribute__ ((dllexport))
    #define AGV_CORRIDOR_LAYER_IMPORT __attribute__ ((dllimport))
  #else
    #define AGV_CORRIDOR_LAYER_EXPORT __declspec(dllexport)
    #define AGV_CORRIDOR_LAYER_IMPORT __declspec(dllimport)
  #endif
  #ifdef AGV_CORRIDOR_LAYER_BUILDING_LIBRARY
    #define AGV_CORRIDOR_LAYER_PUBLIC AGV_CORRIDOR_LAYER_EXPORT
  #else
    #define AGV_CORRIDOR_LAYER_PUBLIC AGV_CORRIDOR_LAYER_IMPORT
  #endif
  #define AGV_CORRIDOR_LAYER_PUBLIC_TYPE AGV_CORRIDOR_LAYER_PUBLIC
  #define AGV_CORRIDOR_LAYER_LOCAL
#else
  #define AGV_CORRIDOR_LAYER_EXPORT __attribute__ ((visibility("default")))
  #define AGV_CORRIDOR_LAYER_IMPORT
  #if __GNUC__ >= 4
    #define AGV_CORRIDOR_LAYER_PUBLIC __attribute__ ((visibility("default")))
    #define AGV_CORRIDOR_LAYER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AGV_CORRIDOR_LAYER_PUBLIC
    #define AGV_CORRIDOR_LAYER_LOCAL
  #endif
  #define AGV_CORRIDOR_LAYER_PUBLIC_TYPE
#endif

#endif  // AGV_CORRIDOR_LAYER__VISIBILITY_CONTROL_H_
