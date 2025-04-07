#ifndef RVIZ_MIN_MAX_CUR_OVERLAY__VISIBILITY_CONTROL_HPP_
#define RVIZ_MIN_MAX_CUR_OVERLAY__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RVIZ_MIN_MAX_CUR_OVERLAY_EXPORT __attribute__ ((dllexport))
    #define RVIZ_MIN_MAX_CUR_OVERLAY_IMPORT __attribute__ ((dllimport))
  #else
    #define RVIZ_MIN_MAX_CUR_OVERLAY_EXPORT __declspec(dllexport)
    #define RVIZ_MIN_MAX_CUR_OVERLAY_IMPORT __declspec(dllimport)
  #endif
  #ifdef RVIZ_MIN_MAX_CUR_OVERLAY_BUILDING_DLL
    #define RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC RVIZ_MIN_MAX_CUR_OVERLAY_EXPORT
  #else
    #define RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC RVIZ_MIN_MAX_CUR_OVERLAY_IMPORT
  #endif
  #define RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC_TYPE RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC
  #define RVIZ_MIN_MAX_CUR_OVERLAY_LOCAL
#else
  #define RVIZ_MIN_MAX_CUR_OVERLAY_EXPORT __attribute__ ((visibility("default")))
  #define RVIZ_MIN_MAX_CUR_OVERLAY_IMPORT
  #if __GNUC__ >= 4
    #define RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC __attribute__ ((visibility("default")))
    #define RVIZ_MIN_MAX_CUR_OVERLAY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC
    #define RVIZ_MIN_MAX_CUR_OVERLAY_LOCAL
  #endif
  #define RVIZ_MIN_MAX_CUR_OVERLAY_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_MIN_MAX_CUR_OVERLAY__VISIBILITY_CONTROL_HPP_ 