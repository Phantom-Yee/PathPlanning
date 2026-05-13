#pragma once

#ifdef _WIN32
#ifdef PATHPLANNING_LIBRARY
#define PATHPLANNING_API __declspec(dllexport)
#else
#define PATHPLANNING_API __declspec(dllimport)
#endif
#else
#define PATHPLANNING_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

// 返回 0 表示成功，非 0 表示失败
PATHPLANNING_API int generate_path_from_file(const char* input_path,
                                             const char* output_path,
                                             double robot_width,
                                             double optimize_dist,
                                             double step_size);

#ifdef __cplusplus
}
#endif
