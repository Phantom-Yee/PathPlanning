#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// 返回 0 表示成功，非 0 表示失败
int generate_path_from_file(const char* input_path,
                            const char* output_path,
                            double robot_width,
                            double optimize_dist,
                            double step_size);

#ifdef __cplusplus
}
#endif
