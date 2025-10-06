#pragma once

#include "libxr.hpp"

// STL
#include <array>
#include <cstdint>

class CameraBase
{
 public:
  struct CameraInfo
  {
    // 基本图像信息
    uint32_t width;   // 像素
    uint32_t height;  // 像素
    uint32_t step;    // 每行字节数 (bytes per row)
    LibXR::MicrosecondTimestamp timestamp;
    char encoding[8];  // 例如 "rgb8", "bgr8", "mono8"

    // 相机内参与标定（全部使用行优先 Row-major）
    // 1) 内参矩阵 K = [ fx  0 cx ; 0 fy cy ; 0 0 1 ]
    std::array<double, 9> camera_matrix;  // 3x3

    // 2) 畸变参数（常见 plumb_bob: k1,k2,t1,t2,k3）
    // 若模型不同，可只用前 N 项，其余置 0
    std::array<double, 5> distortion_coefficients;  // 5

    // 3) 立体/矫正后的旋转矩阵 R
    std::array<double, 9> rectification_matrix;  // 3x3

    // 4) 投影矩阵 P (通常为 3x4)
    // P = [ fx'  0   cx'  Tx ;
    //        0  fy' cy'  Ty ;
    //        0   0   1    0 ]
    std::array<double, 12> projection_matrix;  // 3x4

    // 5) 畸变模型名（如 "plumb_bob", "rational_polynomial"...）
    std::array<char, 32> distortion_model;  // 32>
  };
};
