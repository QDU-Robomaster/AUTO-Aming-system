#pragma once
/**
 * @file
 * @brief 相机基础类型与像素编码定义。
 * @details
 * 提供图像像素编码（@ref CameraBase::Encoding）与相机标定/内参（@ref
 * CameraBase::CameraInfo）。
 * - 像素编码与 ROS `sensor_msgs/Image.encoding` 的命名/含义保持一致或可直观映射。
 * - 所有矩阵按 **行优先（Row-major）** 存储。
 * - `step` 为每行字节数（stride, bytes per row），**不是每像素字节数**。
 */

#include "libxr.hpp"

// STL
#include <array>
#include <cstdint>
#include <initializer_list>

/**
 * @class CameraBase
 * @brief 相机通用类型容器（仅定义公共类型/结构，不包含具体接口）。
 */
class CameraBase
{
 public:
  /**
   * @enum Encoding
   * @brief 图像像素编码类型。
   */
  enum Encoding : uint8_t
  {
    INVALID = 0,   ///< 无效/未设置的编码占位。
    RGB8,          ///< 8 位/通道，RGB 通道顺序，24bpp。
    BGR8,          ///< 8 位/通道，BGR 通道顺序，24bpp。
    RGBA8,         ///< 8 位/通道，RGBA（含 alpha），32bpp。
    BGRA8,         ///< 8 位/通道，BGRA（含 alpha），32bpp。
    RGB16,         ///< 16 位/通道，RGB，48bpp。
    BGR16,         ///< 16 位/通道，BGR，48bpp。
    RGBA16,        ///< 16 位/通道，RGBA，64bpp。
    BGRA16,        ///< 16 位/通道，BGRA，64bpp。
    MONO8,         ///< 8 位单通道灰度（Y）。
    MONO16,        ///< 16 位单通道灰度（Y）。
    BAYER_RGGB8,   ///< 8 位 Bayer RAW，排列 RGGB。
    BAYER_GRBG8,   ///< 8 位 Bayer RAW，排列 GRBG。
    BAYER_GBRG8,   ///< 8 位 Bayer RAW，排列 GBRG。
    BAYER_BGGR8,   ///< 8 位 Bayer RAW，排列 BGGR。
    BAYER_RGGB16,  ///< 16 位 Bayer RAW，排列 RGGB。
    BAYER_GRBG16,  ///< 16 位 Bayer RAW，排列 GRBG。
    BAYER_GBRG16,  ///< 16 位 Bayer RAW，排列 GBRG。
    BAYER_BGGR16,  ///< 16 位 Bayer RAW，排列 BGGR。
    YUV422         ///< 打包 YUV 4:2:2（常见布局
                   ///< YUYV/UYVY）。如需精确到子格式，请在协议层另行注明。
  };

  /**
   * @enum DistortionModel
   * @brief 畸变模型枚举。
   */
  enum class DistortionModel : uint8_t
  {
    NONE = 0,             ///< 无畸变；所有系数视为 0。
    PLUMB_BOB,            ///< Brown–Conrady（radtan）；k1,k2,p1,p2,k3。
    RATIONAL_POLYNOMIAL,  ///< OpenCV 扩展 8/12/14 参：+k4..k6,+s1..s4,+tauX,tauY。
    EQUIDISTANT,          ///< 等距（KB4 fisheye）：k1..k4。
    FOV,                  ///< Devernay–Basu FOV：常用单参 w。
    OMNI,                 ///< 统一全向（Mei/Scaramuzza）：含 xi。
    EXTENDED_UNIFIED,     ///< 扩展统一（EUCM）：alpha,beta。
    DOUBLE_SPHERE,        ///< 双球：xi,alpha。
    THIN_PRISM,           ///< 薄棱镜项 s1..s4；常与 radtan 组合。
    UNKNOWN               ///< 未知/自定义。
  };

  /**
   * @union DistortionCoeffs
   * @brief 畸变系数共用体：共享同一段 `double[14]` 存储。
   */
  union DistortionCoeffs
  {
    std::array<double, 14> raw;  ///< 原始系数存储（索引 0..13）。

    struct
    {
      double k1, k2, p1, p2, k3;
    } plumb_bob;  ///< 5
    struct
    {
      double k1, k2, k3, k4;
    } equidistant;  ///< 4
    struct
    {
      double w;
    } fov;  ///< 1
    struct
    {
      double xi, k1, k2, p1, p2, k3;
    } omni;  ///< 6
    struct
    {
      double alpha, beta;
    } extended_unified;  ///< 2
    struct
    {
      double xi, alpha;
    } double_sphere;  ///< 2
    struct
    {
      double s1, s2, s3, s4;
    } thin_prism;  ///< 4
    struct
    {  // 14（OpenCV 完整有理多项式）
      double k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, tauX, tauY;
    } rational14;  ///< 14

    DistortionCoeffs() : raw{} {}  ///< 零初始化并激活 raw 成员。
  };
  static_assert(sizeof(DistortionCoeffs) == sizeof(std::array<double, 14>),
                "overlay size mismatch");
  static_assert(alignof(DistortionCoeffs) == alignof(std::array<double, 14>),
                "overlay align mismatch");

  /**
   * @struct CameraInfo
   * @brief 图像尺寸、时间戳、编码与标定信息。
   *
   * 字段顺序与常见相机模型表达一致：
   * 1) 内参矩阵 K；2) 畸变模型与参数；3) 矫正旋转 R；4) 投影矩阵 P。
   */
  struct CameraInfo
  {
    // 基本图像信息
    uint32_t width{};   ///< 图像宽度（像素）。
    uint32_t height{};  ///< 图像高度（像素）。
    uint32_t step{};    ///< 每行字节数（bytes per row / stride）。
    LibXR::MicrosecondTimestamp
        timestamp;        ///< 采集时间戳（微秒）。具体时钟源见 `LibXR` 定义。
    Encoding encoding{};  ///< 像素编码类型（见 @ref Encoding ），如 RGB8/BGR8/MONO8 等。

    // 1) 内参矩阵 K = [ fx  0 cx ; 0 fy cy ; 0 0 1 ]（3×3，行优先）
    std::array<double, 9> camera_matrix;

    // 2) 畸变模型与参数（依据所选模型使用前 N 项，其余置 0）
    DistortionModel distortion_model{};  ///< 见 @ref DistortionModel 。
    DistortionCoeffs
        distortion_coefficients;  ///< 最多 14 项；常见 plumb_bob: k1,k2,p1,p2,k3。

    // 3) 矫正后的旋转矩阵 R（3×3，行优先）
    std::array<double, 9> rectification_matrix;

    // 4) 投影矩阵 P（3×4，行优先）
    // P = [ fx'  0   cx'  Tx ;
    //        0  fy' cy'  Ty ;
    //        0   0   1    0 ]
    std::array<double, 12> projection_matrix;

    // 默认构造：安全零/单位初始化
    CameraInfo()
        : timestamp{},
          camera_matrix{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
          distortion_coefficients{},
          rectification_matrix{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
          projection_matrix{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}
    {
    }

    static inline std::vector<double> ToPnPDistCoeffs(
        CameraBase::DistortionModel model, const CameraBase::DistortionCoeffs& D)
    {
      std::vector<double> dc;
      switch (model)
      {
        case CameraBase::DistortionModel::NONE:
          // 无畸变：传空向量
          break;

        case CameraBase::DistortionModel::PLUMB_BOB:
          // Brown–Conrady：k1,k2,p1,p2,k3
          dc = {D.plumb_bob.k1, D.plumb_bob.k2, D.plumb_bob.p1, D.plumb_bob.p2,
                D.plumb_bob.k3};
          break;

        case CameraBase::DistortionModel::RATIONAL_POLYNOMIAL:
          // TODO: 若 PnPSolver 支持 rational，多数实现接受前 8 项：
          //       k1,k2,p1,p2,k3,k4,k5,k6；若后端支持 14 项，再扩展。
          dc = {D.rational14.k1, D.rational14.k2, D.rational14.p1, D.rational14.p2,
                D.rational14.k3, D.rational14.k4, D.rational14.k5, D.rational14.k6};
          XR_LOG_WARN(
              "PnPSolver: using 8-term rational coefficients; extend to 14 if backend "
              "supports.");
          break;

        case CameraBase::DistortionModel::EQUIDISTANT:
        case CameraBase::DistortionModel::FOV:
        case CameraBase::DistortionModel::OMNI:
        case CameraBase::DistortionModel::EXTENDED_UNIFIED:
        case CameraBase::DistortionModel::DOUBLE_SPHERE:
        case CameraBase::DistortionModel::THIN_PRISM:
        case CameraBase::DistortionModel::UNKNOWN:
        default:
          XR_LOG_WARN(
              "PnPSolver: distortion model not natively supported (%d). "
              "TODO: undistort to pinhole first, then call PnP with NONE.",
              int(model));
          break;
      }
      return dc;
    }
  };
};
