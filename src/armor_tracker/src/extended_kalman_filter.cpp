#include "extended_kalman_filter.hpp"

/*
f:过程函数
h:观测函数
j_f:过程函数的雅可比矩阵
j_h:测量函数的雅可比矩阵
u_q:过程噪声协方差矩阵
u_r:测量噪声协方差矩阵
P0:初始状态协方差矩阵
*/
ExtendedKalmanFilter::ExtendedKalmanFilter(const VecVecFunc& f, const VecVecFunc& h,
                                           const VecMatFunc& j_f, const VecMatFunc& j_h,
                                           const VoidMatFunc& u_q, const VecMatFunc& u_r,
                                           const Eigen::MatrixXd& P0)
    : f_(f),
      h_(h),
      jacobian_f_(j_f),
      jacobian_h_(j_h),
      update_q_(u_q),
      update_r_(u_r),
      p_post_(P0),
      n_(P0.rows()),
      i_(Eigen::MatrixXd::Identity(n_, n_)),
      x_pri_(n_),
      x_post_(n_)
{
}

void ExtendedKalmanFilter::SetState(const Eigen::VectorXd& x0) { x_post_ = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::Predict()
{
  m_f_ = jacobian_f_(x_post_), m_q_ = update_q_();

  x_pri_ = f_(x_post_);
  p_pri_ = m_f_ * p_post_ * m_f_.transpose() + m_q_;

  // handle the case when there will be no measurement before the next predict
  x_post_ = x_pri_;
  p_post_ = p_pri_;

  return x_pri_;
}

Eigen::MatrixXd ExtendedKalmanFilter::Update(const Eigen::VectorXd& z)
{
  m_h_ = jacobian_h_(x_pri_), m_r_ = update_r_(z);

  m_k_ = p_pri_ * m_h_.transpose() *
         (m_h_ * p_pri_ * m_h_.transpose() + m_r_).inverse();  // inverse计算逆矩阵
  x_post_ = x_pri_ + m_k_ * (z - h_(x_pri_));
  p_post_ = (i_ - m_k_ * m_h_) * p_pri_;

  return x_post_;
}
