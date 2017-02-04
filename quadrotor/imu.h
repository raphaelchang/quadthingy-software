#ifndef _IMU_H_
#define _IMU_H_

#include "bno055.h"
#include <Eigen/Dense>
extern "C"
{
#include "ch.h"
}

class IMU
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMU();
    ~IMU();
    void Update();
    Eigen::Vector3f GetOrientation();
    Eigen::Vector3f GetRotationRate();
    BNO055 *m_imu;

private:
    Eigen::Matrix<float, 6, 1> H(Eigen::Matrix<float, 9, 1> x);
    Eigen::Matrix<float, 9, 1> m_x; // State vector
    Eigen::Matrix<float, 6, 1> m_z; // Measurement vector
    Eigen::Matrix<float, 9, 9> m_P; // Covariance matrix
    Eigen::Matrix<float, 9, 9> m_A; // Predict matrix
    Eigen::Matrix<float, 9, 9> m_Q; // Predict covariance
    Eigen::Matrix<float, 6, 9> m_J; // Jacobian of measurement model (H)
    Eigen::Matrix<float, 6, 6> m_R; // Measurement covariance
};

#endif /* _IMU_H_ */
