#include "imu.h"
#include "utils.h"

#define DT 0.005

using namespace Eigen;

IMU::IMU()
{
    m_imu = new BNO055();
    Vector3f acc(0, 0, 0);
    while (acc(0) == 0 && acc(1) == 0 && acc(2) == 0)
        acc = m_imu->GetVector(VECTOR_ACCELEROMETER);
    m_x << Utils::atan2(-acc(1), acc(2)),
        Utils::atan2(acc(0), acc(2)),
        0, 0, 0, 0, 0, 0, 0;
    m_P = Matrix<float, 9, 9>::Zero();
    m_A << 1, 0, 0, DT, 0, 0, -DT, 0, 0,
        0, 1, 0, 0, DT, 0, 0, -DT, 0,
        0, 0, 1, 0, 0, DT, 0, 0, -DT,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
    m_Q << DT * DT, 0, 0, 0, 0, 0, 0, 0, 0,
        0, DT * DT, 0, 0, 0, 0, 0, 0, 0,
        0, 0, DT * DT, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0.01 * DT, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0.01 * DT, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.01 * DT, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0.03 * DT, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0.03 * DT, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0.03 * DT;
    m_J << 0, 0, 0, 1, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0;
    m_R << 0.001, 0, 0, 0, 0, 0,
        0, 0.001, 0, 0, 0, 0,
        0, 0, 0.001, 0, 0, 0,
        0, 0, 0, 0.001, 0, 0,
        0, 0, 0, 0, 0.001, 0,
        0, 0, 0, 0, 0, 0.001;
}

IMU::~IMU()
{
}

void IMU::Update()
{
    // Predict
    m_x = m_A * m_x;
    Matrix<float, 9, 9> P_old = m_P;
    m_P = m_A * P_old *  m_A.transpose() + m_Q;
    // Update
    float sin_x, sin_y, cos_x, cos_y;
    Utils::sincos((float)m_x(0), &sin_x, &cos_x);
    Utils::sincos((float)m_x(1), &sin_y, &cos_y);
    m_J(3, 1) = cos_y;
    m_J(4, 0) = cos_x * cos_y;
    m_J(4, 1) = -sin_x * sin_y;
    m_J(5, 0) = -sin_x * cos_y;
    m_J(5, 1) = -cos_x * sin_y;
    m_z.head<3>() = m_imu->GetVector(VECTOR_GYROSCOPE);
    m_z.tail<3>() = m_imu->GetVector(VECTOR_ACCELEROMETER) / 9.8;
    m_z(4) *= -1;
    Matrix<float, 9, 6> J_T = m_J.transpose();
    Matrix<float, 6, 1> y = m_z - H(m_x);
    Matrix<float, 6, 6> S = m_J * m_P * J_T + m_R;
    if (!S.fullPivLu().isInvertible())
        return;
    Matrix<float, 6, 6> S_inv =  S.fullPivLu().solve(Matrix<float, 6, 6>::Identity());
    Matrix<float, 9, 6> K = m_P * J_T * S_inv;
    Matrix<float, 9, 1> x_old = m_x;
    m_x = x_old + K * y;
    P_old = m_P;
    m_P = (Matrix<float, 9, 9>::Identity() - K * m_J) * P_old;
}

Vector3f IMU::GetOrientation()
{
    return m_x.head<3>() * 180 / M_PI;
}

Vector3f IMU::GetRotationRate()
{
    return m_x.segment<3>(3);
}

Matrix<float, 6, 1> IMU::H(Matrix<float, 9, 1> x)
{
    float sin_x, sin_y, cos_x, cos_y;
    Utils::sincos((float)x(0), &sin_x, &cos_x);
    Utils::sincos((float)x(1), &sin_y, &cos_y);
    Matrix<float, 6, 1> res;
    res << x(3) + x(6), x(4) + x(7), x(5) + x(8),
    sin_y, sin_x * cos_y, cos_x * cos_y;
    return res;
}
