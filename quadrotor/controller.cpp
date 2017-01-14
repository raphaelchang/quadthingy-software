#include "controller.h"
#include <Eigen/Core>
#include "hw_conf.h"

using namespace Eigen;

Vector3d orientation;
Vector3d rotation;
Vector3d vel_p_gains;
Vector3d vel_i_gains;
Vector3d vel_d_gains;
Vector3d vel_ff_gains;
Vector3d pos_p_gains;
Vector3d pos_i_gains;
Vector3d pos_d_gains;

Controller::Controller()
{
    vel_p_gains(0) = 0.005;
    vel_p_gains(1) = 0.005;
    vel_p_gains(2) = -0.005;
    vel_i_gains(0) = 0;
    vel_i_gains(1) = 0;
    vel_i_gains(2) = 0;
    vel_d_gains(0) = 0;
    vel_d_gains(1) = 0;
    vel_d_gains(2) = 0;
    vel_ff_gains(0) = 0.05;
    vel_ff_gains(1) = 0.05;
    vel_ff_gains(2) = 0.01;
    pos_p_gains(0) = -0.02;
    pos_p_gains(1) = -0.02;
    pos_p_gains(2) = 0.01;
    pos_i_gains(0) = 0;
    pos_i_gains(1) = 0;
    pos_i_gains(2) = 0;
    pos_d_gains(0) = 0;
    pos_d_gains(1) = 0;
    pos_d_gains(2) = 0;
    m_throttle = 0;
    m_imu = new BNO055();
    m_md1 = new MotorDriver(&MOTOR_DRIVER_1_PWM_DEV, MOTOR_DRIVER_1_FWD_CH, MOTOR_DRIVER_1_REV_CH);
    m_md2 = new MotorDriver(&MOTOR_DRIVER_2_PWM_DEV, MOTOR_DRIVER_2_FWD_CH, MOTOR_DRIVER_2_REV_CH);
    m_md3 = new MotorDriver(&MOTOR_DRIVER_3_PWM_DEV, MOTOR_DRIVER_3_FWD_CH, MOTOR_DRIVER_3_REV_CH);
    m_md4 = new MotorDriver(&MOTOR_DRIVER_4_PWM_DEV, MOTOR_DRIVER_4_FWD_CH, MOTOR_DRIVER_4_REV_CH);
}

Controller::~Controller()
{
}

void Controller::Update()
{
    orientation = m_imu->GetVector(VECTOR_EULER);
    rotation = m_imu->GetVector(VECTOR_GYROSCOPE);
    Vector3d setpoint;
    setpoint(0) = 0.0;
    setpoint(1) = 0.0;
    setpoint(2) = 0.0;
    double errX = setpoint(0) - orientation(0); // pitch error
    double errY = setpoint(1) - orientation(1); // roll error
    double errZ = setpoint(2) - orientation(2); // yaw error
    setpoint(0) = errX * pos_p_gains(0);
    setpoint(1) = errY * pos_p_gains(1);
    setpoint(2) = 0.0;//errZ * pos_p_gains(2);
    velocityLoopUpdate(setpoint);
}

void Controller::SetThrottle(double throttle)
{
    m_throttle = throttle;
}

void Controller::velocityLoopUpdate(Vector3d setpoint)
{
    double errX = setpoint(0) - rotation(0); // pitch error
    double errY = setpoint(1) - rotation(1); // roll error
    double errZ = setpoint(2) - rotation(2); // yaw error
    double pitchOut = errX * vel_p_gains(0) + setpoint(0) * vel_ff_gains(0);
    double rollOut = errY * vel_p_gains(1) + setpoint(1) * vel_ff_gains(1);
    double yawOut = errZ * vel_p_gains(2) + setpoint(2) * vel_ff_gains(2);
    m_md1->Set( m_throttle + pitchOut - yawOut);
    m_md2->Set(m_throttle - rollOut + ywOut);
    m_md3->Set(m_throttle - pitchOut - yawOut);
    m_md4->Set(m_throttle + rollOut + yawOut);
}
