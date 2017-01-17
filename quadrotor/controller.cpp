#include "controller.h"
#include <Eigen/Core>
#include "hw_conf.h"

using namespace Eigen;

Controller::Controller()
    : m_vel_p_gains(0.005, 0.005, -0.005),
    m_vel_i_gains(0, 0, 0),
    m_vel_d_gains(0, 0, 0),
    m_vel_ff_gains(0.05, 0.05, 0.01),
    m_pos_p_gains(-0.02, -0.02, 0.01),
    m_pos_i_gains(0, 0, 0),
    m_pos_d_gains(0, 0, 0),
    m_orientation(0, 0, 0),
    m_rotation(0, 0, 0)
{
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
    m_orientation = m_imu->GetVector(VECTOR_EULER);
    m_rotation = m_imu->GetVector(VECTOR_GYROSCOPE);
    Vector3d setpoint(0, 0, 0);
    double errX = setpoint(0) - m_orientation(0); // pitch error
    double errY = setpoint(1) - m_orientation(1); // roll error
    double errZ = setpoint(2) - m_orientation(2); // yaw error
    setpoint(0) = errX * m_pos_p_gains(0);
    setpoint(1) = errY * m_pos_p_gains(1);
    setpoint(2) = 0.0;//errZ * pos_p_gains(2);
    velocityLoopUpdate(setpoint);
}

void Controller::SetThrottle(double throttle)
{
    m_throttle = throttle;
}

void Controller::velocityLoopUpdate(Vector3d setpoint)
{
    double errX = setpoint(0) - m_rotation(0); // pitch error
    double errY = setpoint(1) - m_rotation(1); // roll error
    double errZ = setpoint(2) - m_rotation(2); // yaw error
    double pitchOut = errX * m_vel_p_gains(0) + setpoint(0) * m_vel_ff_gains(0);
    double rollOut = errY * m_vel_p_gains(1) + setpoint(1) * m_vel_ff_gains(1);
    double yawOut = errZ * m_vel_p_gains(2) + setpoint(2) * m_vel_ff_gains(2);
    m_md1->Set(m_throttle + pitchOut - yawOut);
    m_md2->Set(m_throttle - rollOut + yawOut);
    m_md3->Set(m_throttle - pitchOut - yawOut);
    m_md4->Set(m_throttle + rollOut + yawOut);
}
