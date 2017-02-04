#include "controller.h"
#include <Eigen/Core>
#include "hw_conf.h"
#include "comm_usb.h"
#include "utils.h"
#include <math.h>
#include "chprintf.h"

#define CLAMP_ZERO(num) num > 0 ? num : 0

using namespace Eigen;

Controller::Controller() :
    m_vel_p_gains(0.05, 0.05, 0.0001),
    m_vel_i_gains(0, 0, 0),
    m_vel_d_gains(0, 0, 0),
    m_vel_ff_gains(0.05, 0.05, 0.01),
    m_pos_p_gains(0.1, 0.1, 0.01),
    m_pos_i_gains(0, 0, 0),
    m_pos_d_gains(0, 0, 0),
    m_orientation(0, 0, 0),
    m_rotation(0, 0, 0)
{
    m_throttle = 0;
    m_enabled = false;
    m_imu = new IMU();
    m_md1 = new MotorDriver(&MOTOR_DRIVER_1_PWM_DEV, MOTOR_DRIVER_1_FWD_CH, MOTOR_DRIVER_1_REV_CH);
    m_md2 = new MotorDriver(&MOTOR_DRIVER_2_PWM_DEV, MOTOR_DRIVER_2_FWD_CH, MOTOR_DRIVER_2_REV_CH);
    m_md3 = new MotorDriver(&MOTOR_DRIVER_3_PWM_DEV, MOTOR_DRIVER_3_FWD_CH, MOTOR_DRIVER_3_REV_CH);
    m_md4 = new MotorDriver(MOTOR_DRIVER_4_PWM_DEV, MOTOR_DRIVER_4_FWD_CH, MOTOR_DRIVER_4_REV_CH);
    //m_orientation(0) = Utils::atan2(-m_imu->GetVector(VECTOR_ACCELEROMETER)(1), m_imu->GetVector(VECTOR_ACCELEROMETER)(2)) * 180 / M_PI;
    //m_orientation(1) = Utils::atan2(m_imu->GetVector(VECTOR_ACCELEROMETER)(0), m_imu->GetVector(VECTOR_ACCELEROMETER)(2)) * 180 / M_PI;
}

Controller::~Controller()
{
}

void Controller::Update()
{
    //double alpha = 0.05;
    //double dt = 0.001;
    //m_orientation(0) = (1 - alpha) * (m_orientation(0) + m_imu->GetVector(VECTOR_GYROSCOPE)(0) * 180 / M_PI * dt) + alpha * Utils::atan2(-m_imu->GetVector(VECTOR_ACCELEROMETER)(1), m_imu->GetVector(VECTOR_ACCELEROMETER)(2)) * 180 / M_PI;
    //m_orientation(1) = (1 - alpha) * (m_orientation(1) + m_imu->GetVector(VECTOR_GYROSCOPE)(1) * 180 / M_PI * dt) + alpha * Utils::atan2(m_imu->GetVector(VECTOR_ACCELEROMETER)(0), m_imu->GetVector(VECTOR_ACCELEROMETER)(2)) * 180 / M_PI;
    //m_rotation = m_imu->GetVector(VECTOR_GYROSCOPE);
    m_imu->Update();
    m_orientation = m_imu->GetOrientation();
    m_rotation = m_imu->GetRotationRate();
    if (fabs(m_orientation(0)) > 50 || fabs(m_orientation(1)) > 50)
    {
        Disable();
    }
    Vector3f setpoint(0, 0, 0);
    double errX = setpoint(0) - m_orientation(0); // pitch error
    double errY = setpoint(1) - m_orientation(1); // roll error
    double errZ = setpoint(2) - m_orientation(2); // yaw error
    setpoint(0) = errX * m_pos_p_gains(0);
    setpoint(1) = errY * m_pos_p_gains(1);
    setpoint(2) = 0.0;//errZ * pos_p_gains(2);
    velocityLoopUpdate(setpoint);
}

Vector3f Controller::GetOrientation()
{
    return m_orientation;
}

Vector3f Controller::GetRotationRate()
{
    return m_rotation;
}

void Controller::SetThrottle(double throttle)
{
    m_throttle = throttle;
}

void Controller::Enable()
{
    m_enabled = true;
}

void Controller::Disable()
{
    m_enabled = false;
    m_throttle = 0;
}

void Controller::velocityLoopUpdate(Vector3f setpoint)
{
    double errX = setpoint(0) - m_rotation(0); // pitch error
    double errY = setpoint(1) - m_rotation(1); // roll error
    double errZ = setpoint(2) - m_rotation(2); // yaw error
    double pitchOut = errX * m_vel_p_gains(0) + setpoint(0) * m_vel_ff_gains(0);
    double rollOut = errY * m_vel_p_gains(1) + setpoint(1) * m_vel_ff_gains(1);
    double yawOut = errZ * m_vel_p_gains(2) + setpoint(2) * m_vel_ff_gains(2);
    if (m_enabled)
    {
        //m_md1->Set(0.5);
        //m_md2->Set(-0.5);
        //m_md3->Set(0.5);
        //m_md4->Set(-0.5);
        m_md1->Set(m_throttle - pitchOut + rollOut + yawOut);
        m_md2->Set(-(m_throttle - pitchOut - rollOut - yawOut));
        m_md3->Set(m_throttle + pitchOut - rollOut + yawOut);
        m_md4->Set(-(m_throttle + pitchOut + rollOut - yawOut));
    }
    else
    {
        m_md1->Set(0.0);
        m_md2->Set(0.0);
        m_md3->Set(0.0);
        m_md4->Set(0.0);
    }
}
