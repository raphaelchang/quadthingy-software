#include "controller.h"
#include <Eigen/Core>
#include "hw_conf.h"

vector3 orientation;
vector3 rotation;
vector3 vel_p_gains;
vector3 vel_i_gains;
vector3 vel_d_gains;
vector3 vel_ff_gains;
vector3 pos_p_gains;
vector3 pos_i_gains;
vector3 pos_d_gains;

Controller::Controller()
{
    vel_p_gains.x = 0.005;
    vel_p_gains.y = 0.005;
    vel_p_gains.z = -0.005;
    vel_i_gains.x = 0;
    vel_i_gains.y = 0;
    vel_i_gains.z = 0;
    vel_d_gains.x = 0;
    vel_d_gains.y = 0;
    vel_d_gains.z = 0;
    vel_ff_gains.x = 0.05;
    vel_ff_gains.y = 0.05;
    vel_ff_gains.z = 0.01;
    pos_p_gains.x = -0.02;
    pos_p_gains.y = -0.02;
    pos_p_gains.z = 0.01;
    pos_i_gains.x = 0;
    pos_i_gains.y = 0;
    pos_i_gains.z = 0;
    pos_d_gains.x = 0;
    pos_d_gains.y = 0;
    pos_d_gains.z = 0;
    m_throttle = 0;
    md1 = new MotorDriver(&MOTOR_DRIVER_1_PWM_DEV, MOTOR_DRIVER_1_FWD_CH, MOTOR_DRIVER_1_REV_CH);
    md2 = new MotorDriver(&MOTOR_DRIVER_2_PWM_DEV, MOTOR_DRIVER_2_FWD_CH, MOTOR_DRIVER_2_REV_CH);
    md3 = new MotorDriver(&MOTOR_DRIVER_3_PWM_DEV, MOTOR_DRIVER_3_FWD_CH, MOTOR_DRIVER_3_REV_CH);
    md4 = new MotorDriver(&MOTOR_DRIVER_4_PWM_DEV, MOTOR_DRIVER_4_FWD_CH, MOTOR_DRIVER_4_REV_CH);
}

void Controller::Update()
{
    orientation = bno055_get_vector(VECTOR_EULER);
    rotation = bno055_get_vector(VECTOR_GYROSCOPE);
    vector3 setpoint;
    setpoint.x = 0.0;
    setpoint.y = 0.0;
    setpoint.z = 0.0;
    double errX = setpoint.x - orientation.x; // pitch error
    double errY = setpoint.y - orientation.y; // roll error
    double errZ = setpoint.z - orientation.z; // yaw error
    setpoint.x = errX * pos_p_gains.x;
    setpoint.y = errY * pos_p_gains.y;
    setpoint.z = 0.0;//errZ * pos_p_gains.z;
    velocityLoopUpdate(setpoint);
}

void Controller::SetThrottle(double throttle)
{
    m_throttle = throttle;
}

void Controller::velocityLoopUpdate(vector3 setpoint)
{
    double errX = setpoint.x - rotation.x; // pitch error
    double errY = setpoint.y - rotation.y; // roll error
    double errZ = setpoint.z - rotation.z; // yaw error
    double pitchOut = errX * vel_p_gains.x + setpoint.x * vel_ff_gains.x;
    double rollOut = errY * vel_p_gains.y + setpoint.y * vel_ff_gains.y;
    double yawOut = errZ * vel_p_gains.z + setpoint.z * vel_ff_gains.z;
    md1->Set( m_throttle + pitchOut - yawOut);
    md2->Set(m_throttle - rollOut + yawOut);
    md3->Set(m_throttle - pitchOut - yawOut);
    md4->Set(m_throttle + rollOut + yawOut);
}
