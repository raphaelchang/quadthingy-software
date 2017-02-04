#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "imu.h"
#include "motor_driver.h"
#include <Eigen/Dense>
extern "C"
{
#include "ch.h"
}

class Controller
{
public:
    Controller();
    ~Controller();
    void Update();
    void SetOrientation(Eigen::Vector3d setpoint);
    void SetThrottle(double throttle);
    void Enable();
    void Disable();
    Eigen::Vector3f GetOrientation();
    Eigen::Vector3f GetRotationRate();
    IMU *m_imu;

private:
    void velocityLoopUpdate(Eigen::Vector3f setpoint);
    double m_throttle;
    bool m_enabled;
    MotorDriver *m_md1;
    MotorDriver *m_md2;
    MotorDriver *m_md3;
    MotorDriver *m_md4;
    Eigen::Vector3f m_orientation;
    Eigen::Vector3f m_rotation;
    Eigen::Vector3f m_vel_p_gains;
    Eigen::Vector3f m_vel_i_gains;
    Eigen::Vector3f m_vel_d_gains;
    Eigen::Vector3f m_vel_ff_gains;
    Eigen::Vector3f m_pos_p_gains;
    Eigen::Vector3f m_pos_i_gains;
    Eigen::Vector3f m_pos_d_gains;
};

#endif /* CONTROLLER_H_ */
