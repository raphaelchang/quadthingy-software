#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "bno055.h"
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
private:
    void velocityLoopUpdate(Eigen::Vector3d setpoint);
    double m_throttle;
    BNO055 *m_imu;
    MotorDriver *m_md1;
    MotorDriver *m_md2;
    MotorDriver *m_md3;
    MotorDriver *m_md4;
    Eigen::Vector3d m_orientation;
    Eigen::Vector3d m_rotation;
    Eigen::Vector3d m_vel_p_gains;
    Eigen::Vector3d m_vel_i_gains;
    Eigen::Vector3d m_vel_d_gains;
    Eigen::Vector3d m_vel_ff_gains;
    Eigen::Vector3d m_pos_p_gains;
    Eigen::Vector3d m_pos_i_gains;
    Eigen::Vector3d m_pos_d_gains;
};

#endif /* CONTROLLER_H_ */
