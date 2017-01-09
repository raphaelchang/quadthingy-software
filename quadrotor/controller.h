#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "bno055.h"
#include "motor_driver.h"
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
    void SetOrientation(vector3 setpoint);
    void SetThrottle(double throttle);
private:
    void velocityLoopUpdate(vector3 setpoint);
    double m_throttle;
    MotorDriver *md1;
    MotorDriver *md2;
    MotorDriver *md3;
    MotorDriver *md4;
};

#endif /* CONTROLLER_H_ */
