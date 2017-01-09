#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

void motor_driver_init(void);
void motor_driver_set(uint8_t id, double duty_cycle);
void motor_driver_set_all(double duty_cycle);

#endif /* ESC_H_ */
