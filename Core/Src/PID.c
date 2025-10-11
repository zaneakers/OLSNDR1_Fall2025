#include "PID.h"

double get_servo_position(void) {
  // TODO: Actually get serve position
  return 0.0;
}

void set_servo_power(double step_power) {
  // TODO: Actually set servo power
}

void PID_init(PIDController* pid, double kp, double ki, double kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->integral = 0.0;
  pid->previous_error = 0.0;
}

double PID_step(PIDController *pid, double setpoint, double measured) {
  double error = setpoint - measured;

  pid->integral += error;

  double derivative = error - pid->previous_error;

  pid->previous_error = error;

  return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}
