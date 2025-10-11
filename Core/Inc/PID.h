#ifndef PID_H
#define PID_H

typedef struct {
  double kp;
  double ki;
  double kd;
  double prev_error;
  double integral;
} PIDController;

/**
 * @brief Initializes the PID controller parameters.
 * @param pid Pointer to the PIDController structure.
 * @param kp proportional constant
 * @param ki integral constant (pd loop should probably be enough for us so just leave as 0)
 * @param kd derivative constant.
 */
void PID_init(PIDController* pid, double kp, double ki, double kd);

/**
 * @brief Computes the PID output value.
 * @param pid Pointer to the PIDController structure.
 * @param setpoint The desired target value.
 * @param measured The current measured value.
 * @return The calculated control output.
 */
double PID_step(PIDController* pid, double setpoint, double measured);

/**
 * @brief idk how to actually read the servo position so I just made a function for it
 * TODO: Actually replace with sensor reading code
 */
double get_servo_position(void);

/**
 * @brief I also dont know how to actually move the servo
 * TODO: Actually replace with working code
 */
void set_servo_power(double step_power);

#endif // PID_H
