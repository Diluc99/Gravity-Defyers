//PID VARIABLES
float positional = 0; 
float previous_error = 0;
float integral = 0; 
float derivative = 0;
float pidVal = 0; // final Pid value

//PID CONSTANTS(I'VE TO TEST AND SET, keeping 0 for now)
float Kp =0;
float Ki =0;
float Kd =0;

// current angle and target angle based on MPU
float target_angle;
// PID CALCULATION
 error = current_angle - target_angle;
  integral += positional * dt;
  derivative = (error - previous_error) / dt;
  previous_error = error;
  pidVal = (Kp * error) + (Ki * integral) + (Kd * derivative);
