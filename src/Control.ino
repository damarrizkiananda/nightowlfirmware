/*
 *  Wheel Configuration
 *           
 *     o2         o1
 * 
 * 
 * 
 *           o3
 * 
 * 
 *  Axis
 *            x+   
 *             I 
 *             I 
 *     y+ _ _ _I
 * 
 * 
 */

void regression()
{
  motorPwm1 = wheelVelocity1_Target/0.4617 + 2.2108/0.4617;
  motorPwm2 = wheelVelocity2_Target/0.4275 + 1.544/0.4275;
  motorPwm3 = wheelVelocity3_Target/0.4541 + 0.7405/0.4541;
}

/* Distance between the center of the robot to the wheels in cm */
#define L 30

/* Important Angle in Degree */
#define delta 30

/* This function converts target vx, vy, & omega to wheel linear velocity (global variables) 
 * and then converts wheel velocity to pwm
 */ 
void inverseKinematics(int vx, int vy, int omega)
{
  wheelVelocity1_Target =  cos(delta*TO_RAD)*vx + sin(delta*TO_RAD)*vy + L*omega*TO_RAD;
  wheelVelocity2_Target = -cos(delta*TO_RAD)*vx + sin(delta*TO_RAD)*vy + L*omega*TO_RAD;
  wheelVelocity3_Target = -vy + L*omega*TO_RAD;  

  regression();
}

void moveRobotGlobal(int vx_Global, int vy_Global, float omega)
{
  float omega_f = omega;
  omega_f = omega_f/100;
  omega_f = omega_f*TO_DEG;
  omega = omega_f;

  vx_Global = constrain(vx_Global, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  vy_Global = constrain(vy_Global, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  omega = constrain(omega, -MAX_ROBOT_OMEGA, MAX_ROBOT_OMEGA);

  noInterrupts();
  auto currentTheta = theta_Real;
  interrupts();

  double sin_theta = sin(currentTheta*TO_RAD);
  double cos_theta = cos(currentTheta*TO_RAD);
  
  double vx_Local = vx_Global*cos_theta + vy_Global*sin_theta;
  double vy_Local = -vx_Global*sin_theta + vy_Global*cos_theta;
  
  double totalSpeed = sqrt(vx_Local*vx_Local + vy_Local*vy_Local);
  if(totalSpeed>MAX_ROBOT_SPEED)
  {
    vx_Local = vx_Local/totalSpeed;
    vy_Local = vy_Local/totalSpeed;

    vx_Local = vx_Local*MAX_ROBOT_SPEED;
    vy_Local = vy_Local*MAX_ROBOT_SPEED;
  }

  inverseKinematics(vx_Local, vy_Local, omega);
  robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
}

void moveRobotLocal(int vx_Local, int vy_Local, float omega)
{
  float omega_f = omega;
  omega_f = omega_f/100;
  omega_f = omega_f*TO_DEG;
  omega = omega_f;

  vx_Local = constrain(vx_Local, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  vy_Local = constrain(vy_Local, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  omega = constrain(omega, -MAX_ROBOT_OMEGA, MAX_ROBOT_OMEGA);
  
  double totalSpeed = sqrt(vx_Local*vx_Local + vy_Local*vy_Local);
  if(totalSpeed>MAX_ROBOT_SPEED)
  {
    vx_Local = vx_Local/totalSpeed;
    vy_Local = vy_Local/totalSpeed;

    vx_Local = vx_Local*MAX_ROBOT_SPEED;
    vy_Local = vy_Local*MAX_ROBOT_SPEED;
  }

  inverseKinematics(vx_Local, vy_Local, omega);
  robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
}
