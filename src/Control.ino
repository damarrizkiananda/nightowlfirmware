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

void moveRobotGlobal(int vX, int vY, float omega)
{
  float omega_f = omega;
  omega_f = omega_f/100;
  omega_f = omega_f*TO_DEG;
  omega = omega_f;

  vX = constrain(vX, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  vY = constrain(vY, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  omega = constrain(omega, -MAX_ROBOT_OMEGA, MAX_ROBOT_OMEGA);

  noInterrupts();
  auto currentTheta = theta_Real;
  interrupts();

  double sin_theta = sin(-currentTheta*TO_RAD);
  double cos_theta = cos(-currentTheta*TO_RAD);
  
  double globalOutputX = vX*cos_theta - vY*sin_theta;
  double globalOutputY = vX*sin_theta + vY*cos_theta;
  
  double totalSpeed = sqrt(globalOutputX*globalOutputX + globalOutputY*globalOutputY);
  if(totalSpeed>MAX_ROBOT_SPEED)
  {
    globalOutputX = globalOutputX/totalSpeed;
    globalOutputY = globalOutputY/totalSpeed;

    globalOutputX = globalOutputX*MAX_ROBOT_SPEED;
    globalOutputY = globalOutputY*MAX_ROBOT_SPEED;
  }

  inverseKinematics(globalOutputX, globalOutputY, omega);
  robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
}

void moveRobotLocal(int vX, int vY, float omega)
{
  float omega_f = omega;
  omega_f = omega_f/100;
  omega_f = omega_f*TO_DEG;
  omega = omega_f;

  vX = constrain(vX, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  vY = constrain(vY, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
  omega = constrain(omega, -MAX_ROBOT_OMEGA, MAX_ROBOT_OMEGA);
  
  double totalSpeed = sqrt(vX*vX + vY*vY);
  if(totalSpeed>MAX_ROBOT_SPEED)
  {
    vX = vX/totalSpeed;
    vY = vY/totalSpeed;

    vX = vX*MAX_ROBOT_SPEED;
    vY = vY*MAX_ROBOT_SPEED;
  }

  inverseKinematics(vX, vY, omega);
  robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
}
