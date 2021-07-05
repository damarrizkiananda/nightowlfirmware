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

// double kp[3] = {1.2, 1.2, 1.2}, ki[3] = {0.000031, 0.000031, 0.000031}, kd[3] = {1.5, 1.5, 1.5}; // Continuous PID constants
double kp[3] = {0.168, 0.182, 0.816}, ki[3] = {6.72, 7.27, 6.68}; // Discrete PI constants
double prevPWM[3] = {0.0, 0.0, 0.0};
int current_time = 0, prev_time=0;

void regression()
{
  motorPwm[0] = wheelVelocity_Target[0]/0.4617 + 2.2108/0.4617;
  motorPwm[1] = wheelVelocity_Target[1]/0.4275 + 1.544/0.4275;
  motorPwm[2] = wheelVelocity_Target[2]/0.4541 + 0.7405/0.4541;
}

void velocityToPwm(){
  for(int i = 0; i < 3; i++){
    wheelVelocity_Error[i] = wheelVelocity_Target[i] - wheelVelocity_Real_50[i];
    wheelVelocity_Error_Sum[i] += wheelVelocity_Error[i];
    // motorPwm[i] = wheelVelocity_Error[i] * kp[i] + wheelVelocity_Error_Sum[i] * ki[i] + (wheelVelocity_Error[i] - wheelVelocity_Prev_Error[i]) * kd[i];
  }
  current_time = millis();
  if(current_time - prev_time >= 50){
    for(int i = 0; i < 3; i++){
      motorPwm[i] = prevPWM[i] + wheelVelocity_Error[i] * kp[i] + wheelVelocity_Prev_Error[i] * (ki[i] * 0.05 - kp[i]);
      motorPwm[i] = constrain(motorPwm[i], -255, 255);
      prevPWM[i] = motorPwm[i];
      wheelVelocity_Prev_Error[i] = wheelVelocity_Error[i];
    }
    prev_time = current_time;
  }
    

  for(int i = 0; i < 3; i++){
    if(abs(wheelVelocity_Real[i]) < 3 && wheelVelocity_Target[i] == 0){
      motorPwm[i] = 0;
      // wheelVelocity1_Error_Sum = 0;
      wheelVelocity_Prev_Error[i] = 0.0;
      wheelVelocity_Error[i] = 0.0;
      prevPWM[i] = 0.0;
    }
  }
}

/* Distance between the center of the robot to the wheels in cm */
#define L 20

/* Important Angle in Degree */
#define delta 30

/* This function converts target vx, vy, & omega to wheel linear velocity (global variables) 
 * and then converts wheel velocity to pwm
 */ 
void inverseKinematics(int vx, int vy, int omega)
{
  wheelVelocity_Target[0] =  cos(delta*TO_RAD)*vx + sin(delta*TO_RAD)*vy + L*omega*TO_RAD;
  wheelVelocity_Target[1] = (-cos(delta*TO_RAD)*vx + sin(delta*TO_RAD)*vy + L*omega*TO_RAD)*0.96;
  wheelVelocity_Target[2] = (-vy + L*omega*TO_RAD)*1.03;  

  // regression();
  // velocityToPwm();
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
    vx_Local = (vx_Local/totalSpeed)*MAX_ROBOT_SPEED;
    vy_Local = (vy_Local/totalSpeed)*MAX_ROBOT_SPEED;
  }

  inverseKinematics(vx_Local, vy_Local, omega);
  velocityToPwm();
  robotMotorWrite(motorPwm[0], motorPwm[1], motorPwm[2]);
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
    vx_Local = (vx_Local/totalSpeed)*MAX_ROBOT_SPEED;
    vy_Local = (vy_Local/totalSpeed)*MAX_ROBOT_SPEED;
  }

  inverseKinematics(vx_Local, vy_Local, omega);

  double acc_thresh = 5.0;
  const double dcc_thresh = acc_thresh*1.49;  //m/s^2
  float acc_thresh_ratio = 1;
   
  // Acceleration limit
  for (int i = 0; i < 3; i++){
    wheelAcc[i] = wheelVelocity_Target[i] - wheelVelocity_Prev_Target[i];
    float acc_thresh_ratio_temp = 0;
    if( wheelAcc[i]*wheelVelocity_Prev_Target[i]>=0 ) //speed up
      acc_thresh_ratio_temp = fabs(wheelAcc[i])/acc_thresh;
    else                                 //speed down
      acc_thresh_ratio_temp = fabs(wheelAcc[i])/dcc_thresh;
    if( acc_thresh_ratio_temp>acc_thresh_ratio ){
      wheelAcc[i]/= acc_thresh_ratio_temp;
      wheelVelocity_Target[i] = wheelVelocity_Prev_Target[i] + wheelAcc[i];
    }
    wheelVelocity_Prev_Target[i] = wheelVelocity_Target[i];
  }

  velocityToPwm();
  robotMotorWrite(motorPwm[0], motorPwm[1], motorPwm[2]);
}
