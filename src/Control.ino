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

/* PID Constants*/
// trial and error tuning:
// double kp1 = 1.2, ki1 = 0.000032, kd1 = 1.5;
// double kp2 = 1.2, ki2 = 0.000031, kd2 = 1.5;
// double kp3 = 1.2, ki3 = 0.000031, kd3 = 1.5;

//discrete PI:
double kp1 = 0.507, ki1 = 6.95;
double kp2 = 0.496, ki2 = 7.14;
double kp3 = 0.702, ki3 = 7.0;
double prevPWM1 = 0.0;
double prevPWM2 = 0.0;
double prevPWM3 = 0.0;
int current_time = 0, prev_time=0;
void regression()
{
  motorPwm1 = wheelVelocity1_Target/0.4617 + 2.2108/0.4617;
  motorPwm2 = wheelVelocity2_Target/0.4275 + 1.544/0.4275;
  motorPwm3 = wheelVelocity3_Target/0.4541 + 0.7405/0.4541;
}

void velocityToPwm(){
  wheelVelocity1_Error = wheelVelocity1_Target - wheelVelocity1_Real;
  wheelVelocity2_Error = wheelVelocity2_Target - wheelVelocity2_Real;
  wheelVelocity3_Error = wheelVelocity3_Target - wheelVelocity3_Real;
  
  
  wheelVelocity1_Error_Sum += wheelVelocity1_Error;
  wheelVelocity2_Error_Sum += wheelVelocity2_Error;
  wheelVelocity3_Error_Sum += wheelVelocity3_Error;
    
  // motorPwm1 = wheelVelocity1_Error * kp1 + wheelVelocity1_Error_Sum * ki1 + (wheelVelocity1_Error - wheelVelocity1_Prev_Error) * kd1;
  // motorPwm2 = wheelVelocity2_Error * kp2 + wheelVelocity2_Error_Sum * ki2 + (wheelVelocity2_Error - wheelVelocity2_Prev_Error) * kd2;
  // motorPwm3 = wheelVelocity3_Error * kp3 + wheelVelocity3_Error_Sum * ki3 + (wheelVelocity3_Error - wheelVelocity3_Prev_Error) * kd3;
  current_time = millis();
  if(current_time - prev_time >= 50){
    // motorPwm1 = prevPWM1 + wheelVelocity1_Error * (kp1 + ki1 * 0.05) - wheelVelocity1_Prev_Error * kp1;
    // motorPwm2 = prevPWM2 + wheelVelocity2_Error * (kp2 + ki2 * 0.05) - wheelVelocity2_Prev_Error * kp2;
    // motorPwm3 = prevPWM3 + wheelVelocity3_Error * (kp3 + ki3 * 0.05) - wheelVelocity3_Prev_Error * kp3;

    motorPwm1 = prevPWM1 + wheelVelocity1_Error * kp1 + wheelVelocity1_Prev_Error * (ki1 * 0.05 - kp1);
    motorPwm2 = prevPWM2 + wheelVelocity2_Error * kp2 + wheelVelocity2_Prev_Error * (ki2 * 0.05 - kp2);
    motorPwm3 = prevPWM3 + wheelVelocity3_Error * kp3 + wheelVelocity3_Prev_Error * (ki3 * 0.05 - kp3);
    
    motorPwm1 = constrain(motorPwm1, -255, 255);
    motorPwm2 = constrain(motorPwm2, -255, 255);
    motorPwm3 = constrain(motorPwm3, -255, 255);

    prevPWM1 = motorPwm1;
    prevPWM2 = motorPwm2;
    prevPWM3 = motorPwm3;
    wheelVelocity1_Prev_Error = wheelVelocity1_Error;
    wheelVelocity2_Prev_Error = wheelVelocity2_Error;
    wheelVelocity3_Prev_Error = wheelVelocity3_Error;
    prev_time = current_time;
  }
    

  if(abs(wheelVelocity1_Real) < 3 && wheelVelocity1_Target == 0){
    motorPwm1 = 0;
    // wheelVelocity1_Error_Sum = 0;
    wheelVelocity1_Prev_Error = 0.0;
    wheelVelocity1_Error = 0.0;
    prevPWM1 = 0.0;
  }
  if(abs(wheelVelocity2_Real) < 3 && wheelVelocity2_Target == 0){
    motorPwm2 = 0;
    // wheelVelocity2_Error_Sum = 0;
    wheelVelocity2_Prev_Error = 0.0;
    wheelVelocity2_Error = 0.0;
    prevPWM2 = 0.0;
  }
  if(abs(wheelVelocity3_Real) < 3 && wheelVelocity3_Target == 0){
    motorPwm3 = 0;
    // wheelVelocity3_Error_Sum = 0;
    wheelVelocity3_Prev_Error = 0.0;
    wheelVelocity3_Error = 0.0;
    prevPWM3 = 0.0;
  }
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
    vx_Local = (vx_Local/totalSpeed)*MAX_ROBOT_SPEED;
    vy_Local = (vy_Local/totalSpeed)*MAX_ROBOT_SPEED;
  }

  inverseKinematics(vx_Local, vy_Local, omega);

  double acc_thresh = 5.0;
  const double dcc_thresh = acc_thresh*1.49;  //m/s^2
  float acc_thresh_ratio = 1;
   
  wheelAcc1 = wheelVelocity1_Target - wheelVelocity1_Prev_Target;
  float acc_thresh_ratio_temp = 0;
  if( wheelAcc1*wheelVelocity1_Prev_Target>=0 ) //speed up
    acc_thresh_ratio_temp = fabs(wheelAcc1)/acc_thresh;
  else                                 //speed down
    acc_thresh_ratio_temp = fabs(wheelAcc1)/dcc_thresh;
  if( acc_thresh_ratio_temp>acc_thresh_ratio ){
    wheelAcc1/= acc_thresh_ratio_temp;
    wheelVelocity1_Target = wheelVelocity1_Prev_Target + wheelAcc1;
  }

  wheelAcc2 = wheelVelocity2_Target - wheelVelocity2_Prev_Target;
  acc_thresh_ratio_temp = 0;
  if( wheelAcc2*wheelVelocity2_Prev_Target>=0 ) //speed up
    acc_thresh_ratio_temp = fabs(wheelAcc2)/acc_thresh;
  else                                 //speed down
    acc_thresh_ratio_temp = fabs(wheelAcc2)/dcc_thresh;
  if( acc_thresh_ratio_temp>acc_thresh_ratio ){
    wheelAcc2/= acc_thresh_ratio_temp;
    wheelVelocity2_Target = wheelVelocity2_Prev_Target + wheelAcc2;
  }

  wheelAcc3 = wheelVelocity3_Target - wheelVelocity3_Prev_Target;
  acc_thresh_ratio_temp = 0;
  if( wheelAcc3*wheelVelocity3_Prev_Target>=0 ) //speed up
    acc_thresh_ratio_temp = fabs(wheelAcc3)/acc_thresh;
  else                                 //speed down
    acc_thresh_ratio_temp = fabs(wheelAcc3)/dcc_thresh;
  if( acc_thresh_ratio_temp>acc_thresh_ratio ){
    wheelAcc3/= acc_thresh_ratio_temp;
    wheelVelocity3_Target = wheelVelocity3_Prev_Target + wheelAcc3;
  }
//            acc_thresh_ratio = acc_thresh_ratio_temp;
    ////bad programming

//    if( acc_thresh_ratio > 1 )
//    {
//        for(int i=0; i<WHEELS; i++)
//        {
//            wheel_acc[i] /= acc_thresh_ratio;
//            wheel_speed[i] = wheel_speed_old[i] + wheel_acc[i];
//        }
//    }
    

    // if(hypot(Vx,Vy)*fabs(_w)*0.03>_acc_thresh) //kinda weird
    // {
    //     float v_wheel=0;
    //     for(int i=0; i<WHEELS; i++)
    //     {
    //         if( fabs(wheel_speed[i])>v_wheel )
    //             v_wheel = fabs(wheel_speed[i]);
    //     }
    //     if(v_wheel<_acc_thresh)
    //         v_wheel = _acc_thresh;
    //     for(int i=0; i<WHEELS; i++)
    //         wheel_speed[i] *= (1-_acc_thresh/v_wheel);

    //     Vx =-0.57735*  wheel_speed[0] + 0.57735*wheel_speed[1]  + 0      *wheel_speed[2];
    //     Vy = 0.33333*  wheel_speed[0] + 0.33333*wheel_speed[1]  - 0.66666*wheel_speed[2];
    //     _w  = 1.7094*  wheel_speed[0] + 1.7094*wheel_speed[1]  + 1.7094*wheel_speed[2];
    // }
    // if(use_convected_acc) //more weird
    // {
    //     float temp = Vx;
    //     Vx -= -Vy*_w*0.1;
    //     Vy -= temp*_w*0.1;
    // }
  wheelVelocity1_Prev_Target = wheelVelocity1_Target;
  wheelVelocity2_Prev_Target = wheelVelocity2_Target;
  wheelVelocity3_Prev_Target = wheelVelocity3_Target;

  velocityToPwm();
  robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
}
