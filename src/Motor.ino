
void motorWrite(int pwmInput, int pinA, int pinB)
{
  pwmInput = constrain(pwmInput, -255, 255);
  if(pwmInput>=40)
  {
    analogWrite(pinA, pwmInput);
    analogWrite(pinB, 0);
  }
  else if(pwmInput<=-40)
  {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwmInput);
  }
  else
  {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

#define MAX_PWM_STEP 25
int currentPwm1, currentPwm2, currentPwm3;
void robotMotorWrite(int pwm1, int pwm2, int pwm3)
{
  /* Limit Motor Acceleration Step */
  if(abs(pwm1-currentPwm1)>=MAX_PWM_STEP && abs(pwm1)>=40)
  {
    if(pwm1>0)currentPwm1 += MAX_PWM_STEP;
    else      currentPwm1 -= MAX_PWM_STEP;
  }
  else currentPwm1 = pwm1;
  
  if(abs(pwm2-currentPwm2)>=MAX_PWM_STEP && abs(pwm2)>=40)
  {
    if(pwm2>0)currentPwm2 += MAX_PWM_STEP;
    else      currentPwm2 -= MAX_PWM_STEP;
  }
  else currentPwm2 = pwm2;

  if(abs(pwm3-currentPwm3)>=MAX_PWM_STEP && abs(pwm3)>=40) 
  {
    if(pwm3>0)currentPwm3 += MAX_PWM_STEP;
    else      currentPwm3 -= MAX_PWM_STEP;
  }
  else currentPwm3 = pwm3;

  /* Output pwm to motor */ 
  motorWrite(currentPwm1, MOTOR_1_A_PIN, MOTOR_1_B_PIN);
  motorWrite(currentPwm2, MOTOR_2_A_PIN, MOTOR_2_B_PIN);
  motorWrite(currentPwm3, MOTOR_3_A_PIN, MOTOR_3_B_PIN);
}
