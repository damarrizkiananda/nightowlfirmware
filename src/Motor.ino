void motorWrite(int pwmInput, int pinA, int pinB)
{
  pwmInput = constrain(pwmInput, -255, 255);
  if(pwmInput>20)
  {
    analogWrite(pinA, pwmInput);
    analogWrite(pinB, 0);
  }
  else if(pwmInput<-20)
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

void robotMotorWrite(int pwm1, int pwm2, int pwm3)
{
  motorWrite(pwm1, MOTOR_1_A_PIN, MOTOR_1_B_PIN);
  motorWrite(pwm2, MOTOR_2_A_PIN, MOTOR_2_B_PIN);
  motorWrite(pwm3, MOTOR_3_A_PIN, MOTOR_3_B_PIN);
}
