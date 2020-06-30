signed long lastPulse1, lastPulse2, lastPulse3;
double lastX, lastY, lastYaw;

int counter;
bool ledState;

void updateVelocityAndPosition()
{
  encoderPulseDif1 = (int32_t)(enc1.read()/4-lastPulse1);
  encoderPulseDif2 = (int32_t)(enc2.read()/4-lastPulse2);
  encoderPulseDif3 = (int32_t)(enc3.read()/4-lastPulse3);

  lastPulse1 = enc1.read()/4;
  lastPulse2 = enc2.read()/4;
  lastPulse3 = enc3.read()/4;  

  wheelVelocity1_Real = (double)encoderPulseDif1  * TWO_PI * 1000.0 / (PPR * TIMER_INTERRUPT_PERIOD);
  wheelVelocity2_Real = (double)encoderPulseDif2  * TWO_PI * 1000.0 / (PPR * TIMER_INTERRUPT_PERIOD);
  wheelVelocity3_Real = (double)encoderPulseDif3  * TWO_PI * 1000.0 / (PPR * TIMER_INTERRUPT_PERIOD);

  getYawDeg();
  getPosition();

  robotVelocityX_Real = (x_Real-lastX) * 1000 / TIMER_INTERRUPT_PERIOD;
  robotVelocityY_Real = (y_Real-lastY) * 1000 / TIMER_INTERRUPT_PERIOD;
  // robotOmega_Real     = (yaw_Real-lastYaw) * TO_RAD * 1000 / TIMER_INTERRUPT_PERIOD;
  robotOmega_Real     = (yaw_Real-lastYaw) * 1000 / TIMER_INTERRUPT_PERIOD;


  lastX = x_Real;
  lastY = y_Real;
  lastYaw = yaw_Real;

  counter++;
  if(counter>20)
  {
    ledState = !ledState;
    counter = 0;
  }
  digitalWrite(13, ledState);


  velocityAndPositionUpdated = true;
}

#define sqrt3 1.73205080757
double sin_theta, cos_theta;
double x_count, y_count;

void getPosition()
{
  sin_theta = sin(yaw_Real*TO_RAD);
  cos_theta = cos(yaw_Real*TO_RAD);
  
  y_count += (sqrt3 * sin_theta * (encoderPulseDif1 - encoderPulseDif2) + cos_theta * (encoderPulseDif1 + encoderPulseDif2 - 2 * encoderPulseDif3)) / 3;
  x_count -= (-sqrt3 * cos_theta * (encoderPulseDif1 - encoderPulseDif2) + sin_theta * (encoderPulseDif1 + encoderPulseDif2 - 2 * encoderPulseDif3)) / 3;
       
  x_Real = (int)(x_count * DIST_PER_PULSE);
  y_Real = (int)(y_count * DIST_PER_PULSE);
}
