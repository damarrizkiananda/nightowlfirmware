signed long lastPulse1, lastPulse2, lastPulse3;
double lastX, lastY, lastYaw;

void updateSpeedAndPosition()
{
  encoderPulseDif1 = (int32_t)(enc1.read()/4-lastPulse1);
  encoderPulseDif2 = (int32_t)(enc2.read()/4-lastPulse2);
  encoderPulseDif3 = (int32_t)(enc3.read()/4-lastPulse3);

  lastPulse1 = enc1.read()/4;
  lastPulse2 = enc2.read()/4;
  lastPulse3 = enc3.read()/4;  

  wheelSpeed1_Real = encoderPulseDif1  * TWO_PI * 1000 / (PPR * TIMER_INTERRUPT_PERIOD);
  wheelSpeed2_Real = encoderPulseDif2  * TWO_PI * 1000 / (PPR * TIMER_INTERRUPT_PERIOD);
  wheelSpeed3_Real = encoderPulseDif3  * TWO_PI * 1000 / (PPR * TIMER_INTERRUPT_PERIOD);

  getYawDeg();
  getPosition();

  robotSpeedX_Real = (x-lastX) * 1000 / TIMER_INTERRUPT_PERIOD;
  robotSpeedY_Real = (y-lastY) * 1000 / TIMER_INTERRUPT_PERIOD;
  robotOmega_Real  = (yaw-lastYaw) * TO_RAD * 1000 / TIMER_INTERRUPT_PERIOD;

  sendDataPlease = true;
}

#define sqrt3 1.73205080757
double sin_theta, cos_theta;
double x_count, y_count;

void getPosition()
{
  sin_theta = sin(yaw*TO_RAD);
  cos_theta = cos(yaw*TO_RAD);
  
  y_count += (sqrt3 * sin_theta * (encoderPulseDif1 - encoderPulseDif2) + cos_theta * (encoderPulseDif1 + encoderPulseDif2 - 2 * encoderPulseDif3)) / 3;
  x_count -= (-sqrt3 * cos_theta * (encoderPulseDif1 - encoderPulseDif2) + sin_theta * (encoderPulseDif1 + encoderPulseDif2 - 2 * encoderPulseDif3)) / 3;
       
  x = (int)(x_count * DIST_PER_PULSE);
  y = (int)(y_count * DIST_PER_PULSE);
}
