signed long lastPulse1, lastPulse2, lastPulse3;

void update_speed()
{
  encoderPulseDif1 = (int32_t)(enc1.read()/4-lastPulse1);
  encoderPulseDif2 = (int32_t)(enc2.read()/4-lastPulse2);
  encoderPulseDif3 = (int32_t)(enc3.read()/4-lastPulse3);

  lastPulse1 = enc1.read()/4;
  lastPulse2 = enc2.read()/4;
  lastPulse3 = enc3.read()/4;  

  wheelSpeed1_Real = encoderPulseDif1;
  wheelSpeed2_Real = encoderPulseDif2;
  wheelSpeed3_Real = encoderPulseDif3;

  getPosition();

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



