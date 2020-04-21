signed long lastCounter1, lastCounter2, lastCounter3;

void update_speed()
{
  speed1 = (int32_t)(enc1.read()/4-lastCounter1);
  speed2 = (int32_t)(enc2.read()/4-lastCounter2);
  speed3 = (int32_t)(enc3.read()/4-lastCounter3);

  lastCounter1 = enc1.read()/4;
  lastCounter2 = enc2.read()/4;
  lastCounter3 = enc3.read()/4;  

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
  
  y_count += (sqrt3 * sin_theta * (speed1 - speed2) + cos_theta * (speed1 + speed2 - 2 * speed3)) / 3;
  x_count -= (-sqrt3 * cos_theta * (speed1 - speed2) + sin_theta * (speed1 + speed2 - 2 * speed3)) / 3;
       
  x = (int)(x_count * DIST_PER_PULSE);
  y = (int)(y_count * DIST_PER_PULSE);
   
}



