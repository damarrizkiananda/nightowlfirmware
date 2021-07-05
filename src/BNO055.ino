
/*
 *          0
 *          I   
 *    +90   I    -90
 *          I
 *          I
 *        +/-180
 * 
 * 
 */

void readBNO055(sensors_event_t* event1, sensors_event_t* event2) 
{
  bno.getEvent(event1, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(event2, Adafruit_BNO055::VECTOR_LINEARACCEL);

  theta_BNO055 = event1->orientation.x;
  theta360 = theta_BNO055;
  if(theta_BNO055 > 180) theta_BNO055 -= 360;
  theta_BNO055 = theta_BNO055*(-1);
  ax = event2->acceleration.x;
  ay = event2->acceleration.y;
  az = event2->acceleration.z;
  vx = ACCEL_VEL_TRANSITION * event2->acceleration.x;
  vy = ACCEL_VEL_TRANSITION * event2->acceleration.y;
  if (vx < 0.25) vx = 0.0;
  if (vy < 0.25) vy = 0.0;
  xPos = xPos + vx * ACCEL_VEL_TRANSITION;
  yPos = yPos + vy * ACCEL_VEL_TRANSITION;
  headingVel = ACCEL_VEL_TRANSITION * event2->acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
}

void getBNO055Data()
{
  readBNO055(&orientationData, &linearAccelData);
}
