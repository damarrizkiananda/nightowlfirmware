
/*
 *          0
 *     +T   I   -T
 *          I
 *          I
 *          I
 * 
 * 
 */

void readBNO055(sensors_event_t* event) 
{
  bno.getEvent(event, Adafruit_BNO055::VECTOR_EULER);

  yaw_Real = event->orientation.x;
  if(yaw_Real > 180) yaw_Real -= 360;
  yaw_Real = yaw_Real*(-1);
}

void getYawDeg()
{
  readBNO055(&orientationData);
}
