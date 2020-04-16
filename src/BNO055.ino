
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

  yaw = event->orientation.x;
  if(yaw > 180) yaw -= 360;
  yaw = yaw*(-1);
}

void getYawDeg()
{
  readBNO055(&orientationData);
}
