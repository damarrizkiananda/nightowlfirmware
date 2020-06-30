
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

  theta_Real = event->orientation.x;
  if(theta_Real > 180) theta_Real -= 360;
  theta_Real = theta_Real*(-1);
}

void getYawDeg()
{
  readBNO055(&orientationData);
}
