
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

void readBNO055(sensors_event_t* event) 
{
  bno.getEvent(event, Adafruit_BNO055::VECTOR_EULER);

  theta_BNO055 = event->orientation.x;
  if(theta_BNO055 > 180) theta_BNO055 -= 360;
  theta_BNO055 = theta_BNO055*(-1);
}

void getThetaBNO055Deg()
{
  readBNO055(&orientationData);
}
