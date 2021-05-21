signed long lastCount1, lastCount2, lastCount3;
double lastX, lastY, lastYaw;

int counter;
bool ledState;

void updateVelocityAndPosition()
{
  enc1_read=enc1.read();
  enc2_read=enc2.read();
  enc3_read=enc3.read();

  encoderCountDif1 = enc1_read-lastCount1;
  encoderCountDif2 = enc2_read-lastCount2;
  encoderCountDif3 = enc3_read-lastCount3;

  lastCount1 = enc1_read;
  lastCount2 = enc2_read;
  lastCount3 = enc3_read;  

  wheelVelocity1_Real = (double)encoderCountDif1  * TWO_PI * 5 * 1000.0 / (CPR * TIMER_INTERRUPT_PERIOD);
  wheelVelocity2_Real = (double)encoderCountDif2  * TWO_PI * 5 * 1000.0 / (CPR * TIMER_INTERRUPT_PERIOD);
  wheelVelocity3_Real = (double)encoderCountDif3  * TWO_PI * 5 * 1000.0 / (CPR * TIMER_INTERRUPT_PERIOD);

  getThetaBNO055Deg();
  theta_Real = theta_BNO055;
  calculatePosition();

  robotVelocityX_Real = (x_Real-lastX) * 1000.0 / TIMER_INTERRUPT_PERIOD;
  robotVelocityY_Real = (y_Real-lastY) * 1000.0 / TIMER_INTERRUPT_PERIOD;
  robotOmega_Real     = (theta_Real-lastYaw) * 100000.0 / TIMER_INTERRUPT_PERIOD; // 100 Degree/s
  // robotOmega_Real     = (theta_Real-lastYaw) * 1000 / TIMER_INTERRUPT_PERIOD;

  lastX = x_Real;
  lastY = y_Real;
  lastYaw = theta_Real;

  // Blink LED
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

void calculatePosition()
{
  sin_theta = sin(theta_Real*TO_RAD);
  cos_theta = cos(theta_Real*TO_RAD);
  
  y_count += (sqrt3 * sin_theta * (encoderCountDif1 - encoderCountDif2) + cos_theta * (encoderCountDif1 + encoderCountDif2 - 2 * encoderCountDif3)) / 3;
  x_count -= (-sqrt3 * cos_theta * (encoderCountDif1 - encoderCountDif2) + sin_theta * (encoderCountDif1 + encoderCountDif2 - 2 * encoderCountDif3)) / 3;
       
  x_Real = (x_count * DIST_PER_COUNT);
  y_Real = (y_count * DIST_PER_COUNT);
}
