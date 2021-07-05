signed long lastCount[3] = {0, 0, 0}, lastCount_50[3] = {0, 0, 0};
double lastX, lastY, lastYaw;

int counter = 0, counter2 = 0;
bool ledState;

void updateVelocityAndPosition()
{
  enc_read[0]=enc1.read();
  enc_read[1]=enc2.read();
  enc_read[2]=enc3.read();

  for(int i = 0; i < 3; i++){
    encoderCountDif[i] = enc_read[i]-lastCount[i];
    lastCount[i] = enc_read[i];
    wheelVelocity_Real[i] = (double)encoderCountDif[i] * TWO_PI * 5 * 1000.0 / (CPR * TIMER_INTERRUPT_PERIOD);
  }

  if(counter2>=5){
    for(int i = 0; i < 3; i++){
      // enc_read_50[i] = enc_read[i];
      // encoderCountDif_50[i] = enc_read_50[i]-lastCount_50[i];
      // lastCount_50[i] = enc_read_50[i];
      wheelVelocity_Real_50[i] = wheelVelocity_Real[i]; // (double)encoderCountDif_50[i] * TWO_PI * 5 * 1000.0 / (CPR * 50);
      counter2 = 0;
    }
  }
  getBNO055Data();
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
  counter2++;
  if(counter>=100)
  {
    ledState = !ledState;
    counter = 0;
  }
  digitalWrite(13, ledState);


  velocityAndPositionUpdated = true;
}

#define sqrt3 1.73205080757
double sin_theta, cos_theta;
double x_count, y_count, x_count_local, y_count_local, x_count_global, y_count_global;

void calculatePosition()
{
  sin_theta = sin(theta_Real*TO_RAD);
  cos_theta = cos(theta_Real*TO_RAD);
  
  x_count -= (-sqrt3 * cos_theta * (encoderCountDif[0] - encoderCountDif[1]*(1/0.96)) + sin_theta * (encoderCountDif[0] + encoderCountDif[1]*(1/0.96) - 2 * encoderCountDif[2]*(1/1.03))) / 3;
  y_count += (sqrt3 * sin_theta * (encoderCountDif[0] - encoderCountDif[1]*(1/0.96)) + cos_theta * (encoderCountDif[0] + encoderCountDif[1]*(1/0.96) - 2 * encoderCountDif[2]*(1/1.03))) / 3;
       
  x_Real = (x_count * DIST_PER_COUNT);
  y_Real = (y_count * DIST_PER_COUNT);

  
  // x_count_local = (sqrt3 * (enc1_read - enc2_read*(1/0.95))) / 3;
  // y_count_local = (enc1_read + enc2_read*(1/0.95) - 2 * enc3_read*(1/1.03)) / 3;
  // x_count_global = (sqrt3 * cos_theta * (enc1_read - enc2_read*(1/0.95)) - sin_theta * (enc1_read + enc2_read*(1/0.95) - 2 * enc3_read*(1/1.03))) / 3;
  // y_count_global = (sqrt3 * sin_theta * (enc1_read - enc2_read*(1/0.95)) + cos_theta * (enc1_read + enc2_read*(1/0.95) - 2 * enc3_read*(1/1.03))) / 3;
  // x_Real = (x_count_global * DIST_PER_COUNT);
  // y_Real = (y_count_global * DIST_PER_COUNT);
}
