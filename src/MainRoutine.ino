
const byte filterVal = 15;
int filteredPwm1, filteredPwmPrev1;
int filteredPwm2, filteredPwmPrev2;
int filteredPwm3, filteredPwmPrev3;

void NightOwlMain()
{

}

void NightOwlMainOld()
{
  /* Send data every 100ms */
  if(sendDataPlease)
  {
    getYawDeg();
    send_to_laptop();
  }  
  
  if(robotState == WAITING_FOR_NEW_DESTINATION)
  {
    robotMotorWrite(0, 0, 0); 
    bluetooth_receive();
    
    if(newDestination) 
    {
      robotState = MOVING;
      newDestination = false;   
    }
  }
  else if(robotState == MOVING)
  {
    // serial_receive();

    // filteredPwm1 = filter(filteredPwmPrev1, pwm1, filterVal);
    // filteredPwmPrev1 = filteredPwm1;

    // filteredPwm2 = filter(filteredPwmPrev2, pwm2, filterVal);
    // filteredPwmPrev2 = filteredPwm2;

    // filteredPwm3 = filter(filteredPwmPrev3, pwm3, filterVal);
    // filteredPwmPrev3 = filteredPwm3;

    // robotMotorWrite(filteredPwm1, filteredPwm2, filteredPwm3); 

    // if(arrived)
    // {
    //   send_to_tablet();
    //   robotState = WAITING_FOR_NEW_DESTINATION;
    // }
  }
}

unsigned long CT = 0;

/* PWMs without Sign & Their Parity */ 
int uPwm1, uPwm2, uPwm3, pwm1Parity, pwm2Parity, pwm3Parity, pwmParity;

/* PWMs with Sign */
int pwm1, pwm2, pwm3;

void vibeCheck()
{
  /* Motor Check */  
  if(Serial.available())
  {
    uPwm1 = Serial.parseInt();
    uPwm2 = Serial.parseInt();
    uPwm3 = Serial.parseInt();
    pwmParity = Serial.parseInt();
    pwm1Parity = pwmParity & 0x01;
    pwm2Parity = (pwmParity >> 1) & 0x01;
    pwm3Parity = (pwmParity >> 2) & 0x01;

    if(pwm1Parity) pwm1 = uPwm1*(-1);
    else pwm1 = uPwm1;
    if(pwm2Parity) pwm2 = uPwm2*(-1);
    else pwm2 = uPwm2;
    if(pwm3Parity) pwm3 = uPwm3*(-1);
    else pwm3 = uPwm3;
  }

  if(sendDataPlease){
    getYawDeg();
    //positionPID(uPwm1, uPwm2, 0);
    //motorPID(uPwm1, -uPwm1, 0);
    sendDataPlease = false; 

    Serial.print("vx:");
    Serial.print(OutputX);
    Serial.print("   vy:");
    Serial.print(OutputY);
    Serial.print("   x:");
    Serial.print(x);
    Serial.print("   y:");
    Serial.print(y);
    Serial.print("   yaw:");
    Serial.println(yaw);
  }
}

/* Use this main if Teensy is the main hardware controller */
void NightOwlControllerMain()
{
  
}

char data[3] = {'h','g','z'};

void bluetoothCheck()
{
  bluetooth_receive();
  if(newDestination)
  {
    Serial.println("Data Accepted, Sending Replies");
    bluetooth_send_packets(data, 3);
    newDestination = false;
  }
}

void timerCheck()
{
   if(sendDataPlease)
  {
    Serial.println(millis());
    sendDataPlease = false;
  }
}

void odometryCheck()
{
  if(sendDataPlease)
  {
    getYawDeg();
    Serial.print(" x:"); Serial.print(x);
    Serial.print(" y:"); Serial.print(y);
    Serial.print(" yaw:"); Serial.print(yaw);
    Serial.println();
    sendDataPlease = false;
  }
}

#define STATE_GO 0
#define STATE_WAIT 1
#define STATE_SEND_DATA 2

int bluetoothX, bluetoothY, bluetoothO;
bool go = false;
unsigned long bc, btime;
int state = STATE_WAIT;

void mainMain()
{
  if(sendDataPlease)
  {
    getYawDeg();
    sendDataPlease = false;
  }
  
  if(state==STATE_WAIT)
  {
    if(BLUETOOTH_SERIAL.available())
    {
      bluetoothX = BLUETOOTH_SERIAL.parseInt();
      bluetoothY = BLUETOOTH_SERIAL.parseInt();
      bluetoothO = BLUETOOTH_SERIAL.parseInt();
      btime = BLUETOOTH_SERIAL.parseInt();
      //bc = millis();
      state = STATE_GO;
    }

  // if(Serial.available())
  // {
  //   bluetoothX = Serial.parseInt();
  //   bluetoothY = Serial.parseInt();
  //   bluetoothO = Serial.parseInt();
  //   btime = Serial.parseInt();
  //   bc = millis();
  //   go = true;
  // }
  }
  else if (state==STATE_GO)
  {
    inverseKinematics(bluetoothX, bluetoothY, bluetoothO);

    /* Without motor Velocity control */
    //robotMotorWrite(pwm1,pwm2,pwm3);, 

    motorPID(pwm1, pwm2, pwm3);
    
  
    if(x>btime)
    {
      Serial.println("Time Ex");
      robotMotorWrite(0,0,0);
      go = false;
      state = STATE_SEND_DATA; 
    }
  }
  else if(state==STATE_SEND_DATA)
  {
    Serial.print(" x:"); Serial.print(x);
    Serial.print(" y:"); Serial.print(y);
    Serial.print(" yaw:"); Serial.print(yaw);
    state = STATE_WAIT;
  }
}

int inverseX, inverseY, inverseO;

void inverseCheck()
{
  if(Serial.available())
  {
    inverseX = Serial.parseInt();
    inverseY = Serial.parseInt();
    inverseO = Serial.parseInt();
  }

  inverseKinematics(inverseX, inverseY, inverseO);
  Serial.print(pwm1);
  Serial.print(" ");
  Serial.print(pwm2);
  Serial.print(" ");
  Serial.println(pwm3);
  delay(10);  
}




