
void NightOwlMain()
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
    serial_receive();
    robotMotorWrite(pwm1, pwm2, pwm3); 

    if(arrived)
    {
      send_to_tablet();
      robotState = WAITING_FOR_NEW_DESTINATION;
    }
  }
}

unsigned long CT = 0;

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
  
  //robotMotorWrite(pwm1, pwm2, pwm3);

  //Serial.println(spd);
//  noInterrupts();
//  rpm1Copy = rpm1;
//  rpm2Copy = rpm2;
//  rpm3Copy = rpm3;
//  interrupts();

  /*Encoder Check */
  //Serial.print(" RPM 1:"); Serial.print(rpm1Copy);
  //Serial.print(" RPM 2:"); Serial.print(rpm2Copy);
  //Serial.print(" RPM 3:"); Serial.print(rpm3Copy);
  //Serial.print(" YAW:");   Serial.println(yaw);

  //delay(10);
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

int bluetoothX, bluetoothY, bluetoothO, btime, go;
unsigned long bc;

void mainMain()
{
  if(BLUETOOTH_SERIAL.available())
  {
    bluetoothX = BLUETOOTH_SERIAL.parseInt();
    bluetoothY = BLUETOOTH_SERIAL.parseInt();
    bluetoothO = BLUETOOTH_SERIAL.parseInt();
    btime = BLUETOOTH_SERIAL.parseInt();
    bc = millis();
    go = true;
  }

  if(go)
  {
    inverseKinematics(bluetoothX, bluetoothY, bluetoothO);
    
    /* Without motor speed control */
    robotMotorWrite(pwm1,pwm2,pwm3);
    //delay(1000);
    if(millis()-bc>btime)
    {
      robotMotorWrite(0,0,0);
      go = false;
      //bc = millis(); 
    }
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




