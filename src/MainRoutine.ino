
const byte filterVal = 15;
int filteredPwm1, filteredPwmPrev1;
int filteredPwm2, filteredPwmPrev2;
int filteredPwm3, filteredPwmPrev3;

void NightOwlMain()
{
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
    
  }
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
    Serial.print(x_Real);
    Serial.print("   y:");
    Serial.print(y_Real);
    Serial.print("   yaw:");
    Serial.println(yaw_Real);
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
    for(int i = 0; i<=3; i++) Serial.print(bluetooth_buffer[i]);
    Serial.println();
    //bluetooth_send_packets(data, 3);
    newDestination = false;
  }
}

void timerCheck()
{
   if(velocityAndPositionUpdated == true)
  {
    Serial.println(millis());
    velocityAndPositionUpdated = false;
  }
}

void odometryCheck()
{
  if(millis()-CT>200)
  {
    //getYawDeg();
    Serial.print(" x:"); Serial.print(x_Real);
    Serial.print(" y:"); Serial.print(y_Real);
    Serial.print(" yaw:"); Serial.print(yaw_Real);
    Serial.println();
    CT = millis();
    //sendDataPlease = false;
  }
}

#define STATE_GO 0
#define STATE_WAIT 1
#define STATE_SEND_DATA 2

int bluetoothX, bluetoothY, bluetoothO;
bool go = false;
unsigned long bc, btime;
int state = STATE_WAIT;
int path = 1;
int printCounter;
int copyOfX, copyOfY, copyOfYaw;

void mainMain()
{ 
  if(state==STATE_WAIT)
  {
    if(BLUETOOTH_SERIAL.available())
    {
      bluetoothX = BLUETOOTH_SERIAL.parseInt();
      //bluetoothY = BLUETOOTH_SERIAL.parseInt();
      //bluetoothO = BLUETOOTH_SERIAL.parseInt();
      //btime = BLUETOOTH_SERIAL.parseInt();
      //bc = millis();
      Serial.println("Going");
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
    if(velocityAndPositionUpdated == true)
    {
      printCounter++;
      velocityAndPositionUpdated = false;
    }
    if(printCounter>=4)
    {
      noInterrupts();
      copyOfX = x_Real;
      copyOfY = y_Real;
      copyOfYaw = yaw_Real;
      interrupts();
      Serial.print(copyOfX);Serial.print(";");
      Serial.print(copyOfY);Serial.print(";");
      Serial.println(copyOfYaw);
      printCounter = 0;
    }
    if(path==1)
    {
      positionPID(200,0,0);
      if(abs(x_Real-200)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==2)
    {
      positionPID(400,0,0);
      if(abs(x_Real-400)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==3)
    {
      positionPID(600,0,0);
      if(abs(x_Real-600)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==4)
    {
      positionPID(800,0,0);
      if(abs(x_Real-800)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==5)
    {
      positionPID(1000,0,0);
      if(abs(x_Real-1000)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==6)
    {
      positionPID(1150,0,0);
      if(abs(x_Real-1150)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==7)
    {
      positionPID(1345,0,0);
      if(abs(x_Real-1345)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==8) /* Belok */
    {
      positionPID(1345,0,-90);
      if(abs(yaw_Real+90)<5)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==9)
    {
      positionPID(1345,-300,-90);
      if(abs(y_Real+300)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==10)
    {
      positionPID(1345,-600,-90);
      if(abs(y_Real+600)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==11)
    {
      positionPID(1345,-900,-90);
      if(abs(y_Real+900)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==12)
    {
      positionPID(1345,-1200,-90);
      if(abs(y_Real+1200)<15)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==13)
    {
      positionPID(1345,-1200,120);
      if(abs(yaw_Real-120)<5)
      {
        Serial.print("Target Reached");
        robotMotorWrite(0,0,0);
        path++;
      }
    }
    else if(path==14) 
    {
      state = STATE_SEND_DATA;
    }
  }

  else if(state==STATE_SEND_DATA)
  {
    Serial.print(" x:"); Serial.print(x_Real);
    Serial.print(" y:"); Serial.print(y_Real);
    Serial.print(" yaw:"); Serial.println(yaw_Real);
    state = STATE_WAIT;
    Serial.println("Waiting for new data");
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
  Serial.print(wheelVelocity1_Target);
  Serial.print(" ");
  Serial.print(wheelVelocity2_Target);
  Serial.print(" ");
  Serial.println(wheelVelocity3_Target);
  delay(10);  
}
