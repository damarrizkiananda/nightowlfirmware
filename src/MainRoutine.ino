
#define AUTOMATIC 0
#define MANUAL    1
#define INDEPENDENT_WHEEL 2
#define STEP_INPUT 3
#define MANUAL_V  60
#define MANUAL_OM 100
char bluetoothCommand = 'z';
int manualVX = 0, manualVY = 0, manualOmega = 0, mode = MANUAL;
int currentMillis=0, previousMillis=0, interval=50;
int count=0;
void NightOwlLoop()
{
  currentMillis=millis();
  if(currentMillis - previousMillis >= interval){
    updateVelocityAndPosition();
    previousMillis=currentMillis;
  }
  serial_receive();
  if(velocityAndPositionUpdated == true)
  {
    send_to_laptop();  
    // Serial.print(digitalRead(IR_FRONT_PIN));
    // Serial.print(digitalRead(IR_BACK_PIN));
    // Serial.print(digitalRead(IR_RIGHT_PIN));
    // Serial.println(digitalRead(IR_LEFT_PIN));
    // if(((wheelVelocity1_Target || wheelVelocity2_Target || wheelVelocity3_Target) != 0) && count <= 250){
    //   Serial.println(wheelVelocity3_Real);
    //   if(count >= 50 && count < 100){
    //     // bluetoothCommand = 's';
    //     wheelVelocity1_Target = 20;
    //     wheelVelocity2_Target = 20;
    //     wheelVelocity3_Target = 20;
    //   }
    //   else if(count >= 100 && count < 150){
    //     // bluetoothCommand = 's';
    //     wheelVelocity1_Target = 40;
    //     wheelVelocity2_Target = 40;
    //     wheelVelocity3_Target = 40;
    //   }
    //   else if(count >= 150 && count < 200){
    //     // bluetoothCommand = 's';
    //     wheelVelocity1_Target = 60;
    //     wheelVelocity2_Target = 60;
    //     wheelVelocity3_Target = 60;
    //   }
    //   else if(count >= 200 && count < 250){
    //     // bluetoothCommand = 's';
    //     wheelVelocity1_Target = 30;
    //     wheelVelocity2_Target = 30;
    //     wheelVelocity3_Target = 30;
    //   }
    //   else if(count >= 250){
    //     // bluetoothCommand = 's';
    //     wheelVelocity1_Target = 0;
    //     wheelVelocity2_Target = 0;
    //     wheelVelocity3_Target = 0;
    //   }
    //   count++;
    // }
    // else if(count > 250 && count < 300){
    //   Serial.println(wheelVelocity3_Real);
    //   count++;
    // }

    // if(count >= 200 && count <=250){
    //   count++;
    //   // bluetoothCommand = 's';
    //   // wheelVelocity1_Target = 0;
    //   // wheelVelocity2_Target = 0;
    //   // wheelVelocity3_Target = 0;
    //   // motorPwm1 = 0;
    //   // motorPwm2 = 0;
    //   // motorPwm3 = 0;
    //   // robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
    //   // Serial.println(wheelVelocity2_Real);
    //   count = 0;
    // }

    // if(((motorPwm1 || motorPwm2 || motorPwm3) != 0) && count < 100){
    //   Serial.println(wheelVelocity3_Real);
    //   count++;
    // }
    // else{
    //   count = 0;
    //   motorPwm1 = 0;
    //   motorPwm2 = 0;
    //   motorPwm3 = 0;
    // }
    // if(((motorPwm1 || motorPwm2 || motorPwm3) != 0) && count < 100){
    //   Serial.println(wheelVelocity1_Real);
    //   count++;
    // }
    // if(count == 100) Serial.println();
    // if(count >= 100 && count < 120){
    //   bluetoothCommand = 's';
    //   count++;
    // }
    // if(/*((motorPwm1 || motorPwm2 || motorPwm3) != 0) &&*/ count >= 120 && count < 220){
    //   bluetoothCommand = '2';
    //   Serial.println(wheelVelocity2_Real);
    //   count++;
    // }
    // if(count == 220) Serial.println();
    // if(count >= 220 && count < 240){
    //   bluetoothCommand = 's';
    //   count++;
    // }
    // if(/*((motorPwm1 || motorPwm2 || motorPwm3) != 0) &&*/ count >= 240 && count < 340){
    //   bluetoothCommand = '3';
    //   Serial.println(wheelVelocity3_Real);
    //   count++;
    // }
    // if(count == 340) Serial.println();
    // if(count >= 340){
    //   bluetoothCommand = 's';
    //   count = 0;
    // }
    
    // if(count>20)
    // {

    // //   BLUETOOTH_SERIAL.print("x:"); BLUETOOTH_SERIAL.print(x_Real);
    // //   BLUETOOTH_SERIAL.print(" y:"); BLUETOOTH_SERIAL.println(y_Real);
    //   // BLUETOOTH_SERIAL.print("vx:"); BLUETOOTH_SERIAL.print(robotVelocityX_Target);
    //   // BLUETOOTH_SERIAL.print(" vy:"); BLUETOOTH_SERIAL.println(robotVelocityY_Target);

    //   // BLUETOOTH_SERIAL.print(" xcount:"); BLUETOOTH_SERIAL.print(x_count);
    //   // BLUETOOTH_SERIAL.print(" ycount:"); BLUETOOTH_SERIAL.println(y_count);

    //   Serial.print("v1target: "); Serial.print(wheelVelocity1_Target);
    //   Serial.print("  v2target: "); Serial.print(wheelVelocity2_Target);
    //   Serial.print("  v3target: "); Serial.println(wheelVelocity3_Target);

    //   Serial.print("      v1: "); Serial.print(wheelVelocity1_Real);
    //   Serial.print("         v2: "); Serial.print(wheelVelocity2_Real);
    //   Serial.print("        v3: "); Serial.println(wheelVelocity3_Real);


    //   Serial.print("    PWM1: "); Serial.print(motorPwm1);
    //   Serial.print("       PWM2: "); Serial.print(motorPwm2);
    //   Serial.print("      PWM3: "); Serial.println(motorPwm3);

    //   Serial.println();
    //   BLUETOOTH_SERIAL.print("v1: "); BLUETOOTH_SERIAL.print(wheelVelocity1_Real);
    //   BLUETOOTH_SERIAL.print(" v2: "); BLUETOOTH_SERIAL.print(wheelVelocity2_Real);
    //   BLUETOOTH_SERIAL.print(" v3: "); BLUETOOTH_SERIAL.println(wheelVelocity3_Real);

    //   BLUETOOTH_SERIAL.print("v1target: "); BLUETOOTH_SERIAL.print(wheelVelocity1_Target);
    //   BLUETOOTH_SERIAL.print(" v2target: "); BLUETOOTH_SERIAL.print(wheelVelocity2_Target);
    //   BLUETOOTH_SERIAL.print(" v3target: "); BLUETOOTH_SERIAL.println(wheelVelocity3_Target);

    //   // BLUETOOTH_SERIAL.print("PWM1: "); BLUETOOTH_SERIAL.print(motorPwm1);
    //   // BLUETOOTH_SERIAL.print(" PWM2: "); BLUETOOTH_SERIAL.print(motorPwm2);
    //   // BLUETOOTH_SERIAL.print(" PWM3: "); BLUETOOTH_SERIAL.println(motorPwm3);

    //   // BLUETOOTH_SERIAL.print("enc1:"); BLUETOOTH_SERIAL.print(a);
    //   // BLUETOOTH_SERIAL.print(" enc2:"); BLUETOOTH_SERIAL.print(b);
    //   // BLUETOOTH_SERIAL.print(" enc3:"); BLUETOOTH_SERIAL.println(c);
    //   count=0;


    // }

    // count++;
    
    // signed long encoder1=enc1.read();
    // Serial.println(encoder1);
    velocityAndPositionUpdated = false;
  }

  if(BLUETOOTH_SERIAL.available()){
    bluetoothCommand = BLUETOOTH_SERIAL.read();
  }

  if(bluetoothCommand == 'f'){
    manualVX = MANUAL_V;
  }
  else if(bluetoothCommand == 'b'){
    manualVX = -MANUAL_V;
  }
  else if(bluetoothCommand == 'l'){
    manualVY = MANUAL_V;
  }
  else if(bluetoothCommand == 'r'){
    manualVY = -MANUAL_V;
  }
  else if(bluetoothCommand == 'p'){
    wheelVelocity1_Target = 50.0;
    wheelVelocity2_Target = 50.0;
    wheelVelocity3_Target = 50.0;
    manualOmega = MANUAL_OM;
  }
  else if(bluetoothCommand == 'q')
  {
    wheelVelocity1_Target = -50.0;
    wheelVelocity2_Target = -50.0;
    wheelVelocity3_Target = -50.0;
    manualOmega = -MANUAL_OM;
  }
  else if(bluetoothCommand == 's'){
    manualVY = 0;
    manualVX = 0;
    manualOmega = 0;
    wheelVelocity1_Target = 0.0;
    wheelVelocity2_Target = 0.0;
    wheelVelocity3_Target = 0.0;
    motorPwm1 = 0;
    motorPwm2 = 0;
    motorPwm3 = 0;
    // regression();
  }
  else if(bluetoothCommand == 'a'){
    mode = AUTOMATIC;
  }
  else if(bluetoothCommand == 'm'){
    manualVY = 0;
    manualVX = 0;
    manualOmega = 0;
    mode = MANUAL;
  }
  else if(bluetoothCommand == 'w'){
    wheelVelocity1_Target = 0;
    wheelVelocity2_Target = 0;
    wheelVelocity3_Target = 0;
    mode = INDEPENDENT_WHEEL;
  }
  else if(bluetoothCommand == '1'){
    // wheelVelocity1_Target = 50.0;
    // wheelVelocity2_Target = 0.0;
    // wheelVelocity3_Target = 0.0;
    motorPwm1 = 150.0;
    motorPwm2 = 0.0;
    motorPwm3 = 0.0;
  }
  else if(bluetoothCommand == '2'){
    // wheelVelocity1_Target = 0;
    // wheelVelocity2_Target = 50;
    // wheelVelocity3_Target = 0;
    motorPwm1 = 0.0;
    motorPwm2 = 150.0;
    motorPwm3 = 0.0;
  }
  else if(bluetoothCommand == '3'){
    // wheelVelocity1_Target = 0;
    // wheelVelocity2_Target = 0;
    // wheelVelocity3_Target = 50;
    motorPwm1 = 0,0;
    motorPwm2 = 0,0;
    motorPwm3 = 150,0;
  }
  else if(bluetoothCommand == '4'){
    wheelVelocity1_Target = -40;
    wheelVelocity2_Target = 0;
    wheelVelocity3_Target = 0;
  }
  else if(bluetoothCommand == '5'){
    wheelVelocity1_Target = 0;
    wheelVelocity2_Target = -40;
    wheelVelocity3_Target = 0;
  }
  else if(bluetoothCommand == '6'){
    wheelVelocity1_Target = 0;
    wheelVelocity2_Target = 0;
    wheelVelocity3_Target = -40;
  }
  // else if(bluetoothCommand == 'u'){
  //   motorPwm1 = 150;
  //   motorPwm2 = 150;
  //   motorPwm3 = 150;
  // }
  bluetoothCommand = 'z';
  if(mode == MANUAL){
    moveRobotLocal(manualVX, manualVY, manualOmega);
  }
  else if(mode == INDEPENDENT_WHEEL){
    // velocityToPwm();
    robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
  }
  else if(mode == STEP_INPUT){
    robotMotorWrite(motorPwm1, motorPwm2, motorPwm3);
  }
  else moveRobotLocal(robotVelocityX_Target, robotVelocityY_Target, robotOmega_Target);
}
