
#define AUTOMATIC 0
#define MANUAL    1
#define INDEPENDENT_WHEEL 2
#define STEP_INPUT 3
#define MANUAL_V  60
#define MANUAL_OM 100
char bluetoothCommand = 'z';
int manualVX = 0, manualVY = 0, manualOmega = 0, mode = AUTOMATIC;
int currentMillis=0, previousMillis=0, interval=10;
int count=0, count3=0;
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
    
    // Program undak
    /*if(count3 >= 5){
      if(((wheelVelocity_Target[0] || wheelVelocity_Target[1] || wheelVelocity_Target[2]) != 0) && count <= 300){
        if(count >= 0 && count < 50){
          wheelVelocity_Target[0] = 50;
          wheelVelocity_Target[1] = 50;
          wheelVelocity_Target[2] = 50;
        }
        else if(count >= 50 && count < 100){
          wheelVelocity_Target[0] = 20;
          wheelVelocity_Target[1] = 20;
          wheelVelocity_Target[2] = 20;
        }
        else if(count >= 100 && count < 150){
          wheelVelocity_Target[0] = 40;
          wheelVelocity_Target[1] = 40;
          wheelVelocity_Target[2] = 40;
        }
        else if(count >= 150 && count < 200){
          wheelVelocity_Target[0] = 60;
          wheelVelocity_Target[1] = 60;
          wheelVelocity_Target[2] = 60;
        }
        else if(count >= 200 && count < 250){
          wheelVelocity_Target[0] = 30;
          wheelVelocity_Target[1] = 30;
          wheelVelocity_Target[2] = 30;
        }
        else if(count >= 250 && count < 300){
          wheelVelocity_Target[0] = 0;
          wheelVelocity_Target[1] = 0;
          wheelVelocity_Target[2] = 0;
        }
        Serial.println(wheelVelocity_Real[0]);
        count++;
      }
      else if(count > 250 && count < 300){
        wheelVelocity_Target[0] = 0;
        wheelVelocity_Target[1] = 0;
        wheelVelocity_Target[2] = 0;
        Serial.println(wheelVelocity_Real[0]);
        count++;
      }
      else if(count >= 300){
        count = 0;
      }
      count3 = 0;
    }
    else count3++;*/

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
    // if(((motorPwm[0] || motorPwm[1] || motorPwm[2]) != 0) && count < 500){
    //   count++;
    //   count3++;
    //   if(count3 == 5){
    //     Serial.println(wheelVelocity_Real[0]);
    //     count3 = 0;
    //   }
    // }
    // if(count == 500) Serial.println();
    // if(count >= 500 && count < 600){
    //   bluetoothCommand = 's';
    //   count++;
    // }
    // if(count >= 600 && count < 1100){
    //   bluetoothCommand = '2';
    //   count++;
    //   count3++;
    //   if(count3 == 5){
    //     Serial.println(wheelVelocity_Real[1]);
    //     count3 = 0;
    //   }
    // }
    // if(count == 1100) Serial.println();
    // if(count >= 1100 && count < 1200){
    //   bluetoothCommand = 's';
    //   count++;
    // }
    // if(count >= 1200 && count < 1700){
    //   bluetoothCommand = '3';
    //   count++;
    //   count3++;
    //   if(count3 == 5){
    //     Serial.println(wheelVelocity_Real[2]);
    //     count3 = 0;
    //   }
    // }
    // if(count == 1700) Serial.println();
    // if(count >= 1700){
    //   bluetoothCommand = 's';
    //   count = 0;
    // }
    
    // Showing data
    // if(count>100)
    // {
    //   BLUETOOTH_SERIAL.print("x:"); BLUETOOTH_SERIAL.print(x_Real);
    //   BLUETOOTH_SERIAL.print(" y:"); BLUETOOTH_SERIAL.println(y_Real);
    //   BLUETOOTH_SERIAL.print("vx:"); BLUETOOTH_SERIAL.print(robotVelocityX_Real);
    //   BLUETOOTH_SERIAL.print(" vy:"); BLUETOOTH_SERIAL.println(robotVelocityY_Real); BLUETOOTH_SERIAL.println();

      // BLUETOOTH_SERIAL.print(" xcount:"); BLUETOOTH_SERIAL.print(x_count);
      // BLUETOOTH_SERIAL.print(" ycount:"); BLUETOOTH_SERIAL.println(y_count);

      // BLUETOOTH_SERIAL.print("v1: "); BLUETOOTH_SERIAL.print(wheelVelocity_Real_50[0]);
      // BLUETOOTH_SERIAL.print(" v2: "); BLUETOOTH_SERIAL.print(wheelVelocity_Real_50[1]);
      // BLUETOOTH_SERIAL.print(" v3: "); BLUETOOTH_SERIAL.println(wheelVelocity_Real_50[2]);

      // BLUETOOTH_SERIAL.print("v1target: "); BLUETOOTH_SERIAL.print(wheelVelocity_Target[0]);
      // BLUETOOTH_SERIAL.print(" v2target: "); BLUETOOTH_SERIAL.print(wheelVelocity_Target[1]);
      // BLUETOOTH_SERIAL.print(" v3target: "); BLUETOOTH_SERIAL.println(wheelVelocity_Target[2]);

      // BLUETOOTH_SERIAL.print("PWM1: "); BLUETOOTH_SERIAL.print(motorPwm[0]);
      // BLUETOOTH_SERIAL.print(" PWM2: "); BLUETOOTH_SERIAL.print(motorPwm[1]);
      // BLUETOOTH_SERIAL.print(" PWM3: "); BLUETOOTH_SERIAL.println(motorPwm[2]);

      // BLUETOOTH_SERIAL.print("enc1:"); BLUETOOTH_SERIAL.print(a);
      // BLUETOOTH_SERIAL.print(" enc2:"); BLUETOOTH_SERIAL.print(b);
      // BLUETOOTH_SERIAL.print(" enc3:"); BLUETOOTH_SERIAL.println(c);
    
      // Serial.print("v1target: "); Serial.print(wheelVelocity_Target[0]);
      // Serial.print("  v2target: "); Serial.print(wheelVelocity_Target[1]);
      // Serial.print("  v3target: "); Serial.println(wheelVelocity_Target[2]);

      // Serial.print("      v1: "); Serial.print(wheelVelocity_Real[0]);
      // Serial.print("         v2: "); Serial.print(wheelVelocity_Real[1]);
      // Serial.print("        v3: "); Serial.println(wheelVelocity_Real[2]);

      // Serial.print("    PWM1: "); Serial.print(motorPwm[0]);
      // Serial.print("       PWM2: "); Serial.print(motorPwm[1]);
      // Serial.print("      PWM3: "); Serial.println(motorPwm[2]);

      // Serial.print(ax); Serial.print("   ");
      // Serial.print(ay);  Serial.print("   ");
      // Serial.println(az);
    //   Serial.println();
    //   count=0;
    // }
    
    // signed long encoder1=enc1.read();
    // Serial.println(encoder1);
    // Serial.println(theta360);
    // Serial.println(wheelVelocity_Target[0]);
    // Serial.print(digitalRead(IR_FRONT_PIN));
    // Serial.print(digitalRead(IR_BACK_PIN));
    // Serial.print(digitalRead(IR_RIGHT_PIN));
    // Serial.println(digitalRead(IR_LEFT_PIN));
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
    // wheelVelocity_Target[0] = 50.0;
    // wheelVelocity_Target[1] = 50.0;
    // wheelVelocity_Target[2] = 50.0;
    manualOmega = MANUAL_OM;
  }
  else if(bluetoothCommand == 'q')
  {
    // wheelVelocity_Target[0] = -50.0;
    // wheelVelocity_Target[1] = -50.0;
    // wheelVelocity_Target[2] = -50.0;
    manualOmega = -MANUAL_OM;
  }
  else if(bluetoothCommand == 's'){
    manualVY = 0;
    manualVX = 0;
    manualOmega = 0;
    wheelVelocity_Target[0] = 0.0;
    wheelVelocity_Target[1] = 0.0;
    wheelVelocity_Target[2] = 0.0;
    motorPwm[0] = 0;
    motorPwm[1] = 0;
    motorPwm[2] = 0;
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
    wheelVelocity_Target[0] = 0;
    wheelVelocity_Target[1] = 0;
    wheelVelocity_Target[2] = 0;
    mode = INDEPENDENT_WHEEL;
  }
  else if(bluetoothCommand == '1'){
    // wheelVelocity1_Target = 50.0;
    // wheelVelocity2_Target = 0.0;
    // wheelVelocity3_Target = 0.0;
    motorPwm[0] = 150.0;
    motorPwm[1] = 0.0;
    motorPwm[2] = 0.0;
  }
  else if(bluetoothCommand == '2'){
    // wheelVelocity1_Target = 0;
    // wheelVelocity2_Target = 50;
    // wheelVelocity3_Target = 0;
    motorPwm[0] = 0.0;
    motorPwm[1] = 150.0;
    motorPwm[2] = 0.0;
  }
  else if(bluetoothCommand == '3'){
    // wheelVelocity1_Target = 0;
    // wheelVelocity2_Target = 0;
    // wheelVelocity3_Target = 50;
    motorPwm[0] = 0.0;
    motorPwm[1] = 0.0;
    motorPwm[2] = 150.0;
  }
  else if(bluetoothCommand == '4'){
    wheelVelocity_Target[0] = -40;
    wheelVelocity_Target[1] = 0;
    wheelVelocity_Target[2] = 0;
  }
  else if(bluetoothCommand == '5'){
    wheelVelocity_Target[0] = 0;
    wheelVelocity_Target[1] = -40;
    wheelVelocity_Target[2] = 0;
  }
  else if(bluetoothCommand == '6'){
    wheelVelocity_Target[0] = 0;
    wheelVelocity_Target[1] = 0;
    wheelVelocity_Target[2] = -40;
  }
  // else if(bluetoothCommand == 'u'){
  //   motorPwm1 = 150;
  //   motorPwm2 = 150;
  //   motorPwm3 = 150;
  // }
  bluetoothCommand = 'z';

  if(mode == AUTOMATIC){
    moveRobotGlobal(robotVelocityX_Target, robotVelocityY_Target, robotOmega_Target);
  }
  if(mode == MANUAL){
    moveRobotLocal(manualVX, manualVY, manualOmega);
  }
  else if(mode == INDEPENDENT_WHEEL){
    velocityToPwm();
    robotMotorWrite(motorPwm[0], motorPwm[1], motorPwm[2]);
  }
  else if(mode == STEP_INPUT){
    robotMotorWrite(motorPwm[0], motorPwm[1], motorPwm[2]);
  }
}
