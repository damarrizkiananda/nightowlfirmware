
#define USE_EXTRA_PROTOCOL

int debugCounter = 0;

//** serial comm
void encode_data(){
  // this is where to encode the data received, like the receive event in i2c
  // where serial_buffer[0] is the data type, and the rest is data

  if(debugCounter>10)
  {
    char text[100];
    sprintf(text, "buff0: %d    buff1:%d     buff2:%d    buff3:%d", serial_buffer[0], serial_buffer[1], serial_buffer[2], serial_buffer[3]);
    BLUETOOTH_SERIAL.println(text);
    debugCounter = 0;
  }
 
  debugCounter++;

  uint8_t robotVelocityX_Received = serial_buffer[0];
  uint8_t robotVelocityY_Received = serial_buffer[1];
  uint8_t robotOmega_Received = serial_buffer[2];
  uint8_t robotVelocityX_Parity = serial_buffer[3] & 0x01;
  uint8_t robotVelocityY_Parity = (serial_buffer[3] >> 1) & 0x01;
  uint8_t robotOmega_Parity = (serial_buffer[3] >> 2) & 0x01;
  //arrived = (serial_buffer[3] >> 3) & 0x01;

  if(robotVelocityX_Parity == 1) robotVelocityX_Target = robotVelocityX_Received*(-1);
  else robotVelocityX_Target = robotVelocityX_Received;
  if(robotVelocityY_Parity == 1) robotVelocityY_Target = robotVelocityY_Received*(-1);
  else robotVelocityY_Target = robotVelocityY_Received;
  if(robotOmega_Parity  == 1) robotOmega_Target  = robotOmega_Received*(-1);
  else robotOmega_Target = robotOmega_Received;
  
}

void serial_receive(){
  while(Serial.available() > 0){
    uint8_t rc = Serial.read();
    
    if(serial_receiving){
      if(rc != SERIAL_TAIL){
        serial_buffer[serial_ndx] = rc;
        serial_ndx++;
      }
      else{
        if(serial_ndx<=3)
        {
          serial_buffer[serial_ndx] = rc;
          serial_ndx++;
        }
        else
        {
          serial_receiving = false;
          serial_new_data = true;
          serial_recv_len = serial_ndx; serial_ndx = 0;
        }
      }
    }
    else if (rc == SERIAL_HEAD) {
      serial_receiving = true; 
      serial_new_data = false;
    }
  }
  
  if(serial_new_data) {
    encode_data();
    serial_new_data = false;
  }
}

void serial_send_packets(uint8_t* data, byte len){
  Serial.write(SERIAL_HEAD);
  for(int i = 0; i < len; i++){
    #ifdef USE_EXTRA_PROTOCOL
    if(data[i]==SERIAL_HEAD||data[i]==SERIAL_TAIL)data[i]++;
    #endif
    Serial.write(data[i]);
  }
  Serial.write(SERIAL_TAIL);
}
void serial_send_packets(int16_t* data, byte len){
  Serial.write(SERIAL_HEAD);
  for(int i = 0; i < len; i++){
    #ifdef USE_EXTRA_PROTOCOL
    if(data[i]==SERIAL_HEAD||data[i]==SERIAL_TAIL)data[i]++;
    #endif
    serial_send(data[i]);
  }
  Serial.write(SERIAL_TAIL);
}
void serial_send_packets(int32_t* data, byte len){
  Serial.write(SERIAL_HEAD);
  for(int i = 0; i < len; i++){
    #ifdef USE_EXTRA_PROTOCOL
    if(data[i]==SERIAL_HEAD||data[i]==SERIAL_TAIL)data[i]++;
    #endif
    serial_send(data[i]+1);
  }
  Serial.write(SERIAL_TAIL);
}

void serial_send( int16_t data){
  uint8_t buff[2];
  memcpy(buff, &data, sizeof(uint16_t));
  Serial.write(buff, sizeof(int16_t));
}
void serial_send( int32_t data){
  uint8_t buff[4];
  memcpy(buff, &data, sizeof(uint32_t));
  Serial.write(buff, sizeof(int32_t));
}

void send_to_laptop()
{
  uint8_t robotVelocityX_Parity, robotVelocityY_Parity, robotOmega_Parity, 
          x_Parity, y_Parity, enc1_Parity, enc2_Parity, enc3_Parity,
          parity;
  uint16_t thetaTimes100;

  /* Without parity */
  uint16_t absRobotVelocityX_Real, absRobotVelocityY_Real, absRobotOmega_Real;
  uint16_t absX, absY;

  /* Copy */
  int16_t x_Copy, y_Copy, robotVelocityX_Copy, robotVelocityY_Copy, robotOmega_Copy, enc1_Copy, enc2_Copy, enc3_Copy;

  noInterrupts();
  x_Copy = x_Real;
  y_Copy = y_Real;
  robotVelocityX_Copy = robotVelocityX_Real;
  robotVelocityY_Copy = robotVelocityY_Real;
  robotOmega_Copy  = robotOmega_Real;
  enc1_Copy = encoderPulseDif1;
  enc2_Copy = encoderPulseDif2;
  enc3_Copy = encoderPulseDif3;
  interrupts();


  if(x_Copy<0)x_Parity = 1;
  else x_Parity = 0;
  if(y_Copy<0)y_Parity = 1;
  else y_Parity = 0;
  if(robotVelocityX_Copy<0)robotVelocityX_Parity = 1;
  else robotVelocityX_Parity = 0;
  if(robotVelocityY_Copy<0)robotVelocityY_Parity = 1;
  else robotVelocityY_Parity = 0;
  if(robotOmega_Copy<0)robotOmega_Parity = 1;
  else robotOmega_Parity = 0;
  if(enc1_Copy<0)enc1_Parity = 1;
  else enc1_Parity = 0;
  if(enc2_Copy<0)enc2_Parity = 1;
  else enc2_Parity = 0;
  if(enc3_Copy<0)enc3_Parity = 1;
  else enc3_Parity = 0;
  

  parity = x_Parity | (y_Parity<<1) | (robotVelocityX_Parity<<2) | (robotVelocityY_Parity<<3) | (robotOmega_Parity<<4) | (enc1_Parity<<5) | (enc2_Parity<<6) | (enc3_Parity<<7) ;
  
  absX = (uint16_t)(abs(x_Copy));
  absY = (uint16_t)(abs(y_Copy));
  absRobotVelocityX_Real = (uint16_t)(abs(robotVelocityX_Copy));
  absRobotVelocityY_Real = (uint16_t)(abs(robotVelocityY_Copy));
  absRobotOmega_Real = (uint16_t)(abs(robotOmega_Copy));
  uint16_t absEnc1 = (uint16_t)(abs(enc1_Copy));
  uint16_t absEnc2 = (uint16_t)(abs(enc2_Copy));
  uint16_t absEnc3 = (uint16_t)(abs(enc3_Copy));

  thetaTimes100 = (uint16_t)(theta360*100);
  
  uint8_t data[20] = {(uint8_t)(absEnc1>>8), (uint8_t)(absEnc1&0x00FF), 
                      (uint8_t)(absEnc2>>8), (uint8_t)(absEnc2&0x00FF), 
                      (uint8_t)(absEnc3>>8), (uint8_t)(absEnc3&0x00FF),
                      (uint8_t)(absRobotVelocityX_Real>>8), (uint8_t)(absRobotVelocityX_Real&0x00FF),
                      (uint8_t)(absRobotVelocityY_Real>>8), (uint8_t)(absRobotVelocityY_Real&0x00FF), 
                      (uint8_t)(absRobotOmega_Real>>8), (uint8_t)(absRobotOmega_Real&0x00FF), 
                      (uint8_t)(absX>>8), (uint8_t)(absX&0x00FF), 
                      (uint8_t)(absY>>8), (uint8_t)(absY&0x00FF),
                      (uint8_t)(thetaTimes100>>8), (uint8_t)(thetaTimes100&0x00FF),
                      (uint8_t)parity, 
                      (uint8_t)IR_READ };
  // uint8_t data[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  serial_send_packets(data, 20);

  sendDataPlease = false;
}

