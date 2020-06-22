
#define USE_EXTRA_PROTOCOL

//** serial comm
void encode_data(){
  // this is where to encode the data received, like the receive event in i2c
  // where serial_buffer[0] is the data type, and the rest is data
  uint8_t robotSpeedX_Parity, robotSpeedY_Parity, robotOmega_Parity;

  robotSpeedX_Target = serial_buffer[0];
  robotSpeedY_Target = serial_buffer[1];
  robotOmega_Target = serial_buffer[2];
  robotSpeedX_Parity = serial_buffer[3] & 0x01;
  robotSpeedY_Parity = (serial_buffer[3] >> 1) & 0x01;
  robotOmega_Parity = (serial_buffer[3] >> 2) & 0x01;
  arrived = (serial_buffer[3] >> 3) & 0x01;

  if(robotSpeedX_Parity == 1) robotSpeedX_Target = robotSpeedX_Target*(-1);
  if(robotSpeedY_Parity == 1) robotSpeedY_Target = robotSpeedY_Target*(-1);
  if(robotOmega_Parity  == 1) robotOmega_Target  = robotOmega_Target*(-1);
  
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
        serial_receiving = false;
        serial_new_data = true;
        serial_recv_len = serial_ndx; serial_ndx = 0;
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
    Serial.write(data[i]+1);
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

int counter;
bool ledState;

void send_to_laptop()
{
  uint8_t robotSpeedX_Parity, robotSpeedY_Parity, robotOmega_Parity, x_Parity, y_Parity, parity;
  uint16_t yaw100;

  /* Without parity */
  uint16_t absRobotSpeedX_Real, absRobotSpeedY_Real, absRobotOmega_Real;
  uint16_t absX, absY;

  /* Copy */
  uint16_t x_Copy, y_Copy, robotSpeedX_Copy, robotSpeedY_Copy, robotOmega_Copy;

  noInterrupts();
  x_Copy = x;
  y_Copy = y;
  robotSpeedX_Copy = robotSpeedX_Real;
  robotSpeedY_Copy = robotSpeedY_Real;
  robotOmega_Copy  = robotOmega_Real;
  interrupts();


  if(x_Copy<0)x_Parity = 1;
  else x_Parity = 0;
  if(y_Copy<0)y_Parity = 1;
  else y_Parity = 0;
  if(robotSpeedX_Copy<0)robotSpeedX_Parity = 1;
  else robotSpeedX_Parity = 0;
  if(robotSpeedY_Copy<0)robotSpeedY_Parity = 1;
  else robotSpeedY_Parity = 0;
  if(robotOmega_Copy<0)robotOmega_Parity = 1;
  else robotOmega_Parity = 0;


  parity = x_Parity | (y_Parity<<1) | (robotSpeedX_Parity<<2) | (robotSpeedY_Parity<<3) | (robotOmega_Parity<<4);
  
  absX = (uint16_t)(abs(x_Copy));
  absY = (uint16_t)(abs(y_Copy));
  absRobotSpeedX_Real = (uint16_t)(abs(robotSpeedX_Copy));
  absRobotSpeedY_Real = (uint16_t)(abs(robotSpeedY_Copy));
  absRobotOmega_Real = (uint16_t)(abs(robotOmega_Copy));

  yaw100 = (uint16_t)(yaw*100);
  
  uint8_t data[14] = {(uint8_t)(absX>>8), (uint8_t)(absX&0x00FF), (uint8_t)(absY>>8), (uint8_t)(absY&0x00FF), (uint8_t)(absRobotSpeedX_Real>>8), (uint8_t)(absRobotSpeedX_Real&0x00FF), 
                      (uint8_t)(absRobotSpeedY_Real>>8), (uint8_t)(absRobotSpeedY_Real&0x00FF), (uint8_t)(absRobotOmega_Real>>8), (uint8_t)(absRobotOmega_Real&0x00FF), parity, 
                      (uint8_t)(yaw100>>8), (uint8_t)(yaw100&0x00FF), IR_READ|(destination<<4) };
  serial_send_packets(data, 14);

  counter++;
  if(counter>10)
  {
    ledState = !ledState;
    counter = 0;
  }
  digitalWrite(13, ledState);

  sendDataPlease = false;
}

