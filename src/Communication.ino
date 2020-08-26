
// #define USE_EXTRA_PROTOCOL

int debugCounter = 0;

//** serial comm
void encode_data(){
  // this is where to encode the data received, like the receive event in i2c
  // where serial_buffer[0] is the data type, and the rest is data

  // if(debugCounter>10)
  // {
  //   char text[100];
  //   sprintf(text, "buff0: %d    buff1:%d     buff2:%d    buff3:%d", serial_buffer[0], serial_buffer[1], serial_buffer[2], serial_buffer[3]);
  //   BLUETOOTH_SERIAL.println(text);
  //   debugCounter = 0;
  // }
 
  // debugCounter++;

  uint8_t robotVelocityX_Received = serial_buffer[0];
  uint8_t robotVelocityY_Received = serial_buffer[1];
  uint8_t robotOmega_Received = serial_buffer[2];
  uint8_t robotVelocityX_Parity = serial_buffer[3] & 0x01;
  uint8_t robotVelocityY_Parity = (serial_buffer[3] >> 1) & 0x01;
  uint8_t robotOmega_Parity = (serial_buffer[3] >> 2) & 0x01;
  //arrived = (serial_buffer[3] >> 3) & 0x01;

  if(robotVelocityX_Parity == 0) robotVelocityX_Target = robotVelocityX_Received*(-1);
  else robotVelocityX_Target = robotVelocityX_Received;
  if(robotVelocityY_Parity == 0) robotVelocityY_Target = robotVelocityY_Received*(-1);
  else robotVelocityY_Target = robotVelocityY_Received;
  if(robotOmega_Parity  == 0) robotOmega_Target  = robotOmega_Received*(-1);
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
    serial_send(data[i]);
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
  uint8_t x_Parity, y_Parity,
          vx_Parity, vy_Parity, omega100_Parity,
          parity;

  noInterrupts();
  int16_t x_Copy = x_Real;
  int16_t y_Copy = y_Real;
  uint16_t theta100_Copy = (uint16_t)(theta360*100); 
  int16_t vx_Copy = robotVelocityX_Real;
  int16_t vy_Copy = robotVelocityY_Real;
  int16_t omega100_Copy = robotOmega_Real;
  interrupts();

  /* Calculate parity */
  if(x_Copy<0)x_Parity = 0;
  else x_Parity = 1;
  if(y_Copy<0)y_Parity = 0;
  else y_Parity = 1;
  if(vx_Copy<0)vx_Parity = 0;
  else vx_Parity = 1;
  if(vy_Copy<0)vy_Parity = 0;
  else vy_Parity = 1;
  if(omega100_Copy<0)omega100_Parity = 0;
  else omega100_Parity = 1;

  parity = (omega100_Parity<<3) | (vy_Parity<<4) | (vx_Parity<<5) | (y_Parity<<6) | (x_Parity<<7);
  
  /* Calculate absolute value */
  uint16_t absX        = (uint16_t)(abs(x_Copy));
  uint16_t absY        = (uint16_t)(abs(y_Copy));
  uint16_t absVX       = (uint16_t)(abs(vx_Copy));
  uint16_t absVY       = (uint16_t)(abs(vy_Copy));
  uint16_t absOmega100 = (uint16_t)(abs(omega100_Copy));
  
  uint8_t data[14] = {(uint8_t)(absX>>8), (uint8_t)(absX&0x00FF), 
                      (uint8_t)(absY>>8), (uint8_t)(absY&0x00FF), 
                      (uint8_t)(theta100_Copy>>8), (uint8_t)(theta100_Copy&0x00FF),
                      (uint8_t)(absVX>>8), (uint8_t)(absVX&0x00FF),
                      (uint8_t)(absVY>>8), (uint8_t)(absVY&0x00FF),
                      (uint8_t)(absOmega100>>8), (uint8_t)(absOmega100&0x00FF),
                      (uint8_t)parity, IR_READ};
  // uint8_t data[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  serial_send_packets(data, 14);

  sendDataPlease = false;
}

