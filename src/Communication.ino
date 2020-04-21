
#define USE_EXTRA_PROTOCOL

//** serial comm
void encode_data(){
  // this is where to encode the data received, like the receive event in i2c
  // where serial_buffer[0] is the data type, and the rest is data
  
  uPwm1 = serial_buffer[0];
  uPwm2 = serial_buffer[1];
  uPwm3 = serial_buffer[2];
  pwm1Parity = serial_buffer[3] & 0x01;
  pwm2Parity = (serial_buffer[3] >> 1) & 0x01;
  pwm3Parity = (serial_buffer[3] >> 2) & 0x01;
  arrived = (serial_buffer[3] >> 3) & 0x01;

  if(pwm1Parity) pwm1 = uPwm1*(-1);
  else pwm1 = uPwm1;
  if(pwm2Parity) pwm2 = uPwm2*(-1);
  else pwm2 = uPwm2;
  if(pwm3Parity) pwm3 = uPwm3*(-1);
  else pwm3 = uPwm3;
  
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
  uint8_t parity1, parity2, parity3, parity;
  uint16_t yaw100;

  /* Speed without parity */
  uint16_t SPEED1, SPEED2, SPEED3;
  
  noInterrupts();
  speed1Copy = speed1;
  speed2Copy = speed2;
  speed3Copy = speed3;
  interrupts();

  if(speed1Copy<0)parity1 = 1;
  else parity1 = 0;
  if(speed2Copy<0)parity2 = 1;
  else parity2 = 0;
  if(speed3Copy<0)parity3 = 1;
  else parity3 = 0;

  parity = parity1 | (parity2<<1) | (parity3<<2);
  
  SPEED1 = (uint16_t)(abs(speed1Copy));
  SPEED2 = (uint16_t)(abs(speed2Copy));
  SPEED3 = (uint16_t)(abs(speed3Copy));

  yaw100 = (uint16_t)(yaw*100);
  
  uint8_t data[10] = {(uint8_t)(SPEED1>>8), (uint8_t)(SPEED1&0x00FF), (uint8_t)(SPEED2>>8), (uint8_t)(SPEED2&0x00FF), (uint8_t)(SPEED3>>8), (uint8_t)(SPEED3&0x00FF), parity, 
                    (uint8_t)(yaw100>>8), (uint8_t)(yaw100&0x00FF), IR_READ|(destination<<4) };
  serial_send_packets(data, 10);

  counter++;
  if(counter>10)
  {
    ledState = !ledState;
    counter = 0;
  }
  digitalWrite(13, ledState);

  sendDataPlease = false;
}

