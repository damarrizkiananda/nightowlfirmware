
#define USE_EXTRA_PROTOCOL

//** bluetooth comm
void bluetooth_encode_data(){
  // this is where to encode the data received, like the receive event in i2c
  // where bluetooth_buffer[0] is the data type, and the rest is data
  
  destination = bluetooth_buffer[0];
  newDestination = true;
}

void bluetooth_receive(){
  while(BLUETOOTH_SERIAL.available() > 0){
    uint8_t rc = BLUETOOTH_SERIAL.read();
    
    if(bluetooth_receiving){
      if(rc != BLUETOOTH_TAIL){
        bluetooth_buffer[bluetooth_ndx] = rc;
        bluetooth_ndx++;
      }
      else{
        bluetooth_receiving = false;
        bluetooth_new_data = true;
        bluetooth_recv_len = bluetooth_ndx; bluetooth_ndx = 0;
      }
    }
    else if (rc == BLUETOOTH_HEAD) {
      bluetooth_receiving = true; 
      bluetooth_new_data = false;
    }
  }
  
  if(bluetooth_new_data) {
    bluetooth_encode_data();
    bluetooth_new_data = false;
  }
}

void bluetooth_send_packets(uint8_t* data, byte len){
  BLUETOOTH_SERIAL.write(BLUETOOTH_HEAD);
  for(int i = 0; i < len; i++){
    #ifdef USE_EXTRA_PROTOCOL
    if(data[i]==BLUETOOTH_HEAD||data[i]==BLUETOOTH_TAIL)data[i]++;
    #endif
    BLUETOOTH_SERIAL.write(data[i]);
  }
  BLUETOOTH_SERIAL.write(BLUETOOTH_TAIL);
}
void bluetooth_send_packets(int16_t* data, byte len){
  BLUETOOTH_SERIAL.write(BLUETOOTH_HEAD);
  for(int i = 0; i < len; i++){
    #ifdef USE_EXTRA_PROTOCOL
    if(data[i]==BLUETOOTH_HEAD||data[i]==BLUETOOTH_TAIL)data[i]++;
    #endif
    bluetooth_send(data[i]);
  }
  BLUETOOTH_SERIAL.write(BLUETOOTH_TAIL);
}
void bluetooth_send_packets(int32_t* data, byte len){
  BLUETOOTH_SERIAL.write(BLUETOOTH_HEAD);
  for(int i = 0; i < len; i++){
    #ifdef USE_EXTRA_PROTOCOL
    if(data[i]==BLUETOOTH_HEAD||data[i]==BLUETOOTH_TAIL)data[i]++;
    #endif
    bluetooth_send(data[i]);
  }
  BLUETOOTH_SERIAL.write(BLUETOOTH_TAIL);
}

void bluetooth_send( int16_t data){
  uint8_t buff[2];
  memcpy(buff, &data, sizeof(uint16_t));
  BLUETOOTH_SERIAL.write(buff, sizeof(int16_t));
}
void bluetooth_send( int32_t data){
  uint8_t buff[4];
  memcpy(buff, &data, sizeof(uint32_t));
  BLUETOOTH_SERIAL.write(buff, sizeof(int32_t));
}

void send_to_tablet()
{
  
}

