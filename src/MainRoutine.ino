
#define AUTOMATIC 0
#define MANUAL    1
#define MANUAL_V  60
#define MANUAL_OM 100
char bluetoothCommand = 'z';
int manualVX = 0, manualVY = 0, manualOmega = 0, mode = AUTOMATIC;
int count = 0;

void NightOwlLoop()
{
  serial_receive();
  if(velocityAndPositionUpdated == true)
  {
    send_to_laptop();  

    count++;
    if(count>100)
    {
      noInterrupts();
      int xc = x_Real;
      int yc = y_Real;
      interrupts();

      BLUETOOTH_SERIAL.print("x:"); BLUETOOTH_SERIAL.print(xc);
      BLUETOOTH_SERIAL.print(" y:"); BLUETOOTH_SERIAL.println(yc);

      count = 0;
    }

    velocityAndPositionUpdated = false;
  }

  if(BLUETOOTH_SERIAL.available())
  {
    bluetoothCommand = BLUETOOTH_SERIAL.read();
  }

  if(bluetoothCommand == 'f')
  {
    manualVX = MANUAL_V;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 'b')
  {
    manualVX = -MANUAL_V;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 'l')
  {
    manualVY = MANUAL_V;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 'r')
  {
    manualVY = -MANUAL_V;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 'p')
  {
    manualOmega = MANUAL_OM;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 'q')
  {
    manualOmega = -MANUAL_OM;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 's')
  {
    manualVY = 0;
    manualVX = 0;
    manualOmega = 0;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 'a')
  {
    mode = AUTOMATIC;
    bluetoothCommand = 'z';
  }
  else if(bluetoothCommand == 'm')
  {
    manualVY = 0;
    manualVX = 0;
    manualOmega = 0;
    mode = MANUAL;
    bluetoothCommand = 'z';
  }

  if(mode == MANUAL) moveRobotLocal(manualVX, manualVY, manualOmega);
  else moveRobotGlobal(robotVelocityX_Target, robotVelocityY_Target, robotOmega_Target);
  
}
