#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <FlexiTimer2.h>
#include <Encoder.h>

Encoder enc1(10, 9);
Encoder enc3(11, 12);
Encoder enc2(15, 14);

/* Encoder Pulse Difference*/
int encoderPulseDif1, encoderPulseDif2, encoderPulseDif3;

/* Linear Wheel Velocity */
double wheelVelocity1_Real, wheelVelocity2_Real, wheelVelocity3_Real;
double wheelVelocity1_Target, wheelVelocity2_Target, wheelVelocity3_Target;

/* Robot Position */
double x_Real, y_Real, yaw_Real;

/* Robot Velocity */
double robotVelocityX_Target, robotVelocityY_Target, robotOmega_Target;
double robotVelocityX_Real, robotVelocityY_Real, robotOmega_Real;

#define TIMER_INTERRUPT_PERIOD 50

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29,&Wire);
sensors_event_t orientationData;

#define SERIAL_HEAD   '{'
#define SERIAL_TAIL   '}'
bool serial_receiving = false;
bool serial_new_data = false;
byte serial_ndx = 0, serial_recv_len = 0;
uint8_t serial_buffer[16];

#define BLUETOOTH_SERIAL Serial5
#define BLUETOOTH_HEAD   '{'
#define BLUETOOTH_TAIL   '}'
bool bluetooth_receiving = false;
bool bluetooth_new_data = false;
byte bluetooth_ndx = 0, bluetooth_recv_len = 0;
uint8_t bluetooth_buffer[16];

unsigned long now = 0;

#define MOTOR_1_A_PIN 3 
#define MOTOR_1_B_PIN 4
#define MOTOR_2_A_PIN 5
#define MOTOR_2_B_PIN 6
#define MOTOR_3_A_PIN 7
#define MOTOR_3_B_PIN 8

#define IR_1_PIN 25
#define IR_2_PIN 27
#define IR_3_PIN 29
#define IR_4_PIN 31

#define IR_READ (!digitalRead(IR_1_PIN)) | ((!digitalRead(IR_2_PIN))<<1) | ((!digitalRead(IR_3_PIN))<<2) | ((!digitalRead(IR_4_PIN))<<3)

bool sendDataPlease;
bool velocityAndPositionUpdated;

#define WAITING_FOR_NEW_DESTINATION 0
#define MOVING 1

uint8_t robotState = MOVING;
bool arrived;

bool newDestination;
uint8_t destination; 
#define HONEYWELL 1
#define FO        2
#define Toilet    3

/* Motor Velocity Control */ 
double Setpoint1, Input1, Output1;
double Kp1=0.5, Ki1=1.5, Kd1=0.001;
PID motorPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

double Setpoint2, Input2, Output2;
double Kp2=0.5, Ki2=1.5, Kd2=0.001;
PID motorPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

double Setpoint3, Input3, Output3;
double Kp3=0.5, Ki3=1.5, Kd3=0.001;
PID motorPID3(&Input3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);

/* Robot Velocity Control */
double SetpointVX, InputVX, OutputVX;
double KpVX=8, KiVX=1, KdVX=0.001;
PID velocityXPID(&InputVX, &OutputVX, &SetpointVX, KpVX, KiVX, KdVX, DIRECT);

double SetpointVY, InputVY, OutputVY;
double KpVY=8, KiVY=1, KdVY=0.001;
PID velocityYPID(&InputVY, &OutputVY, &SetpointVY, KpVY, KiVY, KdVY, DIRECT);

double SetpointOmega, InputOmega, OutputOmega;
double KpOmega=8, KiOmega=1, KdOmega=0.001;
PID omegaPID(&InputOmega, &OutputOmega, &SetpointOmega, KpOmega, KiOmega, KdOmega, DIRECT);


/* Robot Position Control */
double SetpointX, InputX, OutputX;
double KpX=20, KiX=1, KdX=0.001;
PID positionPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);

double SetpointY, InputY, OutputY;
double KpY=20, KiY=1, KdY=0.001;
PID positionPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

double SetpointTheta, InputTheta, OutputTheta;
double KpTheta=20, KiTheta=1.5, KdTheta=0.001;
PID positionPIDTheta(&InputTheta, &OutputTheta, &SetpointTheta, KpTheta, KiTheta, KdTheta, DIRECT);


double SetpointR, InputR, OutputR;
double KpR=20, KiR=1, KdR=0.001;
PID positionPIDR(&InputR, &OutputR, &SetpointR, KpR, KiR, KdR, DIRECT);

/* Conversion Constants */
#define TO_DEG    57.29577951308
#define TO_RAD    0.01745329252
#define PI        3.1415926535
#define TWO_PI    6.2831853071

/*     Dist/pulse = 
 *  Circumference/PPR 
 *       in cm
 */
#define CIRCUMFERENCE 31.4159265
#define PPR 1024
#define DIST_PER_PULSE 0.03067961

void setup() 
{
  /* Initialise the BNO055 */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  pinMode(IR_1_PIN, INPUT);
  pinMode(IR_2_PIN, INPUT);
  pinMode(IR_3_PIN, INPUT);
  pinMode(IR_4_PIN, INPUT);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);

  BLUETOOTH_SERIAL.begin(9600);
  BLUETOOTH_SERIAL.setTimeout(100);
  
  Serial.begin(115200);
  Serial.setTimeout(100);

  FlexiTimer2::set(TIMER_INTERRUPT_PERIOD, 1.0/1000, updateVelocityAndPosition); // call every 100 1ms "ticks"
  // FlexiTimer2::set(500, flash); // MsTimer2 style is also supported
  FlexiTimer2::start();

  motorPID1.SetMode(AUTOMATIC); motorPID1.SetOutputLimits(-255,255); motorPID1.SetSampleTime(50);
  motorPID2.SetMode(AUTOMATIC); motorPID2.SetOutputLimits(-255,255); motorPID1.SetSampleTime(50);
  motorPID3.SetMode(AUTOMATIC); motorPID3.SetOutputLimits(-255,255); motorPID1.SetSampleTime(50);

  positionPIDX.SetMode(AUTOMATIC); positionPIDX.SetOutputLimits(-250,250); positionPIDX.SetSampleTime(50);
  positionPIDY.SetMode(AUTOMATIC); positionPIDY.SetOutputLimits(-250,250); positionPIDY.SetSampleTime(50);
  positionPIDTheta.SetMode(AUTOMATIC); positionPIDTheta.SetOutputLimits(-250,250); positionPIDTheta.SetSampleTime(50);

  getYawDeg();
}


void loop()  
{
  // Serial.println(IR_READ,BIN);
  // delay(500);
  // NightOwlMain();
  // vibeCheck();
  // bluetoothCheck();
  // timerCheck();
  // odometryCheck();
  // inverseCheck();
  mainMain();
  // delay(10);
  // bluetooth_receive();
}
