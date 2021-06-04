#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <FlexiTimer2.h>
#include <Encoder.h>

Encoder enc1(10, 9);
Encoder enc3(11, 12);
Encoder enc2(15, 14);

/* Encoder Count Difference*/
int32_t enc1_read, enc2_read, enc3_read;
signed long encoderCountDif1, encoderCountDif2, encoderCountDif3;

/* Robot Position */
double x_Real, y_Real, theta_Real, theta_BNO055, theta_Odo, theta360;

/* Robot Velocity */
double robotVelocityX_Target = 0, robotVelocityY_Target = 0, robotOmega_Target = 0;
double robotVelocityX_Real, robotVelocityY_Real, robotOmega_Real;

#define TIMER_INTERRUPT_PERIOD 50.0 // 20 Hz

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 25;
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29,&Wire);
sensors_event_t orientationData;

#define SERIAL_HEAD   '{'
#define SERIAL_TAIL   '}'
bool serial_receiving = false;
bool serial_new_data = false;
byte serial_ndx = 0, serial_recv_len = 0;
uint8_t serial_buffer[16] = {0};

#define BLUETOOTH_SERIAL Serial5

unsigned long now = 0;

#define MOTOR_1_A_PIN 3 
#define MOTOR_1_B_PIN 4
#define MOTOR_2_A_PIN 5
#define MOTOR_2_B_PIN 6
#define MOTOR_3_A_PIN 7
#define MOTOR_3_B_PIN 8

double motorPwm1 = 0.0, motorPwm2=0.0, motorPwm3=0.0;

#define IR_BACK_PIN  25
#define IR_RIGHT_PIN 27 
#define IR_LEFT_PIN  29
#define IR_FRONT_PIN 31 

uint8_t IR_READ = ((!digitalRead(IR_BACK_PIN)) | ((!digitalRead(IR_FRONT_PIN))<<1) | ((!digitalRead(IR_LEFT_PIN))<<2) | ((!digitalRead(IR_RIGHT_PIN))<<3));

bool sendDataPlease;
bool velocityAndPositionUpdated;

/* Conversion Constants */
#define TO_DEG    57.29577951308
#define TO_RAD    0.01745329252

/*     Dist/count = 
 *  Circumference/CPR 
 *       in cm
 */
#define CIRCUMFERENCE 31.4159265
#define CPR 4096.0
#define DIST_PER_COUNT 0.0076699025 //  from calculating
// #define DIST_PER_COUNT 0.007428623 // from calibrating

double wheelVelocity1_Real = 0.0, wheelVelocity1_Target = 0.0, wheelVelocity1_Error = 0.0, wheelVelocity1_Prev_Error = 0.0, wheelVelocity1_Error_Sum = 0.0, wheelVelocity1_Prev_Target = 0.0, wheelAcc1 = 0.0;
double wheelVelocity2_Real = 0.0, wheelVelocity2_Target = 0.0, wheelVelocity2_Error = 0.0, wheelVelocity2_Prev_Error = 0.0, wheelVelocity2_Error_Sum = 0.0, wheelVelocity2_Prev_Target = 0.0, wheelAcc2 = 0.0;
double wheelVelocity3_Real = 0.0, wheelVelocity3_Target = 0.0, wheelVelocity3_Error = 0.0, wheelVelocity3_Prev_Error = 0.0, wheelVelocity3_Error_Sum = 0.0, wheelVelocity3_Prev_Target = 0.0, wheelAcc3 = 0.0;


/* Max robot speed in cm/s */
#define MAX_ROBOT_SPEED 60

/* Max robot omega in deg/s */
#define MAX_ROBOT_OMEGA 60

void setup() 
{
  /* Initialise the BNO055 */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  pinMode(IR_BACK_PIN,  INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(IR_LEFT_PIN,  INPUT);
  pinMode(IR_FRONT_PIN, INPUT);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);

  BLUETOOTH_SERIAL.begin(9600);
  BLUETOOTH_SERIAL.setTimeout(100);
  
  Serial.begin(115200);
  Serial.setTimeout(100);

  getThetaBNO055Deg();
}


void loop()  
{
  NightOwlLoop();
}
