/*
 *  Wheel Configuration
 *           
 *     o2         o1
 * 
 * 
 * 
 *           o3
 * 
 * 
 *  Axis
 *            x+   
 *             I 
 *             I 
 *     y+ _ _ _I
 * 
 * 
 */

/* Distance between the center of the robot to the wheels in m */
#define L 0.3

/* Important Angle in Degree */
#define delta 30

/* This function convert target vx, vy, & omega to wheel speed (pwm1, pwm2, pwm3 (global variables)) */ 
void inverseKinematics(int vx, int vy, int omega)
{
  pwm1 =  cos(delta*TO_RAD)*vy + sin(delta*TO_RAD)*vx + L*omega;
  pwm2 = -(-cos(delta*TO_RAD)*vy + sin(delta*TO_RAD)*vx) + L*omega;
  pwm3 = -vy + L*omega;  
}

void positionPID(int targetX, int targetY, int targetTheta)
{
  SetpointX = targetX;
  SetpointY = targetY;
  SetpointTheta = targetTheta;

  noInterrupts();
  InputX = x;
  InputY = y;
  InputTheta = yaw;
  interrupts();

  positionPIDX.Compute();
  positionPIDY.Compute();
  positionPIDTheta.Compute();

  inverseKinematics(OutputX, OutputY, OutputTheta);
  
  /* With Motor Speed Control */
  motorPID(pwm1,pwm2,pwm3);

  /* Without motor speed control */
  //robotMotorWrite(pwm1,pwm2,pwm3);
  
}

void motorPID(int target1, int target2, int target3)
{ 
  Setpoint1 = target1;
  Setpoint2 = target2;
  Setpoint3 = target3;

  noInterrupts();
  Input1 = speed1;
  Input2 = speed2;
  Input3 = speed3;
  interrupts();

  motorPID1.Compute();
  motorPID2.Compute();
  motorPID3.Compute();

  robotMotorWrite(Output1, Output2, Output3);
}


/* Motion filter to filter motions and compliance */
/* Higher filter value -> Smoother and slower response */

/* Usage
 * 
 * float rawVal; // unfiltered value
 * float filteredVal; // filtered output
 * float filteredValPrev; // bookmarked previous value
 * int filter = 10;
 *
 * In the loop:
 * 
 * filteredVal = filter(filteredValPrev, rawVal, filter);
 * filteredValPrev = filteredVal;
 * 
 */

float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}






