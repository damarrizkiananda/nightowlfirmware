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

/* This function convert target vx, vy, & omega to wheel Velocity (global variables) */ 
void inverseKinematics(int vx, int vy, int omega)
{
  wheelVelocity1_Target =  cos(delta*TO_RAD)*vy + sin(delta*TO_RAD)*vx + L*omega;
  wheelVelocity2_Target = -(-cos(delta*TO_RAD)*vy + sin(delta*TO_RAD)*vx) + L*omega;
  wheelVelocity3_Target = -vy + L*omega;  
}

void positionPID(int targetX, int targetY, int targetTheta)
{
  noInterrupts();
  InputX = x_Real;
  InputY = y_Real;
  InputTheta = yaw_Real;
  auto currentTheta = yaw_Real;
  interrupts();

  if(targetTheta-currentTheta < -180)
  {
    targetTheta+=360;
  }

  if(targetTheta-currentTheta > 180)
  {
    targetTheta-=360;
  }

  SetpointX = targetX;
  SetpointY = targetY;
  SetpointTheta = targetTheta;

  positionPIDX.Compute();
  positionPIDY.Compute();
  positionPIDTheta.Compute();

  double sin_theta = sin(-currentTheta*TO_RAD);
  double cos_theta = cos(-currentTheta*TO_RAD);
  
  double globalOutputX = OutputX*cos_theta - OutputY*sin_theta;
  double globalOutputY = OutputX*sin_theta + OutputY*cos_theta;

  inverseKinematics(globalOutputX, globalOutputY, OutputTheta);
  
  /* With Motor Velocity Control */
  //motorPID(wheelVelocity1_Target,wheelVelocity2_Target,wheelVelocity3_Target);

  /* Without motor Velocity control */
  robotMotorWrite(wheelVelocity1_Target,wheelVelocity2_Target,wheelVelocity3_Target);
  
}

void motorPID(int target1, int target2, int target3)
{ 
  Setpoint1 = target1;
  Setpoint2 = target2;
  Setpoint3 = target3;

  noInterrupts();
  Input1 = wheelVelocity1_Real;
  Input2 = wheelVelocity2_Real;
  Input3 = wheelVelocity3_Real;
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

double x_Target, y_Target, yaw_Target;

void trajectoryControl()
{

}