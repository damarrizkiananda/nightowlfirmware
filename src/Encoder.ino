signed long lastCounter1, lastCounter2, lastCounter3;

void update_speed()
{
  speed1 = (int32_t)(enc1.read()/4-lastCounter1);
  speed2 = (int32_t)(enc2.read()/4-lastCounter2);
  speed3 = (int32_t)(enc3.read()/4-lastCounter3);

  lastCounter1 = enc1.read()/4;
  lastCounter2 = enc2.read()/4;
  lastCounter3 = enc3.read()/4;  

  getPosition();

  sendDataPlease = true;
}

#define sqrt3 1.73205080757
double sin_theta, cos_theta;
double x_count, y_count;

void getPosition()
{
  sin_theta = sin(yaw*TO_RAD);
  cos_theta = cos(yaw*TO_RAD);
  
  y_count += (sqrt3 * sin_theta * (speed1 - speed2) + cos_theta * (speed1 + speed2 - 2 * speed3)) / 3;
  x_count -= (-sqrt3 * cos_theta * (speed1 - speed2) + sin_theta * (speed1 + speed2 - 2 * speed3)) / 3;
       
  x = (int)(x_count * DIST_PER_PULSE);
  y = (int)(y_count * DIST_PER_PULSE);
   
}



//void updatePosition(){
//  interv = millis() - last_time;
//  if (interv > 10){
//    for (int i = 0; i < 3; i++)
//    {
//      delta_count[i] = rot_count[i] - last_count[i];
//      last_count[i] = rot_count[i];
//    }   
//    
//    theta += -(delta_count[0] + delta_count[1] + delta_count[2]) * DIST_PER_PULSE / (3 * LENGTH);
//    if (theta > 2 * PI) theta -= 2 * PI;
//    else if(theta < 0) theta += 2 * PI;
//    cos_theta = cos(theta);
//    sin_theta = sin(theta);
//
//    x_count -= (sqrt3 * sin_theta * (delta_count[0] - delta_count[1]) + cos_theta * (delta_count[0] + delta_count[1] - 2 * delta_count[2])) / 3;
//    y_count -= (-sqrt3 * cos_theta * (delta_count[0] - delta_count[1]) + sin_theta * (delta_count[0] + delta_count[1] - 2 * delta_count[2])) / 3;
//
//    posx = (int16_t)(x_count * DIST_PER_PULSE);
//    posy = (int16_t)(y_count * DIST_PER_PULSE);
//    orient = (int16_t)(theta * TO_DEG);
//    if(orient == 360) orient = 0;
//    else if(orient > 360) orient -= 360;
//    else if(orient < 0) orient += 360;
//
//    last_time = millis();
//  }
//}


