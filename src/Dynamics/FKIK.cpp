#include "FKIK.h"

// Round and control decimal places
float Round(float in,int num){
  in = round(in*pow(10,num))/pow(10,num);
  return in;
}

// Degrees -> radians
float D2R(float deg){
  return deg*PI/180;
}

// Radians -> degrees
float R2D(float rad){
  return rad*180/PI;
}

// Forward kinematics function
void FK(float j0,float j1,float point[]){
  
  float x,y;              // Coordinates of the point in space

  j0 = D2R(j0);           // Degrees -> radians
  j1 = D2R(j1);

  // Core forward kinematics formula
  /*---------------------------------------------------------------------*/  
  x =   L0*sin(j0) - L1*sin(j0+j1);
  y = - L0*cos(j0) + L1*cos(j0+j1);
  /*---------------------------------------------------------------------*/
  
  point[0] = Round(x,3);
  point[1] = Round(y,3);
}

// Inverse kinematics function
void IK(float x,float y,float Angle[]){
  float j0,j1;                   // Leg's two joint angles

  // Inverse kinematics formula
  // 1. Solve j1
  j1 = acos(( (L0*L0+L1*L1) - (x*x+y*y))/(2*L0*L1));

  // 2. Solve j0
  float L2 = sqrt(x*x + y*y);
  float j2 = atan(abs(y/x));

  j0 = PI - asin((x*x + y*y + L0*L0 - L1*L1) / (2*L0*L2) ) - j2;

  // Store angle results
  Angle[0] = Round(R2D(j0),2);
  Angle[1] = Round(R2D(j1),2);
}
