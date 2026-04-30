#ifndef _FKIK_H
#define _FKIK_H

#include <Arduino.h>
#include <math.h>

#define L0 66.4
#define L1 145

#define Offset0 32.39

float Round(float in,int num);                          // Control decimal places
float D2R(float deg);                                   // Degrees to radians
float R2D(float rad);                                   // Radians to degrees

void FK(float j0,float j1,float point[]);               // Forward kinematics function
void IK(float x,float y,float Angle[]);                 // Inverse kinematics function


#endif
