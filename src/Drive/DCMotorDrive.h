#ifndef _DCMotorDrive_H
#define _DCMotorDrive_H

#include  <Arduino.h>
#include "IOs.h"
#include "../Dynamics/MotionControl.h"

#define RAIN1 4
#define RAIN2 5
#define RBIN1 2
#define RBIN2 15

#define LAIN1 12
#define LAIN2 13
#define LBIN1 14
#define LBIN2 27

class DCMotorDrive{
    public:
        DCMotorDrive();     // Constructor
        void begin();       // Initialize

        void stop();        // Brake
        
        void forword    (float rate);               // Move forward
        void backword   (float rate);               // Move backward
        void F_turnLeft (float rate,float pro);     // Front-left turn
        void F_turnRight(float rate,float pro);     // Front-right turn
        void B_turnLeft (float rate,float pro);     // Back-left turn
        void B_turnRight(float rate,float pro);     // Back-right turn

        void Test();
};

#endif
