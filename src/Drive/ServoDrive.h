#ifndef _ServeDrive_H
#define _ServeDrive_H

#include  <Arduino.h>
#include  <Wire.h>
#include  <Adafruit_PWMServoDriver.h>

#define ServoMax 470        //50Hz:Maximum PWM
#define ServoMin 100        //50Hz:Minimum PWM

class ServoDrive
{
    private:
      float  Angle[16];                           //16-channel angle storage  
      
    public:
      ServoDrive();                               //Constructor
      void  begin();                              //Initialize
      void  Write(uint8_t servoNum,float pos);    //Control angle
      float Read(uint8_t servoNum);               //Read angle
      void  disattach(uint8_t servoNum);          //Disconnect servo
};

#endif
