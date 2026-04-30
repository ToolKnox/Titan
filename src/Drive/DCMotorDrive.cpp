#include "DCMotorDrive.h"

Robot DCrobot;

DCMotorDrive::DCMotorDrive(){

}



void DCMotorDrive::begin(){
    PWM_Init(0,LAIN1);      PWM_Control(0,0);
    PWM_Init(1,LBIN1);      PWM_Control(1,0);
    PWM_Init(2,RAIN1);      PWM_Control(2,0);
    PWM_Init(3,RBIN1);      PWM_Control(3,0);

    pinMode(LAIN2,OUTPUT);  digitalWrite(LAIN2,0);
    pinMode(LBIN2,OUTPUT);  digitalWrite(LBIN2,0);
    pinMode(RAIN2,OUTPUT);  digitalWrite(RAIN2,0);
    pinMode(RBIN2,OUTPUT);  digitalWrite(RBIN2,0);

    Serial.println(F("[DCms]\tInit Success!"));
}

/**
 * Brake
 */
void DCMotorDrive::stop(){
    PWM_Control(0,0);
    PWM_Control(1,0);
    PWM_Control(2,0);
    PWM_Control(3,0);
    
    digitalWrite(LAIN2,0);
    digitalWrite(LBIN2,0);
    digitalWrite(RAIN2,0);
    digitalWrite(RBIN2,0);
}

/**
 * Move forward
 */
void DCMotorDrive::forword(float rate){
    // start();

    uint16_t speed = 1024*rate;

    PWM_Control(0,speed);
    PWM_Control(1,speed);
    PWM_Control(2,speed);
    PWM_Control(3,speed);

    digitalWrite(LAIN2,0);
    digitalWrite(LBIN2,0);
    digitalWrite(RAIN2,0);
    digitalWrite(RBIN2,0);
}

/**
 * Move backward
 */
void DCMotorDrive::backword(float rate){

    uint16_t speed = 1024*(1 - rate);

    PWM_Control(0,speed);
    PWM_Control(1,speed);
    PWM_Control(2,speed);
    PWM_Control(3,speed);


    digitalWrite(LAIN2,1);
    digitalWrite(LBIN2,1);
    digitalWrite(RAIN2,1);
    digitalWrite(RBIN2,1);
}

float r1 = 1.0;
float r2 = 0.8;
/**
 * Front-left turn
 */
void DCMotorDrive::F_turnLeft(float rate,float pro){

    uint16_t speed = 1024*rate;
    DCrobot.FK_LEFT();

    // //Significant tilt
    // DCrobot.FK_LUMove(125,55,150);  
    // DCrobot.FK_LBMove(125,55,150);
    // DCrobot.FK_RUMove(45,135,150);
    // DCrobot.FK_RBMove(45,135,150);

    PWM_Control(0,speed*r2);
    PWM_Control(1,speed*r2);
    PWM_Control(2,speed);
    PWM_Control(3,speed);

    digitalWrite(LAIN2,0);
    digitalWrite(LBIN2,0);
    digitalWrite(RAIN2,0);
    digitalWrite(RBIN2,0);
    
}

/**
 * Front-right turn
 */
void DCMotorDrive::F_turnRight(float rate,float pro){

    uint16_t speed = 1024*rate;
    DCrobot.FK_RIGHT();

    PWM_Control(0,speed);
    PWM_Control(1,speed);
    PWM_Control(2,speed*r2);
    PWM_Control(3,speed*r2);


    digitalWrite(LAIN2,0);
    digitalWrite(LBIN2,0);
    digitalWrite(RAIN2,0);
    digitalWrite(RBIN2,0);
}

/**
 * Back-left turn
 */
void DCMotorDrive::B_turnLeft(float rate,float pro){

    uint16_t speed = 1024*rate;
    DCrobot.FK_LEFT();

    PWM_Control(0,speed*(1 - r2));
    PWM_Control(1,speed*(1 - r2));

    PWM_Control(2,speed*0);
    PWM_Control(3,speed*0);

    digitalWrite(LAIN2,1);
    digitalWrite(LBIN2,1);
    digitalWrite(RAIN2,1);
    digitalWrite(RBIN2,1);
}

/**
 * Back-right turn
 */
void DCMotorDrive::B_turnRight(float rate,float pro){

    uint16_t speed = 1024*rate;
    DCrobot.FK_RIGHT();

    PWM_Control(0,speed*0);
    PWM_Control(1,speed*0);
    
    PWM_Control(2,speed*(1 - r2)); 
    PWM_Control(3,speed*(1 - r2));

    digitalWrite(LAIN2,1);
    digitalWrite(LBIN2,1);
    digitalWrite(RAIN2,1);
    digitalWrite(RBIN2,1);
}

void DCMotorDrive::Test(){

    uint16_t speed = 1024;

    PWM_Control(0,speed*r2);
    PWM_Control(1,speed*r2);
    PWM_Control(2,speed*(1-r2)); 
    PWM_Control(3,speed*(1-r2));
    
    digitalWrite(LAIN2,0);
    digitalWrite(LBIN2,0);
    digitalWrite(RAIN2,1);
    digitalWrite(RBIN2,1);
}
// pinMode(LAIN1,OUTPUT);  digitalWrite(LAIN1,0);
// pinMode(LBIN1,OUTPUT);  digitalWrite(LBIN1,0);
// pinMode(RAIN1,OUTPUT);  digitalWrite(RAIN1,0);
// pinMode(RBIN1,OUTPUT);  digitalWrite(RBIN1,0);
// digitalWrite(LAIN1,0);
// digitalWrite(LBIN1,0);
// digitalWrite(RAIN1,1);
// digitalWrite(RBIN1,1);
