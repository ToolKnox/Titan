#include "MotionControl.h"

extern HardwareSerial     ServoSer;
extern WiFiClient         client_Move;
extern Adafruit_NeoPixel  pixels;

void Robot::begin(){
  float xs = 70;
  // Get cycloid trajectory
  GetCycloidPoints(F1_CPoints,xs,       xs+XMOVE, YS,H);
  GetCycloidPoints(F2_CPoints,xs-XMOVE, xs,       YS,H);
  GetCycloidPoints(F3_CPoints,xs-XMOVE, xs+XMOVE, YS,H);

  GetCycloidPoints(B1_CPoints,xs,       xs-XMOVE, YS,H);
  GetCycloidPoints(B2_CPoints,xs+XMOVE, xs,       YS,H);
  GetCycloidPoints(B3_CPoints,xs+XMOVE, xs-XMOVE, YS,H);

  // Get initial point values
  IK_ResetQuadruped();
  PowerOff();

  Serial.println(F("[Robot]\tInit Success!"));
}
/* ------------------ Mode control ------------------ */
// Servos: clockwise + , counterclockwise -
void Robot::ResetTrack(bool status){
  uint16_t DSD = 250;

  if(status){
    FK_RUMove(125,55,DSD);  
    FK_LUMove(125,55,DSD);
    FK_RBMove(125,55,DSD);
    FK_LBMove(125,55,DSD);
  }else{
    FK_RUMove(90,90,DSD);  
    FK_LUMove(90,90,DSD);
    FK_RBMove(90,90,DSD);
    FK_LBMove(90,90,DSD);
    // LobotSerialServoMove(ServoSer, 254, 500, DSD);
  }
}

void Robot::FK_ResetQuadruped(){
  FK_RUMove(115,40,250);  
  FK_LUMove(115,40,250);
  FK_RBMove(115,40,250);
  FK_LBMove(115,40,250);
}

void Robot::IK_ResetQuadruped(){
  IK_RUMove(XS,YS,250);  
  IK_LUMove(XS,YS,250);
  IK_RBMove(XS,YS,250);
  IK_LBMove(XS,YS,250);
}

void Robot::FK_TStatus_HIGHER(){
  FK_RUMove(0,180,250);  
  FK_LUMove(0,180,250);
  FK_RBMove(0,180,250);
  FK_LBMove(0,180,250);
}

void Robot::FK_TStatus_HIGH(){
  FK_RUMove(45,135,250);  
  FK_LUMove(45,135,250);
  FK_RBMove(45,135,250);
  FK_LBMove(45,135,250);
}

void Robot::FK_LEFT(){
  FK_LUMove(90,90,150);  
  FK_LBMove(90,90,150);
  FK_RUMove(60,120,150);
  FK_RBMove(60,120,150);
}

void Robot::FK_RIGHT(){
  FK_RUMove(90,90,150);  
  FK_RBMove(90,90,150);
  FK_LUMove(60,120,150);
  FK_LBMove(60,120,150);
}

/* ------------------ Unload control ------------------ */
void Robot::PowerOff(){

  LobotSerialServoUnload(ServoSer,254);   // Broadcast ID

}

void Robot::LegUnload(uint8_t legNum){
  switch(legNum){
    case 1:     //1--->Right front leg
      LobotSerialServoUnload(ServoSer,0); 
      LobotSerialServoUnload(ServoSer,1);
      break;   
    case 2:     //2--->Right rear leg
      LobotSerialServoUnload(ServoSer,2); 
      LobotSerialServoUnload(ServoSer,3);
      break;   
    case 3:     //3--->Left rear leg
      LobotSerialServoUnload(ServoSer,4); 
      LobotSerialServoUnload(ServoSer,5);
      break;   
    case 4:     //4--->Left front leg
      LobotSerialServoUnload(ServoSer,6); 
      LobotSerialServoUnload(ServoSer,7);
      break;   
  }
}

/* ------------------ Single leg control ------------------ */
/*
* Angle Deg-->Servo control value Val
*/
uint16_t Robot::D2V(float angle){
  return angle/180.0*750+125;
}

/*
* Read angles
*/
void Robot::RU_ReadPOS(){
  RUAngle[0] = LobotSerialServoReadPosition(ServoSer,1);delay(100);
  RUAngle[1] = LobotSerialServoReadPosition(ServoSer,0);delay(100);
  
  Serial.print(F("RU read success: "));
  Serial.print(F("K:"));Serial.print(RUAngle[0]);Serial.print(F(" -- "));
  Serial.print(F("X:"));Serial.print(RUAngle[1]);Serial.println();
}

void Robot::RB_ReadPOS(){
  RBAngle[0] = 1000 - LobotSerialServoReadPosition(ServoSer,2);delay(100);
  RBAngle[1] = 1000 - LobotSerialServoReadPosition(ServoSer,3);delay(100);

  Serial.print(F("RB read success: "));
  Serial.print(F("K:"));Serial.print(RBAngle[0]);Serial.print(F(" -- "));
  Serial.print(F("X:"));Serial.print(RBAngle[1]);Serial.println();
}

void Robot::LB_ReadPOS(){
  LBAngle[0] = LobotSerialServoReadPosition(ServoSer,5);delay(100);
  LBAngle[1] = LobotSerialServoReadPosition(ServoSer,4);delay(100);

  Serial.print(F("LB read success: "));
  Serial.print(F("K:"));Serial.print(LBAngle[0]);Serial.print(F(" -- "));
  Serial.print(F("X:"));Serial.print(LBAngle[1]);Serial.println();
}

void Robot::LU_ReadPOS(){
  LUAngle[0] = 1000 - LobotSerialServoReadPosition(ServoSer,6);delay(100);
  LUAngle[1] = 1000 - LobotSerialServoReadPosition(ServoSer,7);delay(100);

  Serial.print(F("LU read success: "));
  Serial.print(F("K:"));Serial.print(LUAngle[0]);Serial.print(F(" -- "));
  Serial.print(F("X:"));Serial.print(LUAngle[1]);Serial.println();
}
/* ---------------------------------------------- */

/*
* Right front leg
*/
void Robot::FK_RUMove(float posk,float posx,uint16_t Time){
  posk = posk + 0;
  posx = posx - 0;
  LobotSerialServoMove(ServoSer, 1, D2V(posk), Time);
  LobotSerialServoMove(ServoSer, 0, D2V(posx), Time);
}
/*
* Right rear leg
*/
void Robot::FK_RBMove(float posk,float posx,uint16_t Time){
  posk = posk + 0;
  posx = posx + 0;
  LobotSerialServoMove(ServoSer, 2, 1000 - D2V(posk), Time);
  LobotSerialServoMove(ServoSer, 3, 1000 - D2V(posx), Time);
}
/*
* Left front leg
*/
void Robot::FK_LUMove(float posk,float posx,uint16_t Time){
  posk = posk + 0;
  posx = posx + 2;
  LobotSerialServoMove(ServoSer, 6, 1000 - D2V(posk), Time);
  LobotSerialServoMove(ServoSer, 7, 1000 - D2V(posx), Time);
}
/*
* Left rear leg
*/
void Robot::FK_LBMove(float posk,float posx,uint16_t Time){
  posk = posk + 6.5;
  posx = posx - 9;
  LobotSerialServoMove(ServoSer, 5, D2V(posk), Time);
  LobotSerialServoMove(ServoSer, 4, D2V(posx), Time);
}

/*
* Left side dual legs
*/
void Robot::FK_LSMove(float posk,uint16_t Time){
  FK_LUMove(posk,180 - posk,Time);
  FK_LBMove(posk,180 - posk,Time);
}
/*
* Right side dual legs
*/
void Robot::FK_RSMove(float posk,uint16_t Time){
  FK_RUMove(posk,180 - posk,Time);
  FK_RBMove(posk,180 - posk,Time);
}

// ********************************** //
// ************** Forward Kinematics ************** //
// ********************************** //
void Robot::FK_LegMove(float Angle[],uint8_t legNum,uint16_t DSD,bool Print){
  float point[2] = {};
  FK(Angle[0],Angle[1],point);

  if(Print){
    Serial.print(F("Calculated coordinates:"));
    Serial.print(F("x:"));Serial.print(point[0]);Serial.print(F(" -- "));
    Serial.print(F("y:"));Serial.print(point[1]);Serial.print(F(" -- "));
    Serial.println();
  }

  switch(legNum){
    case 1: FK_RUMove(Angle[0],Angle[1],DSD);  break;   //1--->Right front
    case 2: FK_RBMove(Angle[0],Angle[1],DSD);  break;   //2--->Right rear
    case 3: FK_LBMove(Angle[0],Angle[1],DSD);  break;   //3--->Left rear
    case 4: FK_LUMove(Angle[0],Angle[1],DSD);  break;   //4--->Left front
  }
}

// ********************************** //
// ************** Inverse Kinematics ************** //
// ********************************** //
/*
* Right front leg
*/
void Robot::IK_RUMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_RUPoint[0] = x;
  IK_RUPoint[1] = y;

  IK(x,y,angle);                    // Inverse kinematics
  angle[1] = angle[1] - Offset0;    // Offset correction

  FK_RUMove(angle[0],angle[1],Time);
}
/*
* Left front leg
*/
void Robot::IK_LUMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_LUPoint[0] = x;
  IK_LUPoint[1] = y;

  IK(x,y,angle);                    // Inverse kinematics
  angle[1] = angle[1] - Offset0;    // Offset correction

  FK_LUMove(angle[0],angle[1],Time);
}
/*
* Right rear leg
*/
void Robot::IK_RBMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_RBPoint[0] = x;
  IK_RBPoint[1] = y;

  IK(x,y,angle);                    // Inverse kinematics
  angle[1] = angle[1] - Offset0;    // Offset correction

  FK_RBMove(angle[0],angle[1],Time);
}
/*
* Left rear leg
*/
void Robot::IK_LBMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_LBPoint[0] = x;
  IK_LBPoint[1] = y;

  IK(x,y,angle);                    // Inverse kinematics
  angle[1] = angle[1] - Offset0;    // Offset correction

  FK_LBMove(angle[0],angle[1],Time);
}


void Robot::IK_LegMove(float Point[],uint8_t LEGNum,uint16_t DSD,bool Print){
  float angle[2] = {};
  IK(Point[0],Point[1],angle);
  angle[1] = angle[1] - Offset0;    // Offset correction

  if(Print){
    Serial.print(F("Calculated angles:"));
    Serial.print(F("K:"));Serial.print(angle[0]);Serial.print(F(" -- "));
    Serial.print(F("H:"));Serial.print(angle[1]);Serial.print(F(" -- "));
    Serial.println();
  }

  switch(LEGNum){
    case 1: FK_RUMove(angle[0],angle[1],DSD);  break;   //1--->Right front
    case 2: FK_RBMove(angle[0],angle[1],DSD);  break;   //2--->Right rear
    case 3: FK_LBMove(angle[0],angle[1],DSD);  break;   //3--->Left rear
    case 4: FK_LUMove(angle[0],angle[1],DSD);  break;   //4--->Left front
  }
}

/*
* Single leg debugging
*/
void Robot::LegPointDebug(float point[],uint8_t cmd,byte offset,bool PrintPoint,bool PrintAngle){
    bool LoadStatus = 0;

    switch(cmd){
      //X
      case '4': point[0] += offset;break;
      case '1': point[0] -= offset;break;
      //Y
      case '5': point[1] += offset;break;
      case '2': point[1] -= offset;break;
      //Leg
      case '0':
        legNum++;
        if(legNum == 5){
          legNum = 1;
        }
        Serial.print("LegNum:");
        Serial.println(legNum);
        break;
      case '8':
        LoadStatus = 1;
        break;
    }

      if(PrintPoint){
        Serial.print(F("Current coordinates:"));
        Serial.print(F("x:"));Serial.print(point[0]);Serial.print(F(" -- "));
        Serial.print(F("y:"));Serial.print(point[1]);Serial.print(F(" -- "));
        Serial.println();
      }

    if(!LoadStatus){
      IK_LegMove(point,legNum,250,PrintAngle);
    }else{
      LegUnload(legNum);
    }
}

/*
* Cycloid trajectory generation
*
* xs: X-axis start position
* xf: X-axis end position
* ys: Y-axis start position
* yh: Leg lift height
*/
void Robot::GetCycloidPoints(float CPoints[][2],float xs,float xf,float ys,float yh){
  uint8_t count = 0;

  float offset  = 0.06;             // Generate 10 points
  float sigma,xep,yep;
  float t;

  float Ts = 1,   fai = 0.5;        // period T, duty cycle fai (support phase)

  /* ############### Calculation section ############### */
  for(t = 0; t <= Ts*fai; t += offset){
    sigma = 2*PI*t/(fai*Ts);
    xep   = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;   // x-axis coordinate
    yep   = yh*(1-cos(sigma))/2+ys;                   // y-axis coordinate
    
    CPoints[count][0] = xep;
    CPoints[count][1] = yep;
    count += 1;
  }

  CPoints[count][0] = xf;
  CPoints[count][1] = ys;
  count += 1;
  /* ######################################## */
}

/*
* Foot cycloid motion
*/
void Robot::LegCycloid(float CPoints[][2],uint8_t LEGNum){
  uint8_t Num = 10;
  uint8_t DSD = 250/Num;

  for(uint8_t i = 0;i < 10;i++){
    switch (LEGNum){
      case 1:IK_RUMove(CPoints[i][0],CPoints[i][1],DSD); break;
      case 2:IK_RBMove(CPoints[i][0],CPoints[i][1],DSD); break;
      case 3:IK_LBMove(CPoints[i][0],CPoints[i][1],DSD); break;
      case 4:IK_LUMove(CPoints[i][0],CPoints[i][1],DSD); break;
    }
    delay(DSD);
  }
}

/* ------------------ Posture control ------------------ */
void Robot::PosMove(float xMove,float yMove,uint16_t Time){

  IK_RUMove(IK_RUPoint[0]-xMove, IK_RUPoint[1]-yMove, Time);
  IK_LUMove(IK_LUPoint[0]-xMove, IK_LUPoint[1]-yMove, Time);
  IK_RBMove(IK_RBPoint[0]+xMove, IK_RBPoint[1]-yMove, Time);
  IK_LBMove(IK_LBPoint[0]+xMove, IK_LBPoint[1]-yMove, Time);
}

void Robot::PosAction1(){
  uint8_t DSD = 200;
  PosMove(50,-25, DSD);  delay(DSD*4);
  PosMove(0,50,   DSD);  delay(DSD*4);
  PosMove(-100,0, DSD);  delay(DSD*4);
  PosMove(0,-50,  DSD);  delay(DSD*4);
  PosMove(50,25,  DSD);
}

void Robot::PosToPoint(float x,float y,uint16_t Time){

  IK_RUMove(XS-x, YS-y, Time);
  IK_LUMove(XS-x, YS-y, Time);
  IK_RBMove(XS+x, YS-y, Time);
  IK_LBMove(XS+x, YS-y, Time);
}

void Robot::PosToPitch(float deg,uint16_t Time){
  float LBody = 85.8;
  float Dx,Dy;

  deg = D2R(deg);   // Convert to radians

  Dx = LBody/2*(1 - cos(deg));
  Dy = LBody/2*sin(deg);

  IK_RUMove(XS+Dx, YS-Dy, Time);
  IK_LUMove(XS+Dx, YS-Dy, Time);
  IK_RBMove(XS+Dx, YS+Dy, Time);
  IK_LBMove(XS+Dx, YS+Dy, Time);
}

void Robot::PosToRoll (float deg,uint16_t Time){
  float WBody = 148;
  float Dh,rad;

  rad = D2R(abs(deg));         // Convert to radians

  Dh  = WBody*tan(rad);

  if(deg > 0){
    IK_RUMove(XS, YS - Dh,  Time);
    IK_RBMove(XS, YS - Dh,  Time);
    IK_LUMove(XS, YS,       Time);
    IK_LBMove(XS, YS,       Time);
  }else{
    IK_RUMove(XS, YS,       Time);
    IK_RBMove(XS, YS,       Time);
    IK_LUMove(XS, YS - Dh,  Time);
    IK_LBMove(XS, YS - Dh,  Time);
  }

}

void Robot::PosAction2(uint8_t Num){
  uint8_t DSD = 25;

  float r     = 30;
  float speed = 5;
  float deg   = 0;
  float theta;
  float xr,yr;

  for(uint8_t i = 0;i < Num;i++){
    while(deg <= 360){
      theta = D2R(deg);

      xr = r*cos(theta);
      yr = r*sin(theta);

      PosToPoint(xr,yr,DSD);
      delay(DSD);

      deg += speed;
    }

    deg = 0;
  }
  
  delay(500);
  PosToPoint(0,0,200);
}

void Robot::PosAction3(){
  PosToPitch(45,200);   // look up
  delay(800);
  PosToPitch(-45,200);  // look down
  delay(800);
  PosToPitch(0,200);    // reset position
}

void Robot::PosAction4(){
  PosToRoll(15,200);   // tilt left
  delay(800);
  PosToRoll(-15,200);  // tilt right
  delay(800);
  PosToRoll(0,200);    // reset position
}

/* ------------------ Motion control ------------------ */
void Robot::Walk_Basic(uint8_t SetpNum,bool dir){
  uint8_t RestTime  = 50;
  uint8_t RestTime2 = 100;
  
  if(dir){
    if(SetpNum != 0){
      //Right front + Left rear
      LegCycloid(F1_CPoints,1); delay(RestTime);
      LegCycloid(B1_CPoints,3); delay(RestTime);

      for(uint8_t i = 0;i < SetpNum - 1;i++){
        PosMove(XMOVE,0,RestTime2);  delay(RestTime2);

        //Left front + Right rear
        LegCycloid(F3_CPoints,4);   delay(RestTime);
        LegCycloid(B3_CPoints,2);   delay(RestTime);

        PosMove(XMOVE,0,RestTime2);  delay(RestTime2);

        //Right front + Left rear
        LegCycloid(F3_CPoints,1); delay(RestTime);
        LegCycloid(B3_CPoints,3); delay(RestTime);
      }

      PosMove(XMOVE,0,RestTime2);  delay(RestTime2);

      //Left front + Right rear
      LegCycloid(F2_CPoints,4);   delay(RestTime);
      LegCycloid(B2_CPoints,2);   delay(RestTime);
    }
  }else{
    if(SetpNum != 0){
      //Right front + Left rear
      LegCycloid(B1_CPoints,1); delay(RestTime);
      LegCycloid(F1_CPoints,3); delay(RestTime);

      for(uint8_t i = 0;i < SetpNum - 1;i++){
        PosMove(-XMOVE,0,RestTime2);  delay(RestTime2);

        //Left front + Right rear
        LegCycloid(B3_CPoints,4);   delay(RestTime);
        LegCycloid(F3_CPoints,2);   delay(RestTime);

        PosMove(-XMOVE,0,RestTime2);  delay(RestTime2);

        //Right front + Left rear
        LegCycloid(B3_CPoints,1); delay(RestTime);
        LegCycloid(F3_CPoints,3); delay(RestTime);
      }

      PosMove(-XMOVE,0,RestTime2);  delay(RestTime2);

      //Left front + Right rear
      LegCycloid(B2_CPoints,4);   delay(RestTime);
      LegCycloid(F2_CPoints,2);   delay(RestTime);
    }
  }
}  

/*
* Trot gait (trotting)
*/
void Robot::Trot(){

  TrotStatus  = 1;
  uint8_t dir = 2;
  uint8_t DSD = 20;
  uint8_t cmd = 0;

  float sigma,xep_b,xep_z,yep;
  float t = 0;
  
  float speed = 0.05;                         // Step frequency

  float Ts = 1,     fai = 0.5;                // Period T, duty cycle fai (support phase)
  float offset = 20;

  float xs = XS - offset,   xf = XS + offset; // Start and end coordinates
  float ys = -130,  yh  = 20;                 // Start position and leg lift height

  //Foot endpoint coordinates
  float x1 = XS,x2 = XS,x3 = XS,x4 = XS;    
  float y1 = YS,y2 = YS,y3 = YS,y4 = YS;

  Serial.println(F("Starting Trot mode..."));

  pixels.fill(pixels.Color(0, 127, 255)); // cyan
  pixels.show(); 

  while(TrotStatus){
    // 1. Trot command
    if(Serial.available() || client_Move.available()){

      if(Serial.available())      cmd = Serial.read();
      if(client_Move.available()) cmd = client_Move.read();
            
      if(cmd != '\n'){
        BlinkLed(LEDpin,1); 
        Serial.printf("  *Trot--Order:%c\n",cmd);

        switch(cmd){
          case '8':dir = 1;       break;
          case '5':dir = 2;       break;
          case '2':dir = 0;       break;
          case 's':dir = 3;       break;

          // #Parameter debugging
          case '4':offset += 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '1':offset -= 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '6':yh     += 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '3':yh     -= 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '7':DSD    += 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;
          case '9':DSD    -= 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;

          case '0':
            TrotStatus = 0;
            pixels.fill(pixels.Color(238, 18, 137)); // Quadruped
            pixels.show(); 
            break;
        }
      }
    }

    // 2. Single gait cycle
    while(t < 1){
      // First half cycle
      if(t <= Ts*fai){
        sigma  = 2*PI*t/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y-axis coordinate

        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // Swing phase x-axis coordinate
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // Support phase x-axis coordinate

        if(dir != 2){
          y1 = yep;
          y2 = ys;
          y3 = yep;
          y4 = ys;
        }

        if(dir == 1){
          x1 = xep_b;
          x2 = xep_z;
          x3 = xep_z;
          x4 = xep_b;
          
        }else if(dir == 0){
          x1 = xep_z;
          x2 = xep_b;
          x3 = xep_b;
          x4 = xep_z;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }
      // Second half cycle
      if(Ts*fai < t && t < Ts){
        sigma  = 2*PI*(t - Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y-axis coordinate

        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // Swing phase x-axis coordinate
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // Support phase x-axis coordinate

        if(dir != 2){
          y1 = ys;
          y2 = yep;
          y3 = ys;
          y4 = yep;
        }

        if(dir == 1){
          x1 = xep_z;
          x2 = xep_b;
          x3 = xep_b;
          x4 = xep_z;
        }else if(dir == 0){
          x1 = xep_b;
          x2 = xep_z;
          x3 = xep_z;
          x4 = xep_b;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }

      t = t + speed;

      IK_RUMove(x1,y1,DSD);
      IK_LUMove(x2,y2,DSD);
      IK_LBMove(x3,y3,DSD);
      IK_RBMove(x4,y4,DSD);

      delay(DSD);
    }

    if(TrotStatus){
      t = 0;
    }   
  }

  IK_ResetQuadruped();
  Serial.println(F("Exiting Trot mode..."));
}
/*
* Walk gait (walking)
*/
void Robot::Walk(){

  WalkStatus  = 1;
  uint8_t dir = 2;
  uint8_t DSD = 20;
  uint8_t cmd = 0;

  float sigma,xep_b,xep_z,yep;
  float t = 0;

  float Ts = 1,     fai = 0.25;      // Period T, duty cycle fai (support phase)
  
  float speed = 0.0249;               //Step frequency
  float offset = 20;

  float xs = XS - offset,   xf = XS + offset; // Start and end coordinates

  // float xs = 45 ,   xf  = 135;
  float ys = -130,  yh  = 20;

  //Foot end coordinates
  float x1,x2,x3,x4;    
  float y1,y2,y3,y4;

  float step = (xf - xs) / 30.0;

  x1 = xs;
  x2 = xs + 21*step;
  x3 = xs + 19*step;
  x4 = xs - step;

  y1 = ys;
  y2 = ys;
  y3 = ys;
  y4 = ys;

  Serial.println(F("Starting Walk mode..."));
  pixels.fill(pixels.Color(98,255, 10));   // green
  pixels.show(); 

  while(WalkStatus){

    // 1. Walk command
    if(Serial.available() || client_Move.available()){

      if(Serial.available())      cmd = Serial.read();
      if(client_Move.available()) cmd = client_Move.read();
      
      if(cmd != '\n'){
        BlinkLed(LEDpin,1); 
        Serial.printf("  *Walk--Order:%c\n",cmd);
        
        switch(cmd){
          case '2':dir = 0;       break;
          case '8':dir = 1;       break;
          case '5':dir = 2;       break;
          case 's':dir = 3;       break;

          // #Parameter debugging
          case '4':offset += 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '1':offset -= 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '6':yh     += 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '3':yh     -= 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '7':DSD    += 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;
          case '9':DSD    -= 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;

          case '0':
            WalkStatus = 0;
            pixels.fill(pixels.Color(238, 18, 137)); // Quadruped
            pixels.show(); 
            break;
        }
      }
    }

    while(t < 1){
      // 1. RU (Right upper leg)
      if(t < Ts*fai){
        sigma  = 2*PI*t/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y-axis coordinate
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // Swing phase x-axis coordinate
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // Support phase x-axis coordinate

        if(dir == 1 || dir == 3){
          y1 = yep;
          y2 = ys;
          y3 = ys;
          y4 = ys;
        }else if(dir == 0){
          y1 = ys;
          y2 = ys;
          y3 = ys;
          y4 = yep;
        }

        if(dir == 1){
          x1 = xep_b;
          x2 = x2 - step;
          x3 = x3 + step;
          x4 = x4 + step;
        }else if(dir == 0){
          x1 = x1 + step;
          x2 = x2 + step;
          x3 = x3 - step;
          x4 = xep_b;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }

      // 2. LB (Left rear leg)
      if(  Ts*fai < t && t < 2*Ts*fai){
        sigma  = 2*PI*(t - Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y-axis coordinate
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // Swing phase x-axis coordinate
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // Support phase x-axis coordinate

        if(dir == 1 || dir == 3){
          y1 = ys;
          y2 = ys;
          y3 = yep;
          y4 = ys;
        }else if(dir == 0){
          y1 = ys;
          y2 = yep;
          y3 = ys;
          y4 = ys;
        }

        if(dir == 1){
          x1 = x1 - step;
          x2 = x2 - step;
          x3 = xep_z;
          x4 = x4 + step;
        }else if(dir == 0){
          x1 = x1 + step;
          x2 = xep_z;
          x3 = x3 - step;
          x4 = x4 - step;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }
      // 3. LU (Left upper leg)
      if(2*Ts*fai < t && t < 3*Ts*fai){
        sigma  = 2*PI*(t - 2*Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y-axis coordinate
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // Swing phase x-axis coordinate
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // Support phase x-axis coordinate

        if(dir == 1 || dir == 3){
          y1 = ys;
          y2 = yep;
          y3 = ys;
          y4 = ys;
        }else if(dir == 0){
          y1 = ys;
          y2 = ys;
          y3 = yep;
          y4 = ys;
        }

        if(dir == 1){
          x1 = x1 - step;
          x2 = xep_b;
          x3 = x3 + step;
          x4 = x4 + step;
        }else if(dir == 0){
          x1 = x1 + step;
          x2 = x2 + step;
          x3 = xep_b;
          x4 = x4 - step;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }
      // 4. RB (Right rear leg)
      if(3*Ts*fai < t && t < 4*Ts*fai){
        sigma  = 2*PI*(t - 3*Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y-axis coordinate
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // Swing phase x-axis coordinate
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // Support phase x-axis coordinate

        if(dir == 1 || dir == 3){
          y1 = ys;
          y2 = ys;
          y3 = ys;
          y4 = yep;
        }else if(dir == 0){
          y1 = yep;
          y2 = ys;
          y3 = ys;
          y4 = ys;
        }

        if(dir == 1){
          x1 = x1 - step;
          x2 = x2 - step;
          x3 = x3 + step;
          x4 = xep_z;
        }else if(dir == 0){
          x1 = xep_z;
          x2 = x2 + step;
          x3 = x3 - step;
          x4 = x4 - step;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }    

      t = t + speed;

      IK_RUMove(x1,y1,DSD);
      IK_LUMove(x2,y2,DSD);
      IK_LBMove(x3,y3,DSD);
      IK_RBMove(x4,y4,DSD);
      delay(DSD);
    }

    if(WalkStatus){
      t = 0;
    }  
  }

  IK_ResetQuadruped();
  Serial.println(F("Exiting Walk mode..."));
}
/*
* VMC
*/
void Robot::VMC(float j0,float j1,float VMCPoint[],float VMCPoint_last[]){

  float Fx,Fy;                // Virtual force
  float Xe,Ze;                // Desired target/control target (coordinate position)

  // ****** ****** ****** //
  Xe = XS;
  Ze = YS;
  // ****** ****** ****** //  

  float X,Z;                        // Feedback position
  float Vx,Vz;                      // Feedback velocity

  float KP_X = 0.9, KP_Z = 0.9;     // Spring coefficient
  float KD_X = 0.05,KD_Z = 0.05;    // Damping coefficient

  // (1) Load previous coordinates
  X = VMCPoint[0];
  Z = VMCPoint[1];

  Vx = X - VMCPoint_last[0];
  Vz = Z - VMCPoint_last[1];

  // Virtual spring damping - force calculation
  Fx = KP_X*(Xe - X) + KD_X*(0 - Vx);
  Fy = KP_Z*(Ze - Z) + KD_Z*(0 - Vz);

  // Store previous position
  VMCPoint_last[0] = X;
  VMCPoint_last[1] = Z;

  // (2) ***** Jacobian matrix ***** //
  float xd_0,xd_1;      // x partial derivative j0,j1
  float yd_0,yd_1;      // y partial derivative j0,j1

  j0  = D2R(j0);        // Convert to radians
  j1  = D2R(j1);

  xd_0 =  L0*cos(j0) - L1*cos(j0+j1);
  xd_1 = -L1*cos(j0+j1);

  yd_0 =  L0*sin(j0) - L1*sin(j0+j1);
  yd_1 = -L1*sin(j0+j1);

  // (3) ***** Solve torque ***** //
  float t0,t1;          // Torque
  
  t0 = Fx*xd_0 + Fy*yd_0;
  t1 = Fx*xd_1 + Fy*yd_1;

  // (4) ***** Torque to angular velocity ***** //
  float KP_S = 0.002;
  float a0,a1;

  a0 = KP_S*t0;
  a1 = KP_S*t1;

  j0 = R2D(j0);
  j1 = R2D(j1);

  j0 += a0;
  j1 += a1;

  // (5) ***** Forward solution pseudo feedback ***** //
  FK(j0,j1,VMCPoint);

  /*
  * Return value:
  *   1. VMCPoint coordinate obtained from this calculation;
  *   2. VMCPoint_last coordinate obtained from last calculation。
  */

}
/* ---- Motion Design ---- */
void Robot::Hello(){

}

/* ---- Information Extraction ---- */
/*
Get data:
  0->Position;
  1->Voltage;
  2->Temperature.
*/
void Robot::GetServoInfo(bool Pos,bool Vin,bool Temp){
  uint8_t i;
  //Position
  if(Pos){
    for(i = 0;i < SerNum;i++){
      SPos[i] = LobotSerialServoReadPosition(ServoSer,i);
    }
  }

  //Voltage
  if(Vin){
    for(i = 0;i < SerNum;i++){
      SVin[i] = LobotSerialServoReadVin(ServoSer,i);
    }
  }

  //Temperature
  if(Temp){
    for(i = 0;i < SerNum;i++){
      STemp[i] = LobotSerialServoReadTemp(ServoSer,i);
    }
  }
}                                  


