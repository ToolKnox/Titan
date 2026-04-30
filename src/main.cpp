#include <Arduino.h>
#include "FreeRTOS.h"

// Drive modules
#include "Drive/DCMotorDrive.h"
#include "Drive/ServoDrive.h"
#include "Drive/IMU.h"
#include "Drive/Hiwonder.h"
#include "Drive/IOs.h"

// Kinematics modules
#include "Dynamics/FKIK.h"
#include "Dynamics/MotionControl.h"

// WiFi library
#include <WiFi.h>
#include <WiFiClient.h>

#define ssid      "ESP_Rabbit"
#define password  "12345678"

// *----------------------- Subfunction declarations -----------------------* //
void TCPDebug();
void SerialDebug();
void CmdSwitch(char cmd);
void ALLPOWOFF();
void HolderDebug();
void ShowStatus();
void Yes();
// *------------------------- Class declarations -------------------------* //
Robot         robot;
IMU           mpu;                  //MPU6050             
DCMotorDrive  DCm;                  // DC motor unit
ServoDrive    Servos;               // PWM servo group
WiFiServer    server(80);           // Start server on port 80 (AP)
WiFiClient    client_Move;

HardwareSerial ServoSer (1);        // Hiwonder servos
HardwareSerial OpenMVSer(2);        // OpenMV

extern U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2;
extern Adafruit_NeoPixel pixels;

/*########################################################### */
bool    Status_Last = 1;      // Command buffer
String  OLED_Str    = "";     // OLED string
/*########################################################### */

void setup() {
  pinMode(LEDpin,OUTPUT);
  digitalWrite(LEDpin,0);

  /*----------------------------------------------------------*/
  // Serial initialization
  Serial.   begin(115200);
  ServoSer. begin(115200,SERIAL_8N1,Servo_TXD,  Servo_RXD);
  // OpenMVSer.begin(115200,SERIAL_8N1,OpenMV_TXD, OpenMV_RXD);

  // Basic driver initialization
  DCm.    begin();
  mpu.    begin();
  robot.  begin();
  Servos. begin();

  InitIOs();

  /*----------------------------------------------------------*/
  // WiFi initialization
  Serial.println(F("** Configure WIFI..."));

  WiFi.mode(WIFI_AP);           // AP mode
  WiFi.softAP(ssid, password);  // WiFi SSID + password
  server.begin();
  
  Serial.print(F("  ----- AP--IPv4: "));Serial.println(WiFi.softAPIP());
  Serial.println(F("[WIFI]\tInit Success!"));

  /*----------------------------------------------------------*/
  Serial.println();
  Serial.println(F("|*****************************|"));
  Serial.println(F("|***> Robot System Start! <***|"));
  Serial.println(F("|*****************************|"));
  Serial.println();
  /*----------------------------------------------------------*/

  BlinkLed(LEDpin,2);  
  digitalWrite(LEDpin,1);
}


void loop() {
  // OLED_Debug();
  SerialDebug();
  TCPDebug();
}

/* ------------------ Command control ------------------ */
/*
*   Command control code
*/
void CmdSwitch(char cmd){
    /*
    * Quadruped mode
    */
  if(robot.RobotStatus){
    switch(cmd){
        case '0':ALLPOWOFF();       break;
        case '4':robot.Trot();      break;
        case '1':robot.Walk();      break;

        // case '6':robot.Walk();      break;
        // case '3':robot.Walk();      break;
        
        case '8':robot.PosAction2(1);break;   // Circle
        case '5':robot.PosAction3(); break;   // Pitch control     
        case '7':robot.PosAction1();break;    // Rectangle
        case '9':robot.PosAction4();break;    // Roll control 

        case 'x': 
          robot.RobotStatus = 0;   
          Serial.println(F("Switching to tracked mode..."));
          robot.ResetTrack(0);
          break;

        case 'a': robot.FK_ResetQuadruped();  break;
        case 'b': robot.IK_ResetQuadruped();  break;
        case 'c': robot.ResetTrack(0);        break;
        case 'd': robot.ResetTrack(1);        break;

        case 'e': 
          robot.RU_ReadPOS();
          robot.RB_ReadPOS();
          robot.LU_ReadPOS();
          robot.LB_ReadPOS();
        break;

        case 'g':
          HolderDebug();
          break;
        case 'h':
          Yes();
          break;
        }
  }else{
    /*
    * Tracked mode
    */
    switch(cmd){
      case '0':ALLPOWOFF();             break;
      case '5': DCm.stop();
            robot.ResetTrack(0);    break;

      case '8':DCm.forword  (1);        break;
      case '2':DCm.backword (1);        break;

      case '7':
        DCm.F_turnLeft (1,0.5);  
        break;   
      case '9':DCm.F_turnRight(1,0.5);  break;
      case '1':DCm.B_turnLeft (1,0.5);  break;
      case '3':DCm.B_turnRight(1,0.5);  break;

      case '4':robot.FK_TStatus_HIGH();   break;
      case '6':robot.FK_TStatus_HIGHER(); break;

      case 'x': 
        robot.RobotStatus = 1;   
        Serial.println(F("Switching to quadruped mode..."));
        robot.IK_ResetQuadruped(); 
        break;

      case 'a': robot.FK_ResetQuadruped();  break;
      case 'b': robot.IK_ResetQuadruped();  break;
      case 'c': robot.ResetTrack(0);        break;
      case 'd': robot.ResetTrack(1);        break;

      case 'e': 
        robot.RU_ReadPOS();
        robot.RB_ReadPOS();
        robot.LU_ReadPOS();
        robot.LB_ReadPOS();
        break;

      case 'f':
        DCm.Test();
        break;

      case 'g':
        HolderDebug();
        break;

      case 'h':
        Yes();
        break;
    }
  }  
}

/*
* Serial debug
*/
void SerialDebug(){
  ShowStatus();
  if(Serial.available()){
  
    uint8_t cmd = Serial.read();
    Serial.print(F("Order:"));
    Serial.println((char)cmd);

    BlinkLed(LEDpin,1);
    CmdSwitch(cmd);
  }
}

/*
* Network debug
*/
void TCPDebug(){
  WiFiClient client = server.available();   // Listen for incoming client

  if (client) {                             
    Serial.println(F("New Client..."));      
    BlinkLed(LEDpin,3); 

    client_Move = client;

    while (client.connected()) {            // If client is connected
      ShowStatus();
      
      if (client.available()) {             
        char cmd =  client.read();             

        if (cmd != '\n') {                  
          BlinkLed(LEDpin,1);               // LED blink once

          Serial.print(F("TCP-Order:"));
          Serial.println((char)cmd);

          CmdSwitch(cmd);                   // Receive control command
        }
      }
    }

    // TCP connection closed
    client.stop();
    Serial.println(F("Client Disconnected."));
    robot.RobotStatus = 1;
    digitalWrite(LEDpin,LOW);
  }
}

// Turn everything off
void ALLPOWOFF(){
  robot.PowerOff();
  DCm.stop();

  pixels.fill(pixels.Color(0, 0, 0));
  pixels.show();
  pixels.fill(pixels.Color(0, 0, 0));
  pixels.show();
   
  u8g2.clear();

}

// Gimbal test
void HolderDebug(){
  Servos.Write(0,90); 
  Servos.Write(15,45);
  delay(800);
  Servos.Write(0,45); 
  delay(400);
  Servos.Write(0,135);
  delay(800);
  Servos.Write(0,90);Servos.Write(15,0);
}

// Gimbal nod
void Yes(){
  Servos.Write(0,90); 
  Servos.Write(15,45);
  delay(800);

  for(uint8_t i = 0;i < 3;i++){
    Servos.Write(15,70); 
    delay(100);
    Servos.Write(15,0);
    delay(100);
  }
  Servos.Write(15,45);

  delay(800);
  Servos.Write(0,90);Servos.Write(15,0);
}
/**
 * Status indicator
 */
void ShowStatus(){
  if(Status_Last != robot.RobotStatus){
    if(!robot.RobotStatus){
      OLED_Str = "Track";
      // pixels.setPixelColor(0, pixels.Color(255, 127, 0)); // orange 
      pixels.fill(pixels.Color(255, 127, 0)); // orange 
    }else{
      OLED_Str = "Quadruped";
      // pixels.setPixelColor(0, pixels.Color(238, 18, 137)); // deep red
      pixels.fill(pixels.Color(238, 18, 137)); // deep red
    }
    pixels.show(); 

    u8g2.firstPage();        // Marks beginning of image loop
    do{
      u8g2.setCursor(128,48);  u8g2.print(OLED_Str);
    }while(u8g2.nextPage());  // Marks end of image loop
  }

  Status_Last = robot.RobotStatus;
}