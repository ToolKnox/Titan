#include "IOs.h"

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE,22,21);  // OLED -> 2-page buffer
Adafruit_NeoPixel pixels(WS_COUNT,WS_PIN, NEO_GRB + NEO_KHZ800);        // WS2812B

void InitIOs(){
    Serial.println(F("** Configure IOs..."));

    //Rotary encoder
    pinMode(EC_A,INPUT);
    pinMode(EC_K,INPUT);
    pinMode(EC_B,INPUT);
    Serial.println(F("  -----EC Init Success!"));

    //Buzzer
    pinMode(Buzzerpin,OUTPUT);
    digitalWrite(Buzzerpin,0);
    Serial.println(F("  -----BEEP Init Success!"));

    //WS2812B
    pixels.begin();
    // pixels.show();            // Turn OFF all pixels ASAP
    // pixels.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    Serial.println(F("  -----WS2812B Init Success!"));

    //OLED
    u8g2.begin();                           //u8g2 initialization
    u8g2.setFontDirection(2);               //Set font direction:180  
    u8g2.enableUTF8Print();                 //Encoding settings
    u8g2.setFont(u8g2_font_8x13_me);        //Set font
    Serial.println(F("  -----OLED Init Success!"));

    Serial.println(F("[IOs]\tInit Success!"));
}

void EC_Debug(){
  Serial.print(digitalRead(EC_A));Serial.print(F(" - "));
  Serial.print(digitalRead(EC_K));Serial.print(F(" - "));
  Serial.print(digitalRead(EC_B));Serial.println();
}

void OLED_Debug(){
  // Known rotated 180°
  u8g2.firstPage();        // Flag the start of image loop
  do{
    u8g2.setCursor(128,48);  u8g2.print("Hello World");

  }while(u8g2.nextPage());  // Flag the end of image loop
}

void BlinkLed(uint8_t lelpin,uint8_t blinknum){
  uint8_t i;
  for(i = 0;i < blinknum;i++){
    digitalWrite(lelpin,0);delay(80);
    digitalWrite(lelpin,1);delay(80);
  }
}

int Motor_freq_PWM        = 2000;         // PWM frequency
int Motor_resolution_PWM  = 10;           // PWM resolution, value between 0-20,2^10,1024

//Channels 0-16, do not conflict with other timers, bind io
void PWM_Init(int PWM_Channel, int PWM_IO){
  pinMode(PWM_IO, OUTPUT);
  ledcSetup(PWM_Channel, Motor_freq_PWM, Motor_resolution_PWM);   // Setup channel
  ledcAttachPin(PWM_IO, PWM_Channel);                             // Bind channel to specified IO pin
}

//pwm's duty cycle
void PWM_Control(int PWM_Channel, int DutyA){

  ledcWrite(PWM_Channel, DutyA);
}