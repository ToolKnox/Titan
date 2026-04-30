#include "IMU.h"
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 IMU_MPU6050;

void IMU::begin(){
  Wire.begin();             // I2C initialization
  IMU_MPU6050.initialize(); // MPU initialization

  Serial.println(IMU_MPU6050.testConnection() ? F("[IMU]\tInit Success!") : F("[IMU]\tInit Failed!"));  // Connection test

  InitDMP();

  // Offset calibration
  Acc_Gyro_Calibration(500);
	Ypr_Calibration     (500);
}

/* ------------------------------ DMP ------------------------------ */
void IMU::InitDMP(){
  // 1. Load and configure DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = IMU_MPU6050.dmpInitialize();

  // 2. Provide gyro offsets, scaled to minimal sensitivity
  IMU_MPU6050.setXGyroOffset(250);
  IMU_MPU6050.setYGyroOffset(76);
  IMU_MPU6050.setZGyroOffset(-85);
  IMU_MPU6050.setZAccelOffset(1788);

  // 3. Check status
  if (devStatus == 0) {
    // Enable DMP
    Serial.println(F("Enabling DMP..."));
    IMU_MPU6050.setDMPEnabled(true);

    // Set DMP ready flag
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = IMU_MPU6050.dmpGetFIFOPacketSize();
    Serial.println(F("DMP Ready!"));

  } else {
    // Error:
    //  1 = initial memory load failed
    //  2 = DMP config update failed
    // (code usually 1 for halt)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

/* ---------------------------------------------------------------- */
Quaternion q;               // [w, x, y, z]         Quaternion
VectorFloat gravity;        // [x, y, z]            Gravity vector
/* ---------------------------------------------------------------- */

void IMU::DMP_Update(){
  // If not ready, do nothing
  if (!dmpReady) return;

  fifoCount       = IMU_MPU6050.getFIFOCount();           // Get current FIFO count

  // Check overflow
  if (fifoCount == 1024) {       
    IMU_MPU6050.resetFIFO();                              // Reset
    Serial.println(F("FIFO overflow!"));
  } else{                                   // Otherwise, check DMP data-ready interrupt
    while (fifoCount < packetSize) fifoCount = IMU_MPU6050.getFIFOCount();  // Wait for correct available data length (should be short)

    IMU_MPU6050.getFIFOBytes(fifoBuffer, packetSize);                       // Read packet from FIFO
    fifoCount -= packetSize;

    IMU_MPU6050.dmpGetQuaternion  (&q, fifoBuffer);     // Get quaternion
    IMU_MPU6050.dmpGetGravity     (&gravity, &q);       // Get gravity vector
    IMU_MPU6050.dmpGetYawPitchRoll(ypr, &q, &gravity);  // Get yaw/pitch/roll
    
    Ag[0] = ypr[1] * RAD - Ax_offset;  //pitch
    Ag[1] = ypr[2] * RAD - Ay_offset;  //roll
    Ag[2] = ypr[0] * RAD - Az_offset;  //yaw
  }
}

/* ------------------------------ Offset calibration ------------------------------ */
void IMU::Acc_Gyro_Calibration(uint16_t times = 200){  
  
  Serial.println(F("Acc_Gyro_Calibration in progress..."));

    for(uint16_t i=0;i<times;i++){                            // Number of samples
      IMU_MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read raw 6-axis values
      axo += ax; ayo += ay; azo += az;                      // Accumulate samples
      gxo += gx; gyo += gy; gzo += gz;
    }
    axo /= times; ayo /= times; azo /= times;                 // Compute accelerometer offsets
    gxo /= times; gyo /= times; gzo /= times;                 // Compute gyroscope offsets
}

void IMU::Ypr_Calibration(uint16_t times = 500){

  Serial.println(F("Ypr_Calibration in progress..."));
  Serial.printf("DMP Code:%d\n",dmpReady);

  SumX = 0; SumY = 0; SumZ = 0;

	for(uint16_t i = 0;i < times;i++){
    if(dmpReady)  {DMP_Update();}
    else          {Attitude_Update();}
		  
		SumX += Ag[0]; SumY += Ag[1]; SumZ += Ag[2];
	}

	Ax_offset += SumX / times;
  Ay_offset += SumY / times;
  Az_offset += SumZ / times;

}

/* ------------------------- Attitude calculation + Kalman filter ------------------------- */
float IMU::Attitude_Update(){
  uint8_t n_sample  = 8;
  unsigned long now = millis();                         // current time (ms)

  float dt  = (now - _lastTime) / 1000.0;               // delta time (s)
  _lastTime = now;                                      // last sample time (ms)

  IMU_MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read raw 6-axis values

  float accx = ax / AcceRatio;                          // x-axis acceleration
  float accy = ay / AcceRatio;                          // y-axis acceleration
  float accz = az / AcceRatio;                          // z-axis acceleration

  /* ########################## Attitude calculation ########################## */
  // Calculation 1
  aax = atan(accy / accz) * 180 / PI;       // angle of y-axis relative to z-axis
  aay = atan(accx / accz) * 180 / PI;       // angle of x-axis relative to z-axis
  // aaz = atan(accz / accy) * 180 / PI;       // angle of z-axis relative to y-axis

  // Calculation 2
  // aax = atan(accx / sqrt(accz*accz + accy*accy)) * 180 / PI;                // angle of x-axis relative to z-axis
  // aay = atan(accy / sqrt(accz*accz + accx*accx)) * 180 / PI;                // angle of y-axis relative to z-axis
  aaz = atan(accz / sqrt(accx*accx + accy*accy)) * 180 / PI;                // angle of z-axis relative to x/y plane
  /* ############################################################## */

  aax_sum = 0;                                          // Sliding weighted filter for accelerometer raw data
  aay_sum = 0;
  aaz_sum = 0;

  for(uint8_t i=1;i<n_sample;i++){
      aaxs[i-1] = aaxs[i]; aax_sum += aaxs[i] * i;
      aays[i-1] = aays[i]; aay_sum += aays[i] * i;
      aazs[i-1] = aazs[i]; aaz_sum += aazs[i] * i;
  }
  
  aaxs[n_sample-1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0;      // Scale angle to 0-90°

  aays[n_sample-1] = aay;                             // Empirically chosen coefficient applied here
  aay_sum += aay * n_sample;                          // coefficient used in this example: 9/7
  aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0*(90/87);

  aazs[n_sample-1] = aaz; 
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

  float gyrox = - (gx-gxo) / GyroRatio * dt;          // x-axis angular velocity
  float gyroy = - (gy-gyo) / GyroRatio * dt;          // y-axis angular velocity
  float gyroz = - (gz-gzo) / GyroRatio * dt;          // z-axis angular velocity

  agx += gyrox;                                       // integrate x-axis angular velocity
  agy += gyroy;                                       // integrate y-axis angular velocity
  agz += gyroz;                                       // integrate z-axis angular velocity
  
  /* Kalman start */
  Sx = 0; Rx = 0;
  Sy = 0; Ry = 0;
  Sz = 0; Rz = 0;
  
  for(int i=1;i<10;i++){   
      //Measurement average calculation, i.e. average acceleration
      a_x[i-1] = a_x[i];Sx += a_x[i];
      a_y[i-1] = a_y[i];Sy += a_y[i];
      a_z[i-1] = a_z[i];Sz += a_z[i];
  }
  
  a_x[9] = aax;Sx += aax;Sx /= 10;                                           //x-axis acceleration average
  a_y[9] = aay;Sy += aay;Sy /= 10;                                           //y-axis acceleration average
  a_z[9] = aaz;Sz += aaz;Sz /= 10;                                           //z-axis acceleration average

  for(int i=0;i < 10;i++){
    Rx += sq(a_x[i] - Sx);
    Ry += sq(a_y[i] - Sy);
    Rz += sq(a_z[i] - Sz);
  }
  
  Rx = Rx / 9;                                        //Get variance
  Ry = Ry / 9;                        
  Rz = Rz / 9;

  Px = Px + 0.0025;                                   
  Kx = Px / (Px + Rx);               //Calculate Kalman gain
  agx = agx + Kx * (aax - agx);      //Gyro angle and accelerometer velocity fusion
  Px = (1 - Kx) * Px;                //Update p value

  Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy); 
  Py = (1 - Ky) * Py;

  Pz = Pz + 0.0025;
  Kz = Pz / (Pz + Rz);
  agz = agz + Kz * (aaz - agz); 
  Pz = (1 - Kz) * Pz;

  /* Kalman end */
  // Store angle
  Ag[0] = agx - Ax_offset;      // Roll  		
  Ag[1] = agy - Ay_offset;      // Pitch										 					
  Ag[2] = agz - Az_offset;      // Yaw

  Ac[0] = accx - axo/AcceRatio;   Ac[1] = accy - ayo/AcceRatio;   Ac[2] = accz - azo/AcceRatio;   // Store acceleration
  Gy[0] = gyrox- gxo/GyroRatio;   Gy[1] = gyroy - gyo/GyroRatio;  Gy[2] = gyroz- gzo/GyroRatio;   // Store angular velocity
}

/*
* Serial Debug
*/
void IMU::InfosPrint(bool ag,bool ac,bool gy){

  if(dmpReady)  {DMP_Update();ac = 0;gy = 0;}
  else          {Attitude_Update();}

	if(ag)	Serial.printf("P :%6.2f - R :%6.2f - Y :%6.2f   |   "   ,Ag[1],Ag[0],Ag[2]);
 	if(ac)	Serial.printf("Ax:%6.2f - Ay:%6.2f - Az:%6.2f   |   "   ,Ac[0],Ac[1],Ac[2]);
  if(gy)	Serial.printf("Gx:%6.2f - Gy:%6.2f - Gz:%6.2f\t"        ,Gy[0],Gy[1],Gy[2]);

  Serial.println();
}

