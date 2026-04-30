#ifndef _IMU_H
#define _IMU_H

#include <Arduino.h>
// #include <MPU6050.h>

#define AcceRatio 16384.0          // Accelerometer scale factor
#define GyroRatio 131.0            // Gyroscope scale factor

#define DEG 0 .017453292519943295769236907684886f    // Degrees to radians
#define RAD 57.295779513082320876798154814105f	    // Radians to degrees

class IMU
{
    private:
      unsigned long _lastTime = 0;
      
      int16_t ax, ay, az, gx, gy, gz;                     //Accelerometer and gyroscope raw data
      float aax=0,aay=0,aaz=0;                            //Angle variable
      float agx=0,agy=0,agz=0;                            //Angle variable
      long  axo=0,ayo=0,azo=0;                            //Accelerometer offset
      long  gxo=0,gyo=0,gzo=0;                            //Gyroscope offset
       
      float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};  // x,y axis sample queues
      long aax_sum, aay_sum,aaz_sum;                      // x,y axis sample sums
       
      float a_x[10]={0}, a_y[10]={0},a_z[10]={0};
      float g_x[10]={0} ,g_y[10]={0},g_z[10]={0};         //AccelerometerCovariance calculation queue
      float Px=1, Rx, Kx, Sx, Vx, Qx;                     //X-axis Kalman variables
      float Py=1, Ry, Ky, Sy, Vy, Qy;                     //Y-axis Kalman variables
      float Pz=1, Rz, Kz, Sz, Vz, Qz;                     //Z-axis Kalman variables      

      float Ax_offset = 0,Ay_offset = 0 , Az_offset = 0;  // Initial offset angles    
      float SumX = 0,     SumY = 0,       SumZ = 0;

      /* ------------------------------ DMP ------------------------------ */
      bool      dmpReady = false; 
      uint8_t   devStatus;    
      uint8_t   mpuIntStatus;     // Save actual interrupt status byte from MPU
      uint8_t   fifoBuffer[64];   // FIFO storage buffer    
      uint16_t  packetSize;       // Expected DMP packet size (default42bytes)
      uint16_t  fifoCount;        // Count of all current bytes in FIFO

      float ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll

    public:

      float Ag[3],Ac[3],Gy[3];

      void begin();

      /* ---------------------------- Offset calibration ------------------------------ */
      void Acc_Gyro_Calibration(uint16_t times);          //Initial sampling
      void Ypr_Calibration(uint16_t times);               //Offset calibration
      float Attitude_Update();                            // Attitude computation + Kalman filter
      
      void InfosPrint(bool ag,bool ac,bool gy);           // Serial data debug

      /* ------------------------------ DMP ------------------------------ */
      void InitDMP();
      void DMP_Update();
};

#endif
