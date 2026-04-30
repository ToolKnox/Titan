#ifndef _MOTIONCONTROL_H_
#define _MOTIONCONTROL_H_

#include "../Drive/Hiwonder.h"
#include "FKIK.h"       // FK/IK library
#include <WiFiClient.h>
#include "../Drive/IOs.h"

#define SerNum  8

// Cycloid parameters
#define XS      90.0    // X origin   40 ~ 140             (50)
#define YS      -130.0  // Y origin   x = 90:-88 ~ -172    (42)
#define XMOVE   40      // X distance
#define H       20      // Lift height

class Robot
{
    public:
        bool RobotStatus = 1;      // 0->Tracked form; 1->Quadruped form

        void begin();       // Initialize

        uint16_t SPos[SerNum]    = {};  // Position storage
        uint16_t SVin[SerNum]    = {};  // Voltage storage
        uint8_t STemp[SerNum]    = {};  // Temperature storage

        uint8_t legNum = 1;             // Leg index
        
        /* ------------------ Mode control ------------------ */
        
        void FK_ResetQuadruped();       // Quadruped gait - FK pose
        void IK_ResetQuadruped();       // Quadruped gait - IK state

        void ResetTrack(bool status);   // Tracked drive mode - base mode
        void FK_TStatus_HIGH();         // Tracked drive mode - high posture
        void FK_TStatus_HIGHER();       // Tracked drive mode - highest posture

        void FK_LEFT();                 // Lean left
        void FK_RIGHT();                // Lean right

        /* ------------------ Unload control ------------------ */
        void PowerOff();                // Unload all
        void LegUnload(uint8_t legNum); // Unload single leg

        /* ------------------ Single leg control ------------------ */
        uint16_t D2V(float angle);
        // Read
        float RUAngle[2] = {};          // Angle storage
        float LUAngle[2] = {};
        float RBAngle[2] = {};
        float LBAngle[2] = {};
        
        void RU_ReadPOS();
        void LU_ReadPOS();
        void RB_ReadPOS();
        void LB_ReadPOS();

        // Forward kinematics
        void FK_LUMove(float posk,float posx,uint16_t Time);    // Left front leg
        void FK_RUMove(float posk,float posx,uint16_t Time);    // Right front leg
        void FK_LBMove(float posk,float posx,uint16_t Time);    // Left rear leg
        void FK_RBMove(float posk,float posx,uint16_t Time);    // Right rear leg

        void FK_LegMove(float Angle[],uint8_t legNum,uint16_t DSD,bool Print);   // Single leg FK

        void FK_LSMove(float posk,uint16_t Time);               // Left side pair legs
        void FK_RSMove(float posk,uint16_t Time);               // Right side pair legs
        // Inverse kinematics
        float IK_RUPoint[2] = {};// Coordinate point storage
        float IK_LUPoint[2] = {};
        float IK_RBPoint[2] = {};
        float IK_LBPoint[2] = {};

        void IK_LUMove(float x,float y,uint16_t Time);// Left front leg
        void IK_RUMove(float x,float y,uint16_t Time);// Right front leg
        void IK_LBMove(float x,float y,uint16_t Time);// Left rear leg
        void IK_RBMove(float x,float y,uint16_t Time);// Right rear leg

        void IK_LegMove(float point[],uint8_t LEGNum,uint16_t DSD,bool Print);// Single leg IK

        void LegPointDebug(float point[],uint8_t cmd,byte offset,bool PrintPoint,bool PrintAngle);// Single leg debug

        /* --- Cycloid --- */
        float F1_CPoints[20][2] = {};// Forward cycloid 1
        float F2_CPoints[20][2] = {};// Forward cycloid 2
        float F3_CPoints[20][2] = {};// Forward cycloid 3

        float B1_CPoints[20][2] = {};// Backward cycloid 1
        float B2_CPoints[20][2] = {};// Backward cycloid 2
        float B3_CPoints[20][2] = {};// Backward cycloid 3

        void GetCycloidPoints(float CPoints[][2],float xs,float xf,float ys,float yh);// Get cycloid trajectory points
        void LegCycloid(float CPoints[][2],uint8_t LEGNum);                           // Foot cycloid motion
        /* ------------------ Posture control ------------------ */
        void PosMove(float xMove,float yMove,uint16_t Time);// Posture linear relative movement
        void PosAction1();                                  // Posture action 1

        void PosToPoint(float x,float y,uint16_t Time); // Posture coordinate position
        void PosToPitch(float deg,uint16_t Time);       // Posture pitch angle
        void PosToRoll (float deg,uint16_t Time);       // Posture roll angle
        void PosAction2(uint8_t Num);                   // Posture action 2
        
        void PosAction3();                   // Posture action 3
        void PosAction4();                   // Posture action 4
        /* ------------------ Motion control ------------------ */
        bool TrotStatus = 0;      // 0->stop; 1->trot
        bool WalkStatus = 0;      // 0->stop; 1->walk

        void Trot();        // Trot gait
        void Walk();        // Walk gait
        void Walk_Basic(uint8_t SetpNum,bool dir);        // Walk basic gait


        void VMC(float j0,float j1,float VMCPoint[],float VMCPoint_last[]);// VMC algorithm framework (testing)

        /* ------------------ Action design ------------------ */
        void Hello();// Greet

        /* ------------------ Information extraction ------------------ */
        void GetServoInfo(bool Pos,bool Vin,bool Temp);// Get data: 0->position;1->voltage;2->temperature
};

#endif
