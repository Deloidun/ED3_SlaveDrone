#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>

#include <myMPU6050.h>
#include <myMotorSetup.h>
#include <SystemSetup.h>
#include <SlaveEsp32WifiReceiver.h>
#include <myBMP280.h>


float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput [] = {0, 0};
float AltitudeKalman = 0;
float VelocityVerticalKalman;
float AccZ_Inertial;

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;


// float PRateRoll = 0.4285; float PRatePitch=PRateRoll; float PRateYaw = 0.0; 
// float IRateRoll = 0.065; float IRatePitch=IRateRoll; float IRateYaw = 2.0;
// float DRateRoll = 0.0375; float DRatePitch=DRateRoll; float DRateYaw = 0.0;


// //PID gains for position (angle)
// float PAngleRoll = 0.325; float PAnglePitch = PAngleRoll;
// float IAngleRoll = 0.01; float IAnglePitch = IAngleRoll;
// float DAngleRoll = 0.0; float DAnglePitch = DAngleRoll; 

float PRateRoll = 0.0; float PRatePitch=PRateRoll; float PRateYaw = 0.0; 
float IRateRoll = 0.0; float IRatePitch=IRateRoll; float IRateYaw = 0.0;
float DRateRoll = 0.0; float DRatePitch=DRateRoll; float DRateYaw = 0.0;


//PID gains for position (angle)
float PAngleRoll = 0.0; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.0; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.0; float DAnglePitch = DAngleRoll; 


////////////////////////////////////////////////
//GENERAL FUNCTION OF KALMAN FILTER TECHNIQUE
////////////////////////////////////////////////
void Kalman_1D(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1/(1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}


////////////////////////////////////////////////////////////////////////
//GENERAL FUNCTION OF KALMAN FILTER FOR ALTITUDE AND VERTICLE VELOCITY
////////////////////////////////////////////////////////////////////////
void Kalman_2D (void){
    Acc = {AccZ_Inertial};
    S = F * S + G * Acc;
    P = F * P *~ F + Q;
    L = H * P *~ H + R;
    K = P *~ H * Invert(L);
    M = {Altitude};
    S = S + K * (M - H * S);
    AltitudeKalman = S(0,0);
    VelocityVerticalKalman = S(1,0);
    P = (I - K * H) * P;
}


//////////////////////////////////////////////////////////
//GENERAL FUNCTION OF PID FOR POSITION AND VELOCITY
//////////////////////////////////////////////////////////
void General_PID_Equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
    float Pterm = P * Error; //P Controller
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004/2; //I Controller

    //Set the limit for I interial controller
    float PIDGainLimit = 400;
    if (Iterm > PIDGainLimit) Iterm = PIDGainLimit;
    else if (Iterm <- PIDGainLimit) Iterm =- PIDGainLimit;

    float Dterm = D * (Error - PrevError) / 0.004; //D Controller
    float PIDOutput = Pterm + Iterm + Dterm; //PID Output is the sum of controllers

    //Set the limit for the PID output
    if (PIDOutput > PIDGainLimit) PIDOutput = PIDGainLimit;
    else if (PIDOutput <- PIDGainLimit) PIDOutput =- PIDGainLimit;

    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}


//////////////////////////////////////////////////////////
//FUNCTION TO RESET PID AFTER EVERY EXECUTION
//////////////////////////////////////////////////////////
void Reset_PID(void){
    //Setpoints for velocity or we can say, the desired velocity so that PID controller can refer back to control the current result.
    PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;

    //Setpoint for position (angle)
    PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;    
    PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}


//////////////////////////////////////////////////////////
//FUNCTION THAT IMPLEMENT TO FILTER ROLL
//////////////////////////////////////////////////////////
void Kalman_1D_Roll(){
    Kalman_1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
}


//////////////////////////////////////////////////////////
//FUNCTION THAT IMPLEMENT TO FILTER PITCH
//////////////////////////////////////////////////////////
void Kalman_1D_Pitch(){
    Kalman_1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
}


//////////////////////////////////////////////////////////
//FUNCTION UPDATES IMPORTANT VALUES
//////////////////////////////////////////////////////////
void Values_Update(){
    DesiredAngleRoll =  0.1*(Convert_Received_PS4_To_Roll() - 127);
    DesiredAnglePitch = 0.1*(Convert_Received_PS4_To_Pitch()- 127);
    DesiredRateYaw = 0.15 * (Convert_Received_PS4_To_Yaw());
    InputThrottle = Convert_Received_PS4_To_Throttle();
    
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR ROLL ANGLE
//////////////////////////////////////////////////////////
void PID_Equation_AngleRoll(){
    General_PID_Equation (ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll,PrevItermAngleRoll);     
    DesiredRateRoll = PIDReturn[0]; // not avaliable
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];
    
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR PITCH ANGLE
//////////////////////////////////////////////////////////
void Error_Update(){
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;
}
void PID_Equation_AnglePitch(){
    General_PID_Equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];
    Error_Update();
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR RATE ROLL
//////////////////////////////////////////////////////////
void PID_Equation_RateRoll(){
    General_PID_Equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1]; 
    PrevItermRateRoll = PIDReturn[2];
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR RATE PITCH
//////////////////////////////////////////////////////////
void PID_Equation_RatePitch(){
    General_PID_Equation (ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0]; 
    PrevErrorRatePitch = PIDReturn[1]; 
    PrevItermRatePitch = PIDReturn[2];
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR RATE YAW
//////////////////////////////////////////////////////////
void PID_Equation_RateYaw(){
    General_PID_Equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[0];
    PrevItermRateYaw = PIDReturn[1];
}