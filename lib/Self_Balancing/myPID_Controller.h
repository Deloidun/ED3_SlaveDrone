#pragma once

extern float InputThrottle, InputRoll, InputPitch, InputYaw;
extern float KalmanAnglePitch, KalmanAngleRoll;
extern float AltitudeKalman;
extern float AccZ_Inertial;
extern float DesiredAngleRoll, DesiredAnglePitch, DesiredRatePitch, ErrorAngleRoll,ErrorAnglePitch,DesiredRateRoll,DesiredRatePitch;

extern float ErrorRatePitch,PrevItermAngleRoll,PrevErrorAngleRoll;
extern float PrevErrorRatePitch;
extern float PrevItermRatePitch;


void Reset_PID();
void Kalman_1D_Roll();
void Kalman_1D_Pitch();
void Kalman_2D();
void Values_Update();
void General_PID_Equation(float Error, float P, float I, float D, float PrevError, float PrevIterm);
void PID_Equation_AngleRoll();
void PID_Equation_AnglePitch();
void PID_Equation_RateRoll();
void PID_Equation_RatePitch();
void PID_Equation_RateYaw();

