#pragma once

extern float InputThrottle;
extern float InputRoll, InputPitch, InputYaw;
extern float KalmanAnglePitch, KalmanAngleRoll;
extern float AltitudeKalman;
extern float AccZ_Inertial;

void Reset_PID(void);
void Kalman_1D_Roll();
void Kalman_1D_Pitch();
void Kalman_2D();
void Values_Update();

void PID_Equation_AngleRoll();
void PID_Equation_AnglePitch();
void PID_Equation_RateRoll();
void PID_Equation_RatePitch();
void PID_Equation_RateYaw();
