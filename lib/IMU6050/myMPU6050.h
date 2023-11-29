#pragma once

extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;
extern int RateCalibrationNumber;
extern float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

void IMU_Setup();
void GyroSignal();
void MPU_Calibration();
void MPU_Value_Update();
void Print_MPU_Angle();
void Print_MPU_Rate();