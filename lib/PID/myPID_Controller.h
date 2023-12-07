#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>
#include <SlaveESP32Wifi.h>



////////////////////////////////////////
//IMU FUNCTIONS
////////////////////////////////////////
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


////////////////////////////////////////
//MOTOR FUNCTIONS
////////////////////////////////////////
extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;
extern int RateCalibrationNumber;
extern float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

float Convert_Received_PS4_To_Throttle();
float Convert_Received_PS4_To_Roll();
float Convert_Received_PS4_To_Pitch();
float Convert_Received_PS4_To_Yaw();

void Send_Input_Throttle_2_ESC();
void Initialize_ESC();
void Calibrate_ESC();
void Control_Throttle_To_Balance_Drone();


////////////////////////////////////////
//PID FUNCTIONS
////////////////////////////////////////
extern float InputThrottle, InputRoll, InputPitch, InputYaw;
extern float KalmanAnglePitch, KalmanAngleRoll;
extern float DesiredAngleRoll, DesiredAnglePitch, DesiredRateRoll, DesiredRatePitch;
extern float ErrorRatePitch;
extern float ErrorRateRoll;
extern float PrevItermAngleRoll, PrevErrorRateRoll;
extern float PrevItermAnglePitch, PrevErrorRatePitch;

void Init_Serial();
void Kalman_1D_Roll();
void Kalman_1D_Pitch();
void Values_Update();
void General_PID_Equation(float Error, float P, float I, float D, float PrevError, float PrevIterm);
void PID_Equation_AngleRoll();
void PID_Equation_AnglePitch();
void PID_Equation_RateRoll();
void PID_Equation_RatePitch();
void PID_Equation_RateYaw();
