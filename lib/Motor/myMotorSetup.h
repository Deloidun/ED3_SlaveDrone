#pragma once

extern float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

float Convert_Received_PS4_To_Throttle();
float Convert_Received_PS4_To_Roll();
float Convert_Received_PS4_To_Pitch();
float Convert_Received_PS4_To_Yaw();


void Send_Input_Throttle_2_ESC();
void Initialize_ESC();
void Calibrate_ESC();
void Control_Throttle_To_Balance_Drone();