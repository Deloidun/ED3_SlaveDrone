#pragma once

extern float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// extern float Convert_Received_PS4_To_Throttle();
// extern float Convert_Received_PS4_To_Roll();
// extern float Convert_Received_PS4_To_Pitch();
// extern float Convert_Received_PS4_To_Yaw();

extern float RecevingThrottleInput, RecevingPitchInput, RecevingYawInput, RecevingRollInput;


void Send_Input_Throttle_2_ESC();
void Initialize_ESC();
void Calibrate_ESC();
void Control_Throttle_To_Balance_Drone();