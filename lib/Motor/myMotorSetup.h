#pragma once

extern float Convert_Received_PS4_To_Throttle();
extern float Convert_Received_PS4_To_Roll();
extern float Convert_Received_PS4_To_Pitch();
extern float Convert_Received_PS4_To_Yaw();


void Send_Input_Throttle_2_ESC();
void Initialize_ESC();
void Calibrate_ESC();
void Control_Throttle_To_Balance_Drone();