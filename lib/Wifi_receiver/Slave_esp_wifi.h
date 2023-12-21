#pragma once

extern int PWM;

extern byte X_value;
extern byte Y_value;

extern byte leftB;
extern byte rightB;

extern float PRate;
extern float IRate;
extern float DRate;

extern float PAngle;
extern float IAngle;
extern float DAngle;

void init_ESPNOW_Slave();
void sendingData();
void Print_PS4_Value ();
void TimeCount();