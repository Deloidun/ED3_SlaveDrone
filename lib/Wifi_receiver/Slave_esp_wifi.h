#pragma once

extern int PWM;
extern byte X_value;
extern byte Y_value;
extern byte leftB;
extern byte rightB;

void init_ESPNOW_Slave();
void sendingData();
void Print_PS4_Value ();
void TimeCount();