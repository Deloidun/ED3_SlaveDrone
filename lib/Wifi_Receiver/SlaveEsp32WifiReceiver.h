#pragma once

extern byte JS_X_Value;
extern byte JS_Y_Value;
extern byte leftButton;
extern byte rightButton;
extern int PWM_value;

void Init_ESPNOW_Slave();
void Print_PS5_Values();
void sendingData_throughESPNOW();