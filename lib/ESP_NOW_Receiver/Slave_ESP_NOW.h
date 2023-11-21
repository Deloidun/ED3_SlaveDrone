#pragma once

extern int ESC_PWM;
extern byte X_joystick_value;
extern byte Y_joystick_value;
extern byte button1_value;
extern byte button2_value;

void init_ESPNOW_Slave();
void checkSignal();