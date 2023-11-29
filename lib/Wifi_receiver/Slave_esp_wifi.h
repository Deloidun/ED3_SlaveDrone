#pragma once

extern int ESC_PWM;
extern byte X_joystick_value;
extern byte Y_joystick_value;
extern byte leftB_value;
extern byte rightB_value;

void init_ESPNOW_Slave();
void checkSignal();