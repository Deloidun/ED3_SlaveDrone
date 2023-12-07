#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

extern byte JS_X_Value;
extern byte JS_Y_Value;
extern byte LeftButton;
extern byte RightButton;
extern int PWM_Value;

void Init_ESPNOW_Slave();
void Print_PS5_Values();
// void SendingData_ThroughESPNOW();