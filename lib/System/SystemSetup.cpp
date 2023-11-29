#include <Arduino.h>
#include <Wire.h>

unsigned long time_prev = 0;

void Initialize_Serial(){
    Serial.begin(115200);
    while (!Serial);
}