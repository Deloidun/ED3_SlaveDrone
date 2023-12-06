#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <myBMP280.h>
#include <myPID_Controller.h>


void Initialize_Serial(){
    Serial.begin(115200);
    while(!Serial);
}

void Realtime_Print(){
    Serial.println(String(LoopTimer/1000000) + "s" + "      Altitude >> " + String(AltitudeKalman) + " cm");
    LoopTimer = micros();
}