#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BasicLinearAlgebra.h>

#include <myMPU6050.h>
#include <myPID_Controller.h>

#define BMP_SCL 18
#define BMP_SDO 19
#define BMP_SDA 23
#define BMP_CSB1 5

float Altitude, AltitudeBarometer, AltitudeBarometerStartUp;
float LoopTimer;

Adafruit_BMP280 bmp (BMP_CSB1, BMP_SDA, BMP_SDO, BMP_SCL); //SPI

using namespace BLA;
BLA :: Matrix <2,2> F; BLA :: Matrix <2,1> G;

BLA :: Matrix <2,2> P; BLA :: Matrix <2,2> Q;

BLA :: Matrix <2,1> S; BLA :: Matrix <1,2> H;

BLA :: Matrix <2,2> I; BLA :: Matrix <1,1> Acc;

BLA :: Matrix <2,1> K; BLA :: Matrix <1,1> R;

BLA :: Matrix <1,1> L; BLA :: Matrix <1,1> M;


////////////////////////////////////////////////
//FUNCTION TO INITIALIZE BMP280
////////////////////////////////////////////////
void BMP280_Setup(void){
    //Wake up BMP280
    Wire.beginTransmission(0x76); // BMP280 I2C address
    Wire.write(0xF4); //Register address for the control register
    Wire.write(0x57); //Wake up BMP280 and configure it for normal operation
    Wire.endTransmission();

    Wire.beginTransmission(0x76);
    Wire.write(0xF5);
    Wire.write(0x14);
    Wire.endTransmission();
}


////////////////////////////////////////////////
//FUNCTION TO CHECK THE PRESENT OF BMP280
////////////////////////////////////////////////
void BMP280_Check(){
    Serial.println("Scanning for BMP280 sensor...");
    //Check available of BMP280
    if(!bmp.begin()){
        Serial.println("BMP280 sensor was not found!");
        while (1);
    }
    Serial.println("Initialize BMP280 completed!");
}


////////////////////////////////////////////////
//FUNCTION TO CALIBRATE BMP280
////////////////////////////////////////////////
void BMP280_Calibration(){
    for (int BaroRateCalibrationNumber = 0; BaroRateCalibrationNumber < 2000; BaroRateCalibrationNumber ++){
        float AltitudeBarometer = bmp.readAltitude(1023);
        AltitudeBarometerStartUp += AltitudeBarometer;  
    } AltitudeBarometerStartUp /= 2000; //meter
}


////////////////////////////////////////////////
//FUNCTION TO MANIPULATE MATRIX
////////////////////////////////////////////////
void Matrix_Manipulation(){
    F = {1, 0.004, 0, 1};  
    G = {0.5 * 0.004 * 0.004, 0.004};
    H = {1, 0};
    I = {1, 0, 0, 1};
    Q = G *~ G * 10 * 10;
    R = {80*80};
    P = {0, 0, 0, 0};
    S = {0, 0};
}


////////////////////////////////////////////////
//FUNCTION TO UPDATE BMP280 VALUES
////////////////////////////////////////////////
void BMP280_Update_Values(){
    AccZ_Inertial =-sin(KalmanAnglePitch*(3.142/180))*AccX+cos(KalmanAnglePitch*(3.142/180))*sin(KalmanAngleRoll*(3.142/180))* AccY+cos(KalmanAnglePitch*(3.142/180))*cos(KalmanAngleRoll*(3.142/180))*AccZ;   
    AccZ_Inertial = (AccZ_Inertial - 1) * 9.81 * 100;
    Altitude = bmp.readAltitude(1023);
    Altitude -= AltitudeBarometerStartUp;
    Altitude *= 100; //Convert from m to cm
    Kalman_2D();
    Serial.println (String(LoopTimer/1000) + "      Altitude >> " + String(Altitude));
    LoopTimer = micros();
}



