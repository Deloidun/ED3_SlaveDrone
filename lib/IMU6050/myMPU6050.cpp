#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>


///////////////////////////////////////////////////
//PARAMETERS DECLARATION
///////////////////////////////////////////////////
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
int RateCalibrationNumber;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;


///////////////////////////////////////////////////
//FUNCTION TO SETUP THE MPU
///////////////////////////////////////////////////
void IMU_Setup(){
    Wire.setClock(400000);
    Wire.begin();

    //Wake up MPU6050
    Wire.beginTransmission(0x68); //ID address of MPU6050
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}


///////////////////////////////////////////////////
//FUNCTION TO INITIALIZE, MEASURE ANGULAR VELOCITY
///////////////////////////////////////////////////
void GyroSignal(void){
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); //Register 0x1A
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);

    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);

    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    RateRoll = (float)GyroX/65.5;
    RatePitch = (float)GyroY/65.5;
    RateYaw = (float)GyroZ/65.5;

    AccX = (float)AccXLSB/4096;
    AccY = (float)AccYLSB/4096;
    AccZ = (float)AccZLSB/4096;

    AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ)) * 1/(3.142/180);
    AnglePitch =-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ)) * 1/(3.142/180);
}

///////////////////////////////////////////////////
//FUNCTION TO CALIBRATE MPU6050
///////////////////////////////////////////////////
void MPU_Calibration(){
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber ++){
        GyroSignal(); //Call back function
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
}


///////////////////////////////////////////////////
//FUNCTION TO CONTINUOUSLY UPDATE
///////////////////////////////////////////////////
void MPU_Value_Update(){
    GyroSignal();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
}


///////////////////////////////////////////////////
//FUNCTION TO PRINT RESULTS FOR DEBUGGING
///////////////////////////////////////////////////
void Print_MPU_Angle(){
    Serial.print("\nRoll Angle >> ");
    Serial.print(AngleRoll);
    Serial.print(" degrees");
    Serial.print ("     ");
    Serial.print("Pitch Angle >> ");
    Serial.print(AnglePitch);
    Serial.print(" degrees");
}

void Print_MPU_Rate(){
    Serial.print("\nRRoll >> ");
    Serial.print(RateRoll);
    Serial.print(" deg/sec");
    Serial.print ("      ");
    Serial.print("RPitch >> ");
    Serial.print(RatePitch);
    Serial.print(" deg/sec");
    Serial.print ("      ");
    Serial.print("RYaw >> ");
    Serial.print(RateYaw);
    Serial.print(" deg/sec");
}