#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>

#include <SlaveEsp32WifiReceiver.h>
#include <myMotorSetup.h>
#include <SystemSetup.h>
#include <myMPU6050.h>
#include <myPID_Controller.h>
#include <myBMP280.h>

#include <DisableReboot.h>


// ///////////////////////////////////////////////
// //TURN THIS ON FOR TESTING BAROMETER
// ///////////////////////////////////////////////
// void setup(){
//   Initialize_Serial();
//   IMU_Setup();
//   MPU_Calibration();
//   BMP280_Setup();
//   // BMP280_Check();
//   BMP280_Calibration();
//   Matrix_Manipulation();
// }
// void loop(){
//   GyroSignal();
//   Kalman_1D_Roll();
//   Kalman_1D_Pitch();
//   BMP280_Update_Values();
// }



void setup(){
  Initialize_Serial();
  Init_ESPNOW_Slave();
  Initialize_ESC();
  IMU_Setup();
  MPU_Calibration();
  // BMP280_Setup();
  // BMP280_Check(); //Turn this on when change to SPI
  // BMP280_Calibration();
  // Matrix_Manipulation();

}

void loop(){
  MPU_Value_Update();
  Kalman_1D_Roll();
  Kalman_1D_Pitch();
  // BMP280_Update_Values(); //Included Kalman 2D
  // Realtime_Print(); //Print altitude for now
  sendingData_throughESPNOW();
  
  Values_Update();

  PID_Equation_AngleRoll();
  PID_Equation_AnglePitch();
  PID_Equation_RateRoll();
  PID_Equation_RatePitch();
  PID_Equation_RateYaw();



  Control_Throttle_To_Balance_Drone();
  // Print_PS5_Values();

  // Print_MPU_Angle();
  // Print_MPU_Rate();
  Serial.printf("%d, %d, %3.0f, %3.0f \n", JS_X_Value,JS_Y_Value,Convert_Received_PS4_To_Throttle(),PrevItermAngleRoll);
  // Print_PS5_Values();
} 

// ////////////////////////////////////////
// //USE THIS CODE WHEN CALIBRATING ESC
// ////////////////////////////////////////
// void setup(){
//   Initialize_Serial();
//   Init_ESPNOW_Slave();
//   Calibrate_ESC();
// }

// void loop(){
//   Send_Input_Throttle_2_ESC();
// }