#include <SlaveESP32Wifi.h>
#include <myPID_Controller.h>

void setup(){
  Init_ESPNOW_Slave();
  Init_Serial();
  Init_ESPNOW_Slave();
  Initialize_ESC();
  IMU_Setup();
  MPU_Calibration();
}

void loop(){
  MPU_Value_Update();
  Kalman_1D_Roll();
  Kalman_1D_Pitch();
  Values_Update();
  PID_Equation_AngleRoll();
  PID_Equation_AnglePitch();
  PID_Equation_RateRoll();
  PID_Equation_RatePitch();
  PID_Equation_RateYaw();
  Control_Throttle_To_Balance_Drone();
  Print_PS5_Values();
  // Print_MPU_Angle();
}