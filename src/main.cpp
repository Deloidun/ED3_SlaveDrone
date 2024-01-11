#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <PID_controller.h>
#include <Slave_esp_wifi.h>
#include <Voltage.h>


// Adafruit_MPU6050 mpu;
uint32_t LoopTimer;
void setup()
{
  Serial.begin(115200);
  // while (!Serial);
  // Set up EPS-NOW for slave ESP3
  init_ESPNOW_Slave();
  init_ESC();
  system_setup();
  calibration_measurement(); //MPU6050
  gyro_compensate();
  LoopTimer = micros();
  // gps_setup();
}

void loop()
{
  // displayLocation();
  trans();
  TimeCount();
  voltage_sensor();
  corrected_values();
  kalman_1d_roll();
  kalman_1d_pitch();
  value_update();

  pid_equation_angleroll();
  pid_equation_anglepitch();
  pid_equation_rateroll();
  pid_equation_ratepitch();
  pid_equation_rateyaw();

  control_throttle();
  sendingData(); //Send sensor data


  while(micros() - LoopTimer < 4000);
  LoopTimer = micros();

  MotorInputPrint();
}



// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial);
//   calibrate(); //Turn this on when calibrate
// }

// void loop()
// {
// }