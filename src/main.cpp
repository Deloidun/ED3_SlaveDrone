#include <Arduino.h>
#include <Slave_esp_wifi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <PID_controller.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  init_ESC();
  init_ESPNOW_Slave();
  // calibrate();
}

void loop()
{
  checkSignal();
  // corrected_values();
  kalman_1d_roll();
  kalman_1d_pitch();
  value_update();
  pid_equation_angleroll();
  pid_equation_anglepitch();
  pid_equation_rateroll();
  pid_equation_ratepitch();
  pid_equation_rateyaw();
  control_throttle();
  printf("%.1f   %.1f   %.1f   %.1f   %.1f\n", InputThrottle, MotorInput1, MotorInput2, MotorInput3, MotorInput4);
}