#include <ESP32Servo.h>
#include <Slave_ESP_NOW.h>

#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition
#define MOTOR_PIN 5    // Pin 5 attached to the ESC signal pin

Servo servo1;     
// Servo servo2;     
// Servo servo3;     
// Servo servo4;

void init_ESC()
{
  servo1.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL);
  servo1.writeMicroseconds(MIN_SIGNAL);
}

void writePWM()
{
  servo1.write(ESC_PWM);
}